"""Autonomous RGB-only active mapping, with explicit finite stopping criteria.
Motion depends only on live laser/map/odometry/TF. No ground truth or depth.
"""
import argparse,json,math,os,signal,subprocess,time,fcntl,shutil
from pathlib import Path
import numpy as np,rclpy
from sensor_msgs.msg import CompressedImage,Imu
from geometry_msgs.msg import TwistStamped,PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from explore import Explorer,wrap
from active_plan import choose,completion

ROOT=Path(__file__).resolve().parent

def atomic(path,value):
    t=path.with_suffix(path.suffix+'.tmp');t.write_text(json.dumps(value,indent=2));t.replace(path)
def available_mb():
    for l in Path('/proc/meminfo').read_text().splitlines():
        if l.startswith('MemAvailable:'):return int(l.split()[1])/1024
    return 0

class Autonomous(Explorer):
    def __init__(self,seconds):
        super().__init__(seconds)
        self.views=[];self.failed=[];self.phase='waiting_sensors';self.terminal=None;self.returning=False
        self.target_yaw=None;self.sweep_hold=None;self.last_checkpoint_goal=0;self.checkpoints=[];self.gains=[]
        self.last_publish=-1.;self.last_sent=(0.,0.);self.known_track_ids=self.landmark_ids();self.no_progress=0;self.recoveries=0;self.vision_t=time.monotonic();self.last_rgb=None
        self.lease=self.out/'command_lease.json';self.external=False;self.child=None;self.check_future=None
        self.progress_pub=self.create_publisher(String,'/mono/autonomy_status',1)
        self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.rgb_cb,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,'/cmd_vel',self.command_cb,10)
        self.create_subscription(Imu,'/imu',lambda m:self.receive('imu',m),qos_profile_sensor_data)
        self.saver=self.create_client(Trigger,'/mono/save_visual')
        self.owned=json.loads((self.out/'pids.json').read_text())
        self.owned['autonomy']=os.getpid();atomic(self.out/'pids.json',self.owned)
        self.guard=subprocess.Popen(['python3',str(ROOT/'command_guard.py'),str(os.getpid()),str(self.lease),'--ros-args','-p','use_sim_time:=true'],env=os.environ,stdout=open(self.out/'guard.log','w'),stderr=subprocess.STDOUT,start_new_session=True)
        self.owned['guard']=self.guard.pid;atomic(self.out/'pids.json',self.owned)
        data=json.loads((self.out/'keyframes.json').read_text())
        from geometry import OPTICAL_TO_BASE
        for f in data['frames']:
            T=np.array(f['T']);R=T[:3,:3]@OPTICAL_TO_BASE.T;yaw=np.arctan2(R[1,0],R[0,0]);self.views.append([T[0,3],T[1,3],yaw])
        self.session_start=time.monotonic();self.reason='running';self.last_status=0.

    def send(self,v=0,w=0,heartbeat=True):
        now=time.monotonic()
        if heartbeat and now-getattr(self,'last_publish',-1.)<.05 and not (v==0 and w==0 and getattr(self,'last_sent',(0,0))!=(0,0)):return
        self.last_publish=now;self.last_sent=(v,w)
        m=TwistStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='mono_autonomy';m.twist.linear.x=float(v);m.twist.angular.z=float(w);self.pub.publish(m)
        if heartbeat:
            self.last_command=time.monotonic()
            if hasattr(self,'lease'):atomic(self.lease,{'monotonic':self.last_command,'v':float(v),'w':float(w)})
    def rgb_cb(self,m):self.last_rgb=time.monotonic()
    def command_cb(self,m):
        if m.header.frame_id not in ['mono_autonomy','mono_autonomy_guard']:self.external=True
    def landmark_ids(self):
        p=self.out/'stable/stable_landmarks.json'
        if not p.exists():return set()
        return {int(q['track_id']) for q in json.loads(p.read_text())}
    def status(self,elapsed):
        result={'state':self.phase,'reason':self.reason,'terminal_reason':self.terminal,'elapsed_wall_s':round(elapsed,1),'travelled_m':self.distance,'goals_reached':len(self.goals),'failed_goals':len(self.failed),'recoveries':self.recoveries,'map_metrics':self.metrics,'current_position':None if self.prevpos is None else self.prevpos.tolist(),'accepted_landmarks':len(self.landmark_ids()),'new_track_fractions':self.gains,'checkpoints':self.checkpoints,'dense_3d_complete':False,'limits':{'wall_seconds':self.limit,'maximum_frames':5500},'available_memory_mb':round(available_mb())}
        atomic(self.out/'autonomy_status.json',result);self.progress_pub.publish(String(data=json.dumps(result)))
    def healthy(self):
        now=time.monotonic()
        if self.external:return 'external_velocity_command'
        if any(x not in self.times or now-self.times[x]>.9 for x in ['scan','odom','imu']):return 'stale_motion_sensors'
        if self.last_rgb is None or now-self.last_rgb>2.:return 'stale_rgb'
        if not Path('/proc/'+str(self.owned['vision'])).exists():return 'vision_process_stopped'
        if available_mb()<500:return 'low_memory'
        if shutil.disk_usage(self.out).free<3*1024**3:return 'low_disk_space'
        if (self.out/'STOP').exists():return 'stop_file_requested'
        return None
    def spin_stopped(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end and not self.stopped:
            self.send();rclpy.spin_once(self,timeout_sec=.03)
    def checkpoint(self,final=False):
        self.phase='saving_and_refining';self.send();self.status(time.monotonic()-self.session_start)
        if not self.saver.service_is_ready():self.reason='save_service_missing';return False
        future=self.saver.call_async(Trigger.Request());deadline=time.monotonic()+30
        while not future.done() and time.monotonic()<deadline and not self.stopped:self.spin_stopped(.05)
        if not future.done() or not future.result().success:self.reason='save_failed';return False
        if available_mb()<550:self.reason='memory_insufficient_for_refinement';return False
        env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
        self.child=subprocess.Popen(['python3',str(ROOT/'stable_cached.py'),str(self.out),str(self.out/'stable')],env=env,stdout=open(self.out/'autonomy_refine.log','a'),stderr=subprocess.STDOUT,start_new_session=True)
        end=time.monotonic()+200
        while self.child.poll() is None and time.monotonic()<end and not self.stopped:
            self.spin_stopped(.1)
            if available_mb()<240:self.reason='low_memory_during_refinement';break
        if self.child.poll() is None:
            os.killpg(self.child.pid,signal.SIGTERM);self.child.wait(timeout=5);self.child=None
            if self.reason=='running':self.reason='refinement_timeout'
            return False
        code=self.child.returncode;self.child=None
        if code!=0:self.reason='refinement_failed';return False
        ids=self.landmark_ids();gain=len(ids-self.known_track_ids)/max(1,len(self.known_track_ids));self.known_track_ids.update(ids);self.gains.append(gain)
        stat=json.loads((self.out/'stable/validation.json').read_text());self.checkpoints.append({'goals':len(self.goals),'new_track_fraction':gain,'accepted_landmarks':stat['accepted_landmarks'],'dataset_pixel_gate_pass':stat['dataset_pixel_gate_pass'],'test_pixels':stat['test_pixels']})
        self.last_checkpoint_goal=len(self.goals);self.phase='exploring';return True
    def cloud_capacity(self):
        try:return json.loads((self.out/'vision_status.json').read_text()).get('frames',0)>=5400
        except (ValueError,FileNotFoundError):return False
    def scan_xy(self):
        r=np.asarray(self.scan.ranges);a=self.scan.angle_min+np.arange(len(r))*self.scan.angle_increment;good=np.isfinite(r)&(r>self.scan.range_min)&(r<=self.scan.range_max)
        if good.sum()<40:return None,None
        return r[good]*np.cos(a[good])-.064,r[good]*np.sin(a[good])
    def recover(self):
        self.phase='limited_reverse';self.recoveries+=1;start=time.monotonic();distance=0.;last=None
        while time.monotonic()-start<10 and distance<.16 and not self.stopped:
            rclpy.spin_once(self,timeout_sec=.025)
            if self.healthy():return False
            x,y=self.scan_xy()
            if x is None or np.hypot(x,y).min()<.215:return False
            rear=-x[(x<-.1)&(abs(y)<.21)]
            if len(rear) and rear.min()<.34:return False
            m=self.odom;t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
            if last is not None:distance+=abs(m.twist.twist.linear.x)*max(0,min(t-last,.1))
            last=t;self.send(-.035,0)
        self.send();return distance>=.12
    def run(self):
        start=self.session_start;goal_start=start;best=1e9;last_progress=start;last_plan=0.;last_view=None;next_tick=0.
        try:
            while not self.stopped:
                rclpy.spin_once(self,timeout_sec=.015);now=time.monotonic();elapsed=now-start
                if now<next_tick:continue
                next_tick=now+.05
                if self.map is None or self.scan is None or self.odom is None or self.last_rgb is None or 'imu' not in self.times:
                    self.send()
                    if elapsed>25:self.reason='missing_sensors';break
                    continue
                failure=self.healthy()
                if failure:self.reason=failure;break
                try:tt=self.tf.lookup_transform('mono_map','mono_base',rclpy.time.Time())
                except Exception:
                    self.send()
                    if elapsed>25:self.reason='missing_tf';break
                    continue
                age=(self.get_clock().now()-rclpy.time.Time.from_msg(tt.header.stamp)).nanoseconds/1e9
                if age>1.:self.reason='stale_tf';break
                t=tt.transform;q=t.rotation;pos=np.array([t.translation.x,t.translation.y]);yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if self.prevpos is not None:
                    ds=np.linalg.norm(pos-self.prevpos)
                    if ds<.1:self.distance+=ds
                self.prevpos=pos.copy()
                if last_view is None or np.linalg.norm(pos-last_view[:2])>.1 or abs(wrap(yaw-last_view[2]))>.12:
                    self.views.append([*pos,yaw]);last_view=np.array([*pos,yaw])
                if now-self.last_status>3:self.status(elapsed);self.last_status=now
                if elapsed>=self.limit:self.reason='time_budget_reached_incomplete';break
                if not self.returning and (elapsed>self.limit-min(200,max(30,self.limit*.2)) or self.cloud_capacity()):
                    self.terminal='resource_budget_reached_incomplete';self.returning=True;self.path=None
                if not self.returning and len(self.goals)-self.last_checkpoint_goal>=6:
                    if not self.checkpoint():break
                    self.path=None;last_plan=0
                if self.path is None:
                    if now-last_plan<.7:self.send();continue
                    g=np.array(self.map.data).reshape(self.map.info.height,self.map.info.width);origin=np.array([self.map.info.origin.position.x,self.map.info.origin.position.y]);res=self.map.info.resolution
                    self.path,self.target_yaw,self.metrics=choose(g,res,origin,pos,self.views,self.failed,self.home,self.returning);last_plan=time.monotonic();goal_start=last_plan;best=1e9;last_progress=last_plan;self.sweep_hold=None
                    end_reason=completion(self.metrics,self.gains)
                    if not self.returning and end_reason:
                        self.terminal=end_reason;self.returning=True;self.path=None;continue
                    if self.path is None:
                        if self.returning and self.metrics.get('reason')=='home_reached':self.reason=self.terminal or 'home_reached';break
                        self.reason='no_safe_reachable_plan';break
                    self.phase='returning' if self.returning else self.metrics['reason']
                    pm=RosPath();pm.header.frame_id='mono_map';pm.header.stamp=self.get_clock().now().to_msg()
                    for xy in self.path:
                        pp=PoseStamped();pp.header=pm.header;pp.pose.position.x,pp.pose.position.y=map(float,xy);pp.pose.orientation.w=1.;pm.poses.append(pp)
                    self.path_pub.publish(pm)
                distance=np.linalg.norm(pos-self.path[-1]);heading_error=0 if self.target_yaw is None else wrap(self.target_yaw-yaw)
                reached=distance<.13 and (self.target_yaw is None or abs(heading_error)<.10)
                if reached:
                    self.send()
                    if self.sweep_hold is None:self.sweep_hold=now
                    if now-self.sweep_hold>.5:
                        self.goals.append({'xy':pos.tolist(),'yaw':yaw,'kind':self.phase});self.path=None
                    continue
                progress=distance+(abs(heading_error)*.12 if distance<.13 else 0)
                if progress<best-.02:best=progress;last_progress=now
                if now-last_progress>24 or now-goal_start>130:
                    self.failed.append([*self.path[-1],yaw if self.target_yaw is None else self.target_yaw]);self.path=None;self.send();continue
                if distance<.13:err=heading_error;v=0.
                else:
                    nearest=int(np.argmin(np.linalg.norm(self.path-pos,axis=1)));idx=nearest;arc=0.
                    while idx+1<len(self.path) and arc<.12:arc+=np.linalg.norm(self.path[idx+1]-self.path[idx]);idx+=1
                    d=self.path[idx]-pos;err=wrap(math.atan2(d[1],d[0])-yaw);v=.085*max(0.,math.cos(err)) if abs(err)<.42 else 0.
                w=float(np.clip(1.5*err,-.32,.32));x,y=self.scan_xy()
                if x is None:self.reason='insufficient_scan';break
                if np.hypot(x,y).min()<.235:
                    self.send()
                    if self.recoveries<3 and self.recover():
                        self.failed.append([*self.path[-1],yaw if self.target_yaw is None else self.target_yaw]);self.path=None;continue
                    self.reason='clearance_stop_incomplete';break
                front=x[(x>0)&(abs(y)<.23)]
                if v>0 and len(front) and front.min()<.36:
                    self.failed.append([*self.path[-1],yaw if self.target_yaw is None else self.target_yaw]);self.path=None;self.send();continue
                self.send(v,w)
            if self.stopped:self.reason='user_stop_incomplete'
        except Exception as exc:
            self.reason='exception_'+type(exc).__name__;self.get_logger().error(str(exc));raise
        finally:
            self.send();self.spin_stopped(1.)
            reason=self.reason
            if not self.stopped:
                try:self.checkpoint(final=True)
                except Exception as exc:self.get_logger().error('Final refinement: '+str(exc))
            self.reason=reason;self.phase='stopped';self.finished=True;self.watch.join(timeout=1)
            atomic(self.out/'view_history.json',self.views);self.status(time.monotonic()-start)
            for script in []:
                try:subprocess.run(['python3',str(ROOT/script)],env=os.environ,stdout=open(self.out/(script+'.log'),'w'),stderr=subprocess.STDOUT,timeout=30)
                except Exception:pass
            pid=self.owned.get('vision')
            if pid:
                try:os.killpg(pid,signal.SIGINT)
                except ProcessLookupError:pass
            print(json.dumps({'state':'stopped','reason':self.reason,'goals':len(self.goals),'zero_sent':True}),flush=True)

def main():
    lock=open(ROOT/'output/controller.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ap=argparse.ArgumentParser();ap.add_argument('--seconds',type=float,default=3600);args,rosargs=ap.parse_known_args();rclpy.init(args=rosargs);n=Autonomous(args.seconds)
    def stop(*unused):n.stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:n.run()
    finally:
        if n.child is not None:
            try:os.killpg(n.child.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        n.send();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
