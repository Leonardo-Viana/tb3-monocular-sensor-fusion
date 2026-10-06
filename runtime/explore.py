"""Bounded simulation exploration using ONLY live lidar, ROS map and TF.

No Gazebo world queries. Independent command watchdog; stale sensors stop.
"""
import argparse,json,math,signal,threading,time,os
from pathlib import Path
import numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid,Odometry,Path as RosPath
from geometry_msgs.msg import TwistStamped,PoseStamped
from tf2_ros import Buffer,TransformListener
from planning import plan

def wrap(x):return (x+np.pi)%(2*np.pi)-np.pi

class Explorer(Node):
    def __init__(self,seconds):
        super().__init__('mono_explorer');self.limit=seconds
        self.out=Path(__file__).parent/'output';self.map=None;self.scan=None;self.odom=None
        self.times={};self.stopped=False;self.last_command=time.monotonic();self.finished=False
        q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/mono/map',lambda m:self.receive('map',m),q)
        self.create_subscription(LaserScan,'/scan',lambda m:self.receive('scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',lambda m:self.receive('odom',m),qos_profile_sensor_data)
        self.pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.path_pub=self.create_publisher(RosPath,'/mono/planned_path',q)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.visited=[];self.failed=[];self.goals=[];self.distance=0.;self.home=np.array([0.,0.])
        history=self.out/'visited.json'
        if history.exists():self.visited=[np.array(p) for p in json.loads(history.read_text())]
        self.path=None;self.kind='waiting';self.metrics={};self.reason='running';self.prevpos=None
        self.watch=threading.Thread(target=self.watchdog,daemon=True);self.watch.start()

    def receive(self,k,m):setattr(self,k,m);self.times[k]=time.monotonic()
    def send(self,v=0,w=0,heartbeat=True):
        m=TwistStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='base_footprint'
        m.twist.linear.x=float(v);m.twist.angular.z=float(w);self.pub.publish(m)
        if heartbeat:self.last_command=time.monotonic()
    def watchdog(self):
        while not self.finished:
            if time.monotonic()-self.last_command>.6:
                try:self.send(0,0,False)
                except Exception:pass
            time.sleep(.05)

    def status(self,elapsed):
        result={'reason':self.reason,'elapsed_wall_s':elapsed,'travelled_m':self.distance,'goal_kind':self.kind,
                'goals_reached':len(self.goals),'failed_goals':len(self.failed),'home':None if self.home is None else self.home.tolist(),
                'current_position':None if self.prevpos is None else self.prevpos.tolist(),'map_metrics':self.metrics,'goals':self.goals}
        (self.out/'exploration.json').write_text(json.dumps(result,indent=2))

    def run(self):
        start=time.monotonic();last_plan=-100.;last_report=-100.;goal_start=0.;bestdistance=1e9;last_progress=start
        try:
            while not self.stopped and time.monotonic()-start<self.limit:
                rclpy.spin_once(self,timeout_sec=.025);now=time.monotonic();elapsed=now-start
                if any(x not in self.times for x in ['map','scan','odom']):
                    self.send();
                    if elapsed>20:self.reason='missing_sensors';break
                    continue
                if any(now-self.times[x]>.8 for x in ['scan','odom']):self.reason='stale_sensors';break
                try:tr=self.tf.lookup_transform('mono_map','mono_base',rclpy.time.Time()).transform
                except Exception:self.send();continue
                pos=np.array([tr.translation.x,tr.translation.y]);q=tr.rotation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                if self.home is None:self.home=pos.copy()
                if self.prevpos is not None:
                    ds=np.linalg.norm(pos-self.prevpos)
                    if ds<.1:self.distance+=ds
                self.prevpos=pos.copy()
                if not self.visited or np.linalg.norm(pos-self.visited[-1])>.2:self.visited.append(pos.copy())
                if self.path is None:
                    if now-last_plan<1:self.send();continue
                    g=np.array(self.map.data).reshape(self.map.info.height,self.map.info.width)
                    origin=np.array([self.map.info.origin.position.x,self.map.info.origin.position.y]);res=self.map.info.resolution
                    self.path,self.kind,self.metrics=plan(g,res,origin,pos,self.visited,self.failed,self.home,
                                                        return_home=getattr(self,'return_only',False) or elapsed>self.limit-75)
                    last_plan=now;goal_start=now;bestdistance=1e9;last_progress=now
                    if self.path is None:
                        if self.kind=='home_reached':self.reason='home_reached_no_selected_frontier';break
                        self.send();continue
                    pm=RosPath();pm.header.frame_id='mono_map';pm.header.stamp=self.get_clock().now().to_msg()
                    for xy in self.path:
                        p=PoseStamped();p.header=pm.header;p.pose.position.x=float(xy[0]);p.pose.position.y=float(xy[1]);p.pose.orientation.w=1.;pm.poses.append(p)
                    self.path_pub.publish(pm)
                target_distance=np.linalg.norm(pos-self.path[-1])
                if target_distance<.14:
                    self.goals.append({'xy':pos.tolist(),'kind':self.kind});self.path=None;self.send();continue
                if target_distance<bestdistance-.03:bestdistance=target_distance;last_progress=now
                if now-last_progress>18 or now-goal_start>90:
                    self.failed.append(self.path[-1].copy());self.path=None;self.send();continue
                # Look ahead along the collision-inflated path, not across corners.
                distances=np.linalg.norm(self.path-pos,axis=1);nearest=int(np.argmin(distances))
                idx=nearest;arc=0.
                while idx+1<len(self.path) and arc<.13:
                    arc+=np.linalg.norm(self.path[idx+1]-self.path[idx]);idx+=1
                delta=self.path[idx]-pos;err=wrap(math.atan2(delta[1],delta[0])-yaw)
                w=np.clip(1.6*err,-.35,.35);v=.09*max(0.,math.cos(err)) if abs(err)<.45 else 0.
                ranges=np.asarray(self.scan.ranges);a=self.scan.angle_min+np.arange(len(ranges))*self.scan.angle_increment
                valid=np.isfinite(ranges)&(ranges>self.scan.range_min)
                if valid.sum()<40:self.reason='insufficient_scan';break
                x=ranges[valid]*np.cos(a[valid])-.064;y=ranges[valid]*np.sin(a[valid])
                clearance=np.hypot(x,y).min()
                front=x[(x>0)&(abs(y)<.23)]
                if clearance<.235:
                    self.send();self.reason='body_clearance';break
                if v>0 and len(front) and front.min()<.34:
                    self.failed.append(self.path[-1].copy());self.path=None;self.send();continue
                self.send(v,w)
                if now-last_report>5:self.status(elapsed);last_report=now
            else:self.reason='interrupted' if self.stopped else 'deadline'
        finally:
            end=time.monotonic()+1.2
            while time.monotonic()<end:self.send();rclpy.spin_once(self,timeout_sec=.025)
            self.finished=True;self.watch.join(timeout=1);self.status(time.monotonic()-start)
            (self.out/'visited.json').write_text(json.dumps([v.tolist() for v in self.visited]))
            print(json.dumps({'reason':self.reason,'distance_m':self.distance,'goals':len(self.goals),'zero_sent':True}),flush=True)

def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=600);p.add_argument('--return-only',action='store_true');args,rosargs=p.parse_known_args()
    rclpy.init(args=rosargs);n=Explorer(args.seconds);n.return_only=args.return_only
    pf=n.out/'pids.json';pids=json.loads(pf.read_text()) if pf.exists() else {};pids['explore']=os.getpid();pf.write_text(json.dumps(pids))
    def stop(*args):n.stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:n.run()
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
