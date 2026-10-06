"""Supervisor for the existing, paused textured Gazebo simulation (domain 73).
No scene geometry is used by exploration, pose fusion or reconstruction.
"""
import os,sys,json,time,signal,subprocess,hashlib,fcntl,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output'
ENV=dict(os.environ,ROS_DOMAIN_ID='74',ROS_LOCALHOST_ONLY='1',GZ_PARTITION='tb3_texture_pilot_20260929',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
TOPICS=['/camera/image_raw/compressed','/camera/camera_info','/scan','/imu','/odom','/joint_states','/tf','/tf_static','/clock','/cmd_vel']
def stop(p,wait=12):
 if p is None or p.poll() is not None:return
 os.killpg(p.pid,signal.SIGINT)
 try:p.wait(timeout=wait)
 except subprocess.TimeoutExpired:
  os.killpg(p.pid,signal.SIGTERM)
  try:p.wait(timeout=4)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
def pause(value):
 r=subprocess.run(['gz','service','-s','/world/texture_trial/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: '+str(value).lower()],env=ENV,capture_output=True,text=True,timeout=8)
 if r.returncode or 'true' not in r.stdout:raise RuntimeError('Simulation control failed: '+r.stdout+r.stderr)
def main():
 if os.environ.get('ROS_DISTRO')!='jazzy':raise SystemExit('Source ROS Jazzy first')
 OUT.mkdir(exist_ok=True);lock=open(OUT/'supervisor.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 if (OUT/'run_started.json').exists():raise SystemExit('Existing run preserved; choose a fresh project/run for a new mission')
 manifest={'started_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),'mission_wall_budget_s':2400,'ros_domain':74,'gz_partition':ENV['GZ_PARTITION'],'sensor_policy':'RGB rays only for 3D; raw gyro/accel, wheel velocity and planar lidar for pose; no depth/compass/simulator pose','source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.iterdir() if p.suffix in ['.py','.yaml','.rviz']},'stage':'starting'}
 (OUT/'run_started.json').write_text(json.dumps(manifest,indent=2));procs={};pids={};normal=False
 def launch(name,cmd):
  p=subprocess.Popen(cmd,env=ENV,stdout=open(OUT/(name+'.log'),'a'),stderr=subprocess.STDOUT,start_new_session=True);procs[name]=p;pids[name]=p.pid;(OUT/'pids.json').write_text(json.dumps(pids));return p
 args=['--ros-args','-p','use_sim_time:=true']
 try:
  pause(True)
  launch('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p','config_file:=/opt/ros/jazzy/share/turtlebot3_gazebo/params/turtlebot3_waffle_bridge.yaml'])
  launch('image',['ros2','run','ros_gz_image','image_bridge','/camera/image_raw'])
  launch('odometry',['python3',str(ROOT/'odometry.py'),*args])
  launch('slam',['ros2','run','slam_toolbox','async_slam_toolbox_node','--ros-args','-r','__node:=mono_slam','--params-file',str(ROOT/'slam.yaml'),'-r','map:=/mono/map','-r','map_metadata:=/mono/map_metadata','-r','pose:=/mono/laser_pose','-r','/slam_toolbox/graph_visualization:=/mono/graph','-r','/mono_slam/graph_visualization:=/mono/graph'])
  for transition in ['configure','activate']:
   for attempt in range(10):
    r=subprocess.run(['ros2','lifecycle','set','/mono_slam',transition],env=ENV,capture_output=True,text=True,timeout=10)
    if 'successful' in r.stdout:break
    time.sleep(1)
   else:raise RuntimeError('SLAM lifecycle failed: '+r.stdout+r.stderr)
  launch('vision',['python3',str(ROOT/'monocular.py'),*args])
  launch('record',['ros2','bag','record','-s','mcap','-o',str(OUT/'raw'),'--use-sim-time','--max-cache-size','16777216','--max-bag-size','1000000000','--qos-profile-overrides-path',str(ROOT/'record_qos.yaml'),'--topics',*TOPICS])
  launch('viewer',['python3',str(ROOT/'live_view.py')])
  launch('rviz',['rviz2','-d',str(ROOT/'mission.rviz')])
  launch('audit',['python3',str(ROOT/'audit_mission.py'),str(os.getpid())])
  pause(False)
  deadline=time.monotonic()+45
  while time.monotonic()<deadline:
   p=OUT/'keyframes.json'
   try:ready=p.exists() and len(json.loads(p.read_text())['frames'])>0
   except ValueError:ready=False
   if ready:break
   if procs['vision'].poll() is not None:raise RuntimeError('Monocular process exited during preflight')
   time.sleep(1)
  else:raise RuntimeError('No RGB keyframe with fused camera pose')
  r=subprocess.run(['python3',str(ROOT/'probe.py')],env=ENV,capture_output=True,text=True,timeout=20);(OUT/'preflight.json').write_text(r.stdout)
  d=json.loads(r.stdout.strip().splitlines()[-1]);needed=['/scan','/odom','/imu','/camera/image_raw/compressed','/mono/map']
  if any(d.get('counts',{}).get(t,0)<1 for t in needed) or 'pose' not in d:raise RuntimeError('Required sensors/pose missing')
  if d.get('publishers',{}).get('/cmd_vel'):raise RuntimeError('Another velocity publisher exists')
  p=launch('autonomy',['python3',str(ROOT/'autonomy_async.py'),'--seconds','2400',*args]);manifest['stage']='exploring';(OUT/'supervisor_status.json').write_text(json.dumps(manifest,indent=2));print('AUTONOMY_STARTED',p.pid,flush=True)
  deadline=time.monotonic()+4080
  while p.poll() is None and time.monotonic()<deadline:
   if procs['record'].poll() is not None:raise RuntimeError('Recorder stopped unexpectedly')
   time.sleep(1)
  if p.poll() is None:raise RuntimeError('Supervisor wall deadline')
  stop(procs['record'],30);stop(procs['vision'],20);time.sleep(2);pause(True)
  manifest['stage']='stopped';manifest['controller_exit']=p.returncode;manifest['result']=json.loads((OUT/'autonomy_status.json').read_text());normal=True
 except BaseException as e:
  manifest['stage']='stopped_error';manifest['exception']=repr(e);raise
 finally:
  stop(procs.get('autonomy'),30)
  stop(procs.get('vision'),20);stop(procs.get('record'),30)
  try:pause(True)
  except Exception as e:manifest['pause_error']=repr(e)
  # Keep one viewer and its frozen/live SLAM frame tree available for inspection.
  if not normal:
   for n in ['image','bridge','odometry','slam']:stop(procs.get(n))
  manifest['finished_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime());(OUT/'supervisor_status.json').write_text(json.dumps(manifest,indent=2));print(json.dumps({'stage':manifest['stage'],'reason':manifest.get('result',{}).get('reason'),'exception':manifest.get('exception')}),flush=True)
if __name__=='__main__':main()
