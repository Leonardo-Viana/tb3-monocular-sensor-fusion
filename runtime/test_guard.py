import time,json,subprocess,os,signal
from pathlib import Path
import rclpy
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock
root=Path(__file__).parent;lease=root/'output/test_lease.json';lease.write_text(json.dumps({'monotonic':time.monotonic()-3}))
rclpy.init();n=rclpy.create_node('guard_test');received=[];clock_pub=n.create_publisher(Clock,'/clock',10);ticks=0
n.create_subscription(TwistStamped,'/cmd_vel',lambda m:received.append((m.header.frame_id,m.twist.linear.x,m.twist.angular.z)),10)
p=subprocess.Popen(['python3',str(root/'command_guard.py'),str(os.getpid()),str(lease),'--ros-args','-p','use_sim_time:=true'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
try:
 end=time.monotonic()+4
 while time.monotonic()<end:
  for _ in range(20):
   ticks+=1;m=Clock();m.clock.sec=ticks//500;m.clock.nanosec=(ticks%500)*2000000;clock_pub.publish(m)
  rclpy.spin_once(n,timeout_sec=.005)
 zeros=[m for m in received if m[0]=='mono_autonomy_guard' and m[1]==0 and m[2]==0]
 assert 3<=len(zeros)<=42,(len(zeros),received)
 print('PASS: independent guard publishes repeated zero on expired heartbeat',len(zeros),flush=True)
finally:
 os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5);n.destroy_node();rclpy.shutdown();lease.unlink(missing_ok=True)
