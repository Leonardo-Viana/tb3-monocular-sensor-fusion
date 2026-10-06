"""Read-only command/status/resource audit, independent of control and fitting."""
import sys,time,json,os
from pathlib import Path
import psutil,rclpy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
root=Path(__file__).resolve().parent;out=root/'output';owner=int(sys.argv[1]);rclpy.init();n=rclpy.create_node('mono_mission_audit');commands=[];last=None;count=0;status={};began=time.monotonic();previous=began
f=(out/'mission_audit.jsonl').open('a',buffering=1)
def command(m):
 global last,count
 last=[m.twist.linear.x,m.twist.angular.z];count+=1;commands.append([time.monotonic(),m.header.frame_id,*last])
def state(m):
 global status
 try:status=json.loads(m.data)
 except ValueError:pass
n.create_subscription(TwistStamped,'/cmd_vel',command,100);n.create_subscription(String,'/mono/autonomy_status',state,10)
while Path(f'/proc/{owner}').exists() and psutil.Process(owner).status()!='zombie':
 rclpy.spin_once(n,timeout_sec=.025);now=time.monotonic()
 if now-previous<1:continue
 mem=psutil.virtual_memory();f.write(json.dumps({'monotonic':now,'wall_time':time.time(),'dt':now-previous,'commands':commands,'command_total':count,'last_command':last,'available_mib':mem.available/2**20,'swap_used_mib':psutil.swap_memory().used/2**20,'disk_free_gib':psutil.disk_usage(str(out)).free/2**30,'status':status})+'\n');commands=[];previous=now
f.write(json.dumps({'finished':True,'commands_total':count,'last_command':last,'elapsed_wall_s':time.monotonic()-began})+'\n');f.close();n.destroy_node();rclpy.shutdown()
