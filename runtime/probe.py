import time,json,rclpy
from sensor_msgs.msg import LaserScan,CompressedImage,Imu
from nav_msgs.msg import Odometry,OccupancyGrid
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from tf2_ros import Buffer,TransformListener
rclpy.init();n=rclpy.create_node('mono_preflight');counts={};last={};tf=Buffer();tl=TransformListener(tf,n)
def cb(k,m):counts[k]=counts.get(k,0)+1;last[k]=m
for typ,t in [(LaserScan,'/scan'),(CompressedImage,'/camera/image_raw/compressed'),(Odometry,'/odom'),(Imu,'/imu')]:n.create_subscription(typ,t,lambda m,k=t:cb(k,m),qos_profile_sensor_data)
n.create_subscription(OccupancyGrid,'/mono/map',lambda m:cb('/mono/map',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
start=time.monotonic()
while time.monotonic()-start<8:rclpy.spin_once(n,timeout_sec=.03)
result={'counts':counts,'nodes':n.get_node_names(),'publishers':{t:[p.node_name for p in n.get_publishers_info_by_topic(t)] for t in ['/cmd_vel','/mono/map']}}
try:
 t=tf.lookup_transform('mono_map','mono_base',rclpy.time.Time());result['pose']=[t.transform.translation.x,t.transform.translation.y];result['tf_stamp']=t.header.stamp.sec
except Exception as e:result['tf_error']=str(e)
if '/odom' in last:result['velocity']=[last['/odom'].twist.twist.linear.x,last['/odom'].twist.twist.angular.z]
print(json.dumps(result),flush=True);n.destroy_node();rclpy.shutdown()
