"""Wheel/gyro fusion. Explicitly NEVER reads Imu.orientation or magnetic data."""
import math
from collections import Counter
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from fusion_math import PlanarEKF

def seconds(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

class WheelIMU(Node):
    def __init__(self):
        super().__init__('mono_wheel_imu')
        self.ekf=PlanarEKF();self.imu=None;self.it=-1.;self.tilt=np.zeros(2)
        self.ready=False;self.last_gyro_t=None;self.count=Counter()
        self.create_subscription(Imu,'/imu',self.imu_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom_cb,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data)
        self.pub=self.create_publisher(Odometry,'/mono/wheel_imu',10)
        self.scan_pub=self.create_publisher(LaserScan,'/mono/scan',10)
        self.tf=TransformBroadcaster(self);self.st=StaticTransformBroadcaster(self)
        frames=[]
        for child,xyz in [('mono_laser',[-.064,0,.132]),('mono_camera_link',[.069,-.047,.117])]:
            t=TransformStamped();t.header.frame_id='mono_base';t.child_frame_id=child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
            t.transform.rotation.w=1.;frames.append(t)
        self.st.sendTransform(frames)

    def imu_cb(self,m):
        t=seconds(m)
        dt=0 if self.last_gyro_t is None else np.clip(t-self.last_gyro_t,0,.1)
        self.last_gyro_t=t
        self.tilt += np.array([m.angular_velocity.x,m.angular_velocity.y])*dt
        a=m.linear_acceleration;norm=math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)
        if abs(norm-9.81)<.25:
            tilt_acc=np.array([math.atan2(a.y,a.z),math.atan2(-a.x,math.hypot(a.y,a.z))])
            alpha=1-math.exp(-dt/.7)
            self.tilt=(1-alpha)*self.tilt+alpha*tilt_acc
        self.imu=m;self.it=t;self.count['imu']+=1

    def odom_cb(self,m):
        t=seconds(m)
        if not self.ready:
            # Establish our own arbitrary initial origin, with no compass heading.
            self.ready=True
        self.ekf.predict(t)
        v=m.twist.twist.linear.x;w=m.twist.twist.angular.z;gyro=None;still=False
        if self.imu is not None and abs(t-self.it)<.15:
            gyro=self.imu.angular_velocity.z
            a=self.imu.linear_acceleration
            still=abs(v)<.003 and abs(w)<.008 and abs(gyro)<.02 and abs(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z)-9.81)<.12
            self.count['imu_fused']+=1
        self.ekf.wheel(v,w,gyro,still)
        out=Odometry();out.header.stamp=m.header.stamp;out.header.frame_id='mono_odom';out.child_frame_id='mono_base'
        out.pose.pose.position.x=float(self.ekf.x[0]);out.pose.pose.position.y=float(self.ekf.x[1])
        q=Rotation.from_euler('xyz',[*self.tilt,self.ekf.x[2]]).as_quat()
        o=out.pose.pose.orientation;o.x,o.y,o.z,o.w=map(float,q)
        out.twist.twist.linear.x=float(self.ekf.x[3]);out.twist.twist.angular.z=float(self.ekf.x[4])
        for i,k in enumerate([0,1,5]):
            for j,l in enumerate([0,1,5]):out.pose.covariance[6*k+l]=float(self.ekf.P[i,j])
        self.pub.publish(out)
        tr=TransformStamped();tr.header=out.header;tr.child_frame_id='mono_base'
        tr.transform.translation.x=out.pose.pose.position.x;tr.transform.translation.y=out.pose.pose.position.y
        tr.transform.rotation=out.pose.pose.orientation;self.tf.sendTransform(tr)

    def scan_cb(self,m):
        if self.ready:
            m.header.frame_id='mono_laser';self.scan_pub.publish(m)

def main():
    rclpy.init();n=WheelIMU()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
