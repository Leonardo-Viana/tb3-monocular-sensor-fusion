"""Independent velocity-stop watchdog for the simulated robot."""
import sys,time,json,os
from pathlib import Path
import rclpy
from geometry_msgs.msg import TwistStamped
from lifecycle import StopRequest
from rclpy.signals import SignalHandlerOptions

def main(stop_request):
    owner=int(sys.argv[1]);lease=Path(sys.argv[2]);rclpy.init(args=sys.argv[3:],signal_handler_options=SignalHandlerOptions.NO);n=rclpy.create_node('mono_command_guard');p=n.create_publisher(TwistStamped,'/cmd_vel',10);dead_since=None;started=time.monotonic();last_zero=-1.
    try:
        while rclpy.ok() and not stop_request.requested:
            now=time.monotonic();alive=Path(f'/proc/{owner}').exists()
            try:alive=alive and Path(f'/proc/{owner}/stat').read_text().split(') ')[1].split()[0]!='Z'
            except FileNotFoundError:alive=False
            try:age=now-json.loads(lease.read_text())['monotonic']
            except (FileNotFoundError,ValueError):age=now-started
            if (not alive or age>.8) and now-last_zero>=.1:
                last_zero=now
                m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='mono_autonomy_guard';p.publish(m)
            if not alive:
                if dead_since is None:dead_since=now
                if now-dead_since>1.2:break
            rclpy.spin_once(n,timeout_sec=.08)
    finally:
        try:
            # Drain zero commands before releasing the publisher/context.
            for _ in range(3):
                if not rclpy.ok():break
                m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='mono_autonomy_guard';p.publish(m)
                rclpy.spin_once(n,timeout_sec=.05)
        finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':
 with StopRequest() as requested:main(requested)
