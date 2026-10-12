"""Exercise the actual zero-command guard and audit in isolated ROS domain 176."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('root',type=Path)
    root=ap.parse_args().root.resolve();sys.path.insert(0,str(root/'runtime'))
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import TwistStamped
    from lifecycle import ProcessStopper
    os.environ['ROS_DOMAIN_ID']='176';os.environ['ROS_LOCALHOST_ONLY']='1'
    work=root/'results'/'stop_audit';work.mkdir(exist_ok=False);(work/'output').mkdir()
    for name in ('audit_mission.py','command_guard.py','lifecycle.py'):
        shutil.copy2(root/'runtime'/name,work/name)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n=rclpy.create_node('shutdown_regression_observer');messages=[]
    n.create_subscription(TwistStamped,'/cmd_vel',
                          lambda m:messages.append([m.twist.linear.x,m.twist.angular.z]),100)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
    logs=[];procs=[];stopper=ProcessStopper();report={}
    try:
        for name,args in [('audit_mission.py',[str(os.getpid())]),
                          ('command_guard.py',[str(os.getpid()),str(work/'absent_lease.json')])]:
            log=(work/(name+'.log')).open('w');logs.append(log)
            procs.append(subprocess.Popen([sys.executable,str(work/name),*args],cwd=work,
                                           stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        # With no lease, this guard can only publish zero. No simulator exists.
        spin(3)
        before=len(messages)
        guard=stopper.stop(procs[1],wait=5);spin(.4)
        audit=stopper.stop(procs[0],wait=5)
        lines=[json.loads(x) for x in (work/'output/mission_audit.jsonl').read_text().splitlines()]
        report={'guard':guard,'audit':audit,'zero_messages_before_stop':before,
                'received_messages':len(messages),'only_zero_commands':bool(messages) and
                    all(v==[0.,0.] for v in messages),
                'audit_finished':bool(lines[-1].get('finished')),
                'audit_last_command':lines[-1].get('last_command'),
                'scope':'ROS domain 176; no simulator; no nonzero command producer',
                'all_pass':guard['exit_code']==0 and audit['exit_code']==0 and
                    not guard['escalated'] and not audit['escalated'] and before>=3 and
                    all(v==[0.,0.] for v in messages) and bool(lines[-1].get('finished')) and
                    lines[-1].get('last_command')==[0.,0.]}
    finally:
        for p in procs:
            if p.poll() is None:stopper.stop(p,wait=3)
        for log in logs:log.close()
        n.destroy_node();rclpy.try_shutdown()
    (work/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    if not report['all_pass']:raise SystemExit(1)


if __name__=='__main__':main()
