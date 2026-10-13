"""One short integration mission, using the existing Dell campaign monitor.

Prepare ROOT/repeat_01 with runtime files and ROOT/scene with a simulator scene.
This engineering test does not establish map completeness or sensor benefit.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform

import psutil
import rclpy
from campaign_runner import atomic, one, utc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--seed', type=int, default=23)
    parser.add_argument('--ros-domain', type=int, default=76)
    parser.add_argument('--partition', default='tb3_integration_20261012')
    args = parser.parse_args()
    root = args.root.resolve()
    lock = (root / 'integration.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if (root / 'integration.json').exists():
        raise SystemExit('Existing integration test preserved; use a fresh root')
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise SystemExit('Source ROS Jazzy first')
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.ros_domain), ROS_LOCALHOST_ONLY='1',
               GZ_PARTITION=args.partition, TB3_MISSION_SECONDS='180',
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               TURTLEBOT3_MODEL='waffle', QT_QPA_PLATFORM='xcb')
    env['GZ_SIM_RESOURCE_PATH'] = '/opt/ros/jazzy/share/turtlebot3_gazebo/models:' + env.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ.update(env)
    report = {'started_utc': utc(), 'status': 'running',
              'navigation_budget_s': 180, 'outer_mission_deadline_s': 1200,
              'ros_domain': args.ros_domain, 'gz_partition': env['GZ_PARTITION'], 'seed': args.seed,
              'host': platform.platform(),
              'total_memory_bytes': psutil.virtual_memory().total,
              'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'protocol_sha256': hashlib.sha256((root / 'PROTOCOL.md').read_bytes()).hexdigest()}
    atomic(root / 'integration.json', report)
    rclpy.init()
    try:
        report['run'] = one(root, 1, env, wall_deadline_s=1200, seed=args.seed)
        report['status'] = 'execution_finished_pending_audit'
    finally:
        report['finished_utc'] = utc()
        atomic(root / 'integration.json', report)
        rclpy.try_shutdown()
    print(json.dumps({'status': report['status'], 'stage': report.get('run', {}).get('stage')}), flush=True)


if __name__ == '__main__':
    main()
