"""Bounded, sequential Dell replication campaign; run from a sourced Jazzy shell.

Simulator world files are passed only to Gazebo. Monitoring subscribes only to clock.
The archived estimator/controller remain unchanged except for transport isolation.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time

import psutil
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data


def atomic(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def utc():
    return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())


def alive(process):
    try:
        return process.is_running() and process.status() != psutil.STATUS_ZOMBIE
    except psutil.NoSuchProcess:
        return False


class Monitor:
    def __init__(self, root):
        self.root = root
        self.owned = {}
        self.last_cpu = {}
        self.last_wall = time.monotonic()
        self.last_sim = None
        self.sim = None
        self.rows = []
        self.node = rclpy.create_node('publication_resource_observer')
        self.node.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        psutil.cpu_percent()

    def clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def sample(self, run, stage):
        now = time.monotonic()
        for p in psutil.Process().children(recursive=True):
            try:
                self.owned[(p.pid, p.create_time())] = p
            except psutil.NoSuchProcess:
                pass
        items = []
        for key, p in list(self.owned.items()):
            if not alive(p):
                continue
            try:
                cpu = sum(p.cpu_times()[:2])
                old = self.last_cpu.get(key)
                pct = None if old is None else 100 * (cpu - old[0]) / (now - old[1])
                self.last_cpu[key] = (cpu, now)
                mem = p.memory_info()
                try:
                    pss = p.memory_full_info().pss
                except (psutil.AccessDenied, AttributeError):
                    pss = None
                items.append({'pid': p.pid, 'create_time': key[1], 'name': p.name(),
                              'cpu_percent_one_core': pct, 'rss_bytes': mem.rss,
                              'pss_bytes': pss, 'cpu_seconds': cpu})
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        mem = psutil.virtual_memory()
        rtf = None
        if self.sim is not None and self.last_sim is not None and self.sim >= self.last_sim:
            rtf = (self.sim - self.last_sim) / (now - self.last_wall)
        row = {'utc': utc(), 'monotonic': now, 'run': run.name, 'stage': stage,
               'sample_interval_s': now - self.last_wall, 'sim_seconds': self.sim,
               'observed_rtf': rtf, 'system_cpu_percent': psutil.cpu_percent(),
               'mem_available_mib': mem.available / 2**20,
               'swap_used_mib': psutil.swap_memory().used / 2**20,
               'disk_free_gib': shutil.disk_usage(self.root).free / 2**30,
               'owned_processes': items,
               'sum_rss_mib_not_unique_memory': sum(x['rss_bytes'] for x in items) / 2**20,
               'sum_pss_mib': sum(x['pss_bytes'] or 0 for x in items) / 2**20,
               'pss_complete': all(x['pss_bytes'] is not None for x in items)}
        with (run / 'resources.jsonl').open('a') as stream:
            stream.write(json.dumps(row) + '\n')
        self.rows.append(row)
        self.last_sim, self.last_wall = self.sim, now
        return row

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def cleanup(self):
        # psutil Process objects retain creation identity, protecting against PID reuse.
        targets = [p for p in self.owned.values() if alive(p)]
        for p in targets:
            try:
                p.send_signal(signal.SIGINT)
            except psutil.NoSuchProcess:
                pass
        _, remaining = psutil.wait_procs(targets, timeout=20)
        for p in remaining:
            try:
                p.terminate()
            except psutil.NoSuchProcess:
                pass
        _, remaining = psutil.wait_procs(remaining, timeout=5)
        for p in remaining:
            try:
                p.kill()
            except psutil.NoSuchProcess:
                pass
        psutil.wait_procs(remaining, timeout=5)


def pause(env):
    result = subprocess.run(['gz', 'service', '-s', '/world/texture_trial/control',
        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000', '--req', 'pause: true'], env=env, capture_output=True,
        text=True, timeout=5)
    return result.returncode == 0 and 'true' in result.stdout


def launch(command, log, env):
    with log.open('a') as stream:
        return subprocess.Popen(command, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, start_new_session=True)


def one(root, number, env):
    run = root / ('repeat_%02d' % number)
    if number != 1:
        run.mkdir(exist_ok=False)
        for source in (root / 'repeat_01').iterdir():
            if source.is_file() and source.suffix in ('.py', '.yaml', '.rviz'):
                shutil.copy2(source, run / source.name)
    if (run / 'output').exists() or (run / 'campaign_run.json').exists():
        raise RuntimeError('Refusing to reuse existing run output: ' + str(run))
    state = {'run': run.name, 'seed': 22 + number, 'started_utc': utc(),
             'condition': 'textured', 'stage': 'preflight',
             'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in run.iterdir() if p.suffix in ('.py', '.yaml', '.rviz')}}
    atomic(run / 'campaign_run.json', state)
    monitor = Monitor(root)
    mission = None
    try:
        sample = monitor.sample(run, 'preflight')
        if sample['mem_available_mib'] < 1700 or sample['disk_free_gib'] < 9:
            raise RuntimeError('resource_preflight_failed')
        server = launch(['gz', 'sim', '-s', '-v', '2', '--seed', str(state['seed']),
                         str(root / 'scene/worlds/textured.sdf')], run / 'gazebo.log', env)
        state['gazebo_pid'] = server.pid
        deadline = time.monotonic() + 90
        ready = False
        while time.monotonic() < deadline:
            monitor.sample(run, 'gazebo_starting')
            if server.poll() is not None:
                raise RuntimeError('gazebo_exited_before_preflight')
            if pause(env):
                ready = True
                break
            monitor.tick()
        if not ready:
            raise RuntimeError('gazebo_control_timeout')
        mission = launch([sys.executable, str(run / 'launch_mission.py')],
                         run / 'mission.log', env)
        state.update(stage='mission_running', mission_pid=mission.pid)
        atomic(run / 'campaign_run.json', state)
        print(json.dumps(state), flush=True)
        began = time.monotonic()
        last_sample = 0
        while mission.poll() is None:
            monitor.tick()
            now = time.monotonic()
            if now - last_sample < 5:
                continue
            row = monitor.sample(run, 'mission_running')
            last_sample = now
            reason = None
            if (root / 'STOP').exists():
                reason = 'campaign_stop_file'
            elif row['mem_available_mib'] < 450:
                reason = 'hardware_low_available_memory'
            elif row['disk_free_gib'] < 6:
                reason = 'hardware_disk_reserve_reached'
            elif now - began > 4500:
                reason = 'campaign_wall_deadline'
            elif server.poll() is not None:
                reason = 'gazebo_exited_during_mission'
            if reason:
                (run / 'output/STOP').touch()
                state['stop_trigger'] = reason
                raise RuntimeError(reason)
        state['mission_exit_code'] = mission.returncode
        for name in ['supervisor_status.json', 'autonomy_status.json', 'stable/validation.json']:
            path = run / 'output' / name
            if path.exists():
                state[name] = json.loads(path.read_text())
        status = state.get('supervisor_status.json', {})
        state['stage'] = 'completed' if mission.returncode == 0 and status.get('stage') == 'stopped' else 'failed'
    except Exception as error:
        state.update(stage='failed', exception=repr(error))
    finally:
        # Stop simulator physics even when final fitting, process cleanup or recording fails.
        if mission is not None and mission.poll() is None:
            (run / 'output').mkdir(exist_ok=True)
            (run / 'output/STOP').touch()
        try:
            state['paused_before_cleanup'] = pause(env)
        except Exception as error:
            state['pause_error'] = repr(error)
        monitor.sample(run, 'cleanup')
        monitor.cleanup()
        if mission is not None:
            mission.poll()
        monitor.node.destroy_node()
        state['finished_utc'] = utc()
        rows = monitor.rows
        state['resources'] = {'samples': len(rows),
            'minimum_available_mib': min(r['mem_available_mib'] for r in rows),
            'maximum_swap_used_mib': max(r['swap_used_mib'] for r in rows),
            'minimum_disk_free_gib': min(r['disk_free_gib'] for r in rows),
            'maximum_sampled_sum_pss_mib': max(r['sum_pss_mib'] for r in rows),
            'note': 'Five-second samples, not continuous peaks; PSS includes only discovered descendants. RTF includes startup/paused intervals; filter by stage for interpretation.'}
        atomic(run / 'campaign_run.json', state)
        print(json.dumps({'run': run.name, 'stage': state['stage'], 'exception': state.get('exception'),
                          'finished_utc': state['finished_utc']}), flush=True)
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--runs', type=int, choices=[1, 2, 3], default=3)
    args = parser.parse_args()
    root = args.root.resolve()
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        raise SystemExit('Source ROS Jazzy before launching')
    lock = (root / 'campaign.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if (root / 'campaign.json').exists():
        raise SystemExit('Campaign already started; preserve it and prepare a new directory')
    env = dict(os.environ, ROS_DOMAIN_ID='75', ROS_LOCALHOST_ONLY='1',
               GZ_PARTITION='tb3_publication_20261007', OMP_NUM_THREADS='1',
               OPENBLAS_NUM_THREADS='1', TURTLEBOT3_MODEL='waffle')
    env['GZ_SIM_RESOURCE_PATH'] = '/opt/ros/jazzy/share/turtlebot3_gazebo/models:' + env.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ.update(env)
    report = {'started_utc': utc(), 'planned_runs': args.runs, 'status': 'running',
              'host': platform.platform(), 'logical_cpus': psutil.cpu_count(),
              'physical_cpus': psutil.cpu_count(logical=False),
              'total_memory_bytes': psutil.virtual_memory().total,
              'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'protocol_sha256': hashlib.sha256((root / 'PROTOCOL.md').read_bytes()).hexdigest(),
              'runs': []}
    atomic(root / 'campaign.json', report)
    rclpy.init()
    try:
        for number in range(1, args.runs + 1):
            if (root / 'STOP').exists():
                report['status'] = 'stopped_by_request'
                break
            result = one(root, number, env)
            report['runs'].append(result)
            atomic(root / 'campaign.json', report)
            if result['stage'] != 'completed':
                report['status'] = 'stopped_after_failure'
                break
        else:
            report['status'] = 'bounded_campaign_completed_pending_scientific_review'
    finally:
        report['finished_utc'] = utc()
        atomic(root / 'campaign.json', report)
        rclpy.shutdown()
    print(json.dumps({'campaign_status': report['status'], 'runs': len(report['runs'])}), flush=True)


if __name__ == '__main__':
    main()
