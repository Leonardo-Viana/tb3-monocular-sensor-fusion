"""Read-only audit of one completed short integration test on the recording Dell."""
import argparse
import json
from pathlib import Path
import re
import time

import psutil
from analyze_campaign import analyze, read, save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    destination = root / 'analysis'
    destination.mkdir(exist_ok=False)
    run = root / 'repeat_01'
    out = run / 'output'
    execution = read(root / 'integration.json')
    if execution.get('status') == 'running':
        raise SystemExit('Wait for execution and cleanup to finish')
    result = analyze(run, destination)
    supervisor = read(out / 'supervisor_status.json')
    cleanup = read(out / 'controller_cleanup.json')
    marker = read(out / 'save_complete.json')
    frames = read(out / 'keyframes.json')
    tracks = read(out / 'tracks.json')
    resources = [json.loads(line) for line in (run / 'resources.jsonl').read_text().splitlines()]
    observed = {(p['pid'], p['create_time']) for row in resources for p in row['owned_processes']}
    remaining = []
    for pid, created in sorted(observed):
        try:
            p = psutil.Process(pid)
            if p.create_time() == created and p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                remaining.append({'pid': pid, 'name': p.name(), 'create_time': created})
        except psutil.NoSuchProcess:
            pass
    logs = {str(p.relative_to(root)): p.read_text(errors='replace')
            for p in [root / 'execution.log', run / 'mission.log', run / 'gazebo.log', *out.glob('*.log')]}
    exceptions = {name: [line for line in data.splitlines() if re.search(
        r'Traceback|KeyboardInterrupt|RCLError|RuntimeError|Fatal Python error|already called on the given context', line)]
        for name, data in logs.items()}
    exceptions = {name: lines for name, lines in exceptions.items() if lines}
    controls = result['command_audit']
    extra = {
        'campaign_completed': execution['run']['stage'] == 'completed',
        'supervisor_cleanup_without_escalation': supervisor.get('cleanup_escalated') is False,
        'supervisor_cleanup_without_errors': supervisor.get('cleanup_errors') == [],
        'controller_cleanup_without_escalation': all(not item['escalated'] and item['exit_code'] == 0 for item in cleanup),
        'saved_frame_count_matches_marker': marker['frames'] == len(frames['frames']),
        'saved_track_count_matches_marker': marker['tracks'] == len(tracks),
        'autonomous_motion_observed': controls['maximum_abs_linear_m_s'] > 0 and result['controller']['travelled_m'] > 0,
        'expected_short_budget_stop': result['controller']['reason'] in ['resource_budget_reached_incomplete', 'time_budget_reached_incomplete'],
        'required_topics_recorded': all(result['recording']['messages'].get(t, 0) > 0 for t in ['/camera/image_raw/compressed', '/camera/camera_info', '/scan', '/imu', '/odom', '/cmd_vel']),
        'no_runtime_exception_markers': not exceptions,
        'no_observed_owned_process_survives': not remaining,
        'one_rviz_at_most': result['resources']['maximum_rviz_instances'] == 1,
    }
    report = {'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
              'execution': execution, 'audit': result, 'integration_checks': extra,
              'integration_pass': all(result['checks'].values()) and all(extra.values()),
              'supervisor': supervisor, 'controller_cleanup': cleanup, 'save_marker': marker,
              'remaining_owned_processes': remaining, 'exception_markers': exceptions,
              'audit_sha256': sha(Path(__file__)),
              'scope': 'Short engineering integration validation. No complete-map, ground-truth accuracy, sensor-ablation or publication-readiness claim.'}
    save(destination / 'INTEGRATION_AUDIT.json', report)
    save(destination / 'process_logs.json', logs)
    print(json.dumps({'integration_pass': report['integration_pass'], 'extra_checks': extra,
                      'historical_audit_checks': result['checks'], 'resources': result['resources']}), flush=True)


if __name__ == '__main__':
    main()
