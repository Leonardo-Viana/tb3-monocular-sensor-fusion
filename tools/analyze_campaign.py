"""Read-only post-run audit of the 2026-10-07 replication campaign.

Run on the recording Dell in a sourced Jazzy shell. Creates new analysis files;
never rewrites captured evidence, imports world assets or publishes ROS messages.
"""
import argparse
import bisect
import collections
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sys
import time

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def read(path):
    return json.loads(path.read_text())


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2) + '\n')


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return dict(n=len(a), minimum=float(a.min()), median=float(np.median(a)),
                mean=float(a.mean()), p90=float(np.percentile(a, 90)),
                maximum=float(a.max())) if len(a) else {'n': 0}


def audit_bag(folder):
    metadata = yaml.safe_load((folder / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    expected = {t['topic_metadata']['name']: t['message_count'] for t in metadata['topics_with_message_count']}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    counts = collections.Counter()
    first = last = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        counts[topic] += 1
        if topic == '/camera/image_raw/compressed':
            if first is None:
                first = data
            last = data
    shapes = []
    for data in (first, last):
        if data is None:
            shapes.append(None)
            continue
        message = deserialize_message(data, CompressedImage)
        decoded = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        shapes.append(list(decoded.shape) if decoded is not None else None)
    return {'messages': dict(counts), 'expected_messages': expected,
            'matches_metadata': dict(counts) == expected,
            'endpoint_rgb_shapes': shapes,
            'duration_sim_s': metadata['duration']['nanoseconds'] * 1e-9,
            'bytes': sum(p.stat().st_size for p in folder.iterdir() if p.is_file()),
            'files_sha256': {p.name: sha(p) for p in folder.iterdir() if p.is_file()},
            'limits': 'Traversal and metadata agreement do not establish zero acquisition loss.'}


def analyze(run, destination):
    began = time.monotonic()
    out = run / 'output'
    manifest = read(run / 'campaign_run.json')
    status = read(out / 'autonomy_status.json')
    validation = read(out / 'stable/validation.json')
    points = read(out / 'stable/stable_landmarks.json')
    frames = read(out / 'keyframes.json')
    vision = read(out / 'vision_status.json')
    supervisor = read(out / 'supervisor_status.json')
    audit = [json.loads(line) for line in (out / 'mission_audit.jsonl').read_text().splitlines()]
    resource = [json.loads(line) for line in (run / 'resources.jsonl').read_text().splitlines()]
    timed_audit = [row for row in audit if 'monotonic' in row]
    at = [row['monotonic'] for row in timed_audit]
    commands = [command for row in audit for command in row.get('commands', [])]
    errors = [e for p in points for e in p['test_errors_px']]
    xyz = np.array([p['xyz'] for p in points])
    rgb = np.array([p['rgb'] for p in points])
    ply = (out / 'stable/monocular_stable.ply').read_text().splitlines()
    header_count = int(next(s for s in ply if s.startswith('element vertex ')).split()[-1])
    plydata = np.loadtxt(ply[ply.index('end_header') + 1:])
    scored = sum(bool(p['test_errors_px']) for p in points)
    checks = {
        'source_hashes_match_launch': all(sha(run / name) == value for name, value in manifest['source_sha256'].items()),
        'supervisor_stopped_exit_zero': supervisor['stage'] == 'stopped' and supervisor['controller_exit'] == 0,
        'simulation_pause_confirmed': manifest['paused_before_cleanup'],
        'final_observed_command_zero': audit[-1].get('finished', False) and audit[-1].get('last_command') == [0., 0.],
        'counts_agree': len(points) == validation['accepted_landmarks'] == status['accepted_landmarks'] == header_count == len(plydata),
        'ply_coordinates_match': bool(np.allclose(plydata[:, :3], xyz, rtol=0, atol=1e-12)),
        'ply_colors_match': bool(np.array_equal(plydata[:, 3:], rgb)),
        'finite_xyz': bool(np.isfinite(xyz).all()),
        'valid_colors': bool(np.all((rgb >= 0) & (rgb <= 255))),
        'captured_frames_equal_final_fit': len(frames['frames']) == validation['frames'],
        'test_count_matches': len(errors) == validation['test_pixels']['n'],
        'test_median_matches': bool(np.isclose(np.median(errors), validation['test_pixels']['median'], rtol=0, atol=1e-12)),
        'test_p90_matches': bool(np.isclose(np.percentile(errors, 90), validation['test_pixels']['p90'], rtol=0, atol=1e-12)),
        'test_max_matches': bool(np.isclose(max(errors), validation['test_pixels']['max'], rtol=0, atol=1e-12)),
        'scored_landmark_count_matches': scored == validation['test_scored_landmarks'],
        'unscored_landmark_count_matches': len(points)-scored == validation['test_unscored_landmarks'],
        'reserved_test_frames_disjoint': all(not(set(p['test_frames']) & (set(p['train_frames']) | set(p['validation_frames']))) for p in points),
    }
    log_issues = {}
    for p in sorted(out.glob('*.log')):
        lines = p.read_text(errors='replace').splitlines()
        selected = [s for s in lines if any(w in s for w in ['Traceback', 'RuntimeError', 'RCLError', '[ERROR]', 'messages lost', 'dropping message', 'Queue full', 'queue is full'])]
        if selected:
            log_issues[p.name] = {'matching_lines': len(selected), 'examples': list(dict.fromkeys(selected))[:12]}
    record_log = (out / 'record.log').read_text()
    loss = [int(x) for x in re.findall(r'Number of messages lost on the transport layer: (\d+)', record_log)]
    trace = []
    for row in resource:
        index = bisect.bisect_right(at, row['monotonic']) - 1
        phase = 'unmatched'
        if index >= 0 and row['monotonic']-at[index] <= 6:
            phase = timed_audit[index].get('status', {}).get('state', 'unknown')
        trace.append({k: row.get(k) for k in ['monotonic', 'sim_seconds', 'observed_rtf', 'sample_interval_s',
            'system_cpu_percent', 'mem_available_mib', 'swap_used_mib', 'disk_free_gib', 'sum_pss_mib',
            'sum_rss_mib_not_unique_memory', 'pss_complete', 'stage']} | {'controller_phase': phase,
            'cpu_percent_owned_one_core': sum(p['cpu_percent_one_core'] or 0 for p in row['owned_processes']),
            'rviz_count': sum(p['name'] == 'rviz2' for p in row['owned_processes'])})
    moving = [row for row in trace if row['controller_phase'] in ('frontier', 'visual_view', 'returning') and 0.5 <= row['sample_interval_s'] <= 8]
    rtf = [row['observed_rtf'] for row in moving if row['observed_rtf'] is not None]
    active = [row for row in trace if row['stage'] == 'mission_running']
    phases = collections.Counter(row['controller_phase'] for row in trace)
    viewer_pid = read(out / 'pids.json').get('viewer')
    viewer_rows = [row for row in resource if any(p['pid'] == viewer_pid for p in row['owned_processes'])]
    # World coordinates are not consumed. Final graph corrects stored estimated poses.
    sys.path.insert(0, str(run))
    from geometry import graph_warp
    nodes = {int(k): np.array(v) for k, v in frames['graph'].items()}
    path = [graph_warp(np.array(f['T']), {int(k): np.array(v) for k, v in f['anchors'].items()}, nodes)[:3, 3].tolist() for f in frames['frames']]
    bag = audit_bag(out / 'raw')
    checks['bag_counts_match_metadata'] = bag['matches_metadata']
    checks['rgb_endpoints_decode'] = all(s is not None for s in bag['endpoint_rgb_shapes'])
    unconverged = [b for b in validation['blocks'] if b.get('converged') is False]
    result = {
        'run': run.name, 'seed': manifest['seed'], 'checks': checks,
        'checks_pass': all(checks.values()), 'controller': status,
        'validation': validation, 'vision': vision, 'recording': bag,
        'recorder_transport_lost_messages_reported': loss[-1] if loss else None,
        'transport_loss_reporting_note': 'None means no explicit count found, not a measured zero.',
        'unconverged_blocks': unconverged,
        'source_sha256': manifest['source_sha256'],
        'estimated_home_distance_m': float(np.linalg.norm(status['current_position'])),
        'resources': manifest['resources'] | {
            'active_sample_count': len(active), 'navigation_sample_count': len(moving),
            'navigation_observed_rtf': stats(rtf),
            'navigation_system_cpu_percent': stats([row['system_cpu_percent'] for row in moving]),
            'navigation_owned_cpu_percent_one_core': stats([row['cpu_percent_owned_one_core'] for row in moving]),
            'phase_sample_counts': dict(phases),
            'first_swap_mib': trace[0]['swap_used_mib'], 'last_swap_mib': trace[-1]['swap_used_mib'],
            'disk_free_decrease_gib': trace[0]['disk_free_gib'] - trace[-1]['disk_free_gib'],
            'maximum_rviz_instances': max(row['rviz_count'] for row in trace),
            'viewer_last_seen_before_last_resource_sample_s': resource[-1]['monotonic']-viewer_rows[-1]['monotonic'] if viewer_rows else None,
            'note_phase': 'Nearest earlier 1 Hz controller status within six seconds labels approximately 5 s resource intervals; intervals can span transitions. RTF is not per-image latency.'},
        'geometry_diagnostics': {'xyz_min_m': xyz.min(axis=0).tolist(), 'xyz_max_m': xyz.max(axis=0).tolist(),
            'non_ground_ids_z_above_0_08_m': int(sum(xyz[:, 2] > .08)),
            'occupied_voxels_0_05_m': int(len(np.unique(np.floor(xyz/.05).astype(int), axis=0))),
            'note': 'Distribution diagnostics only; duplicate feature IDs are not merged and there is no surface ground truth.'},
        'command_audit': {'final': audit[-1], 'recorded_commands': len(commands),
            'publishers_frame_ids': sorted(set(x[1] for x in commands)),
            'maximum_abs_linear_m_s': max(abs(x[2]) for x in commands),
            'maximum_abs_angular_rad_s': max(abs(x[3]) for x in commands)},
        'log_issues': log_issues, 'analysis_seconds': time.monotonic()-began,
    }
    save(destination / (run.name + '_analysis.json'), result)
    save(destination / (run.name + '_plot_data.json'), {'xyz': xyz.tolist(), 'rgb': rgb.tolist(), 'path': path,
           'occupancy': read(out / 'map_snapshot.json'), 'resources': trace})
    print(json.dumps({'run': run.name, 'checks_pass': result['checks_pass'], 'bag_messages': sum(bag['messages'].values()),
                      'transport_loss': result['recorder_transport_lost_messages_reported'],
                      'unconverged_blocks': len(unconverged), 'seconds': result['analysis_seconds']}), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('campaign', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    args.destination.mkdir(exist_ok=True)
    if (args.destination / 'ANALYSIS.json').exists():
        raise SystemExit('Analysis exists; preserve it and choose a fresh directory.')
    began = time.monotonic()
    result = {'created_utc': datetime.now(timezone.utc).isoformat(),
        'analysis_script_sha256': sha(Path(__file__)), 'campaign': read(args.campaign / 'campaign.json'), 'runs': []}
    for run in sorted(args.campaign.glob('repeat_*')):
        result['runs'].append(analyze(run, args.destination))
    result['analysis_seconds'] = time.monotonic()-began
    save(args.destination / 'ANALYSIS.json', result)
    print('ANALYSIS_COMPLETE', flush=True)


if __name__ == '__main__':
    main()
