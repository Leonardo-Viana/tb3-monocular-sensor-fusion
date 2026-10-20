"""Inventory inputs for A/B/C/D without running ROS or publishing commands.

This is an input-readiness check, not a decoder, timestamp/calibration audit,
or a certificate that the four estimators have been implemented. No downloads.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import sys


def sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def inspect_source(root):
    root = Path(root)
    result = {'source': str(root), 'exists': root.is_dir(), 'blockers': []}
    if not root.is_dir():
        result['blockers'].append('source_directory_missing')
        return result
    bags = sorted(p for p in root.rglob('*') if p.is_file() and
                  p.suffix in {'.mcap', '.db3'})
    result['raw_payloads'] = [{'path': str(p.relative_to(root)),
                              'bytes': p.stat().st_size} for p in bags]
    result['bag_metadata_files'] = len(list(root.rglob('metadata.yaml')))
    if not any(p.stat().st_size for p in bags):
        result['blockers'].append('raw_sensor_payloads_missing')
    for name in ['keyframes.json', 'tracks.json']:
        p = root/name
        result[name] = {'present': p.is_file()}
        if p.is_file():
            result[name]['sha256'] = sha256(p)
        else:
            result['blockers'].append(name+'_missing')
    p = root/'keyframes.json'
    if p.is_file():
        try:
            d = json.loads(p.read_text())
            frames = d['frames']
            if not isinstance(frames, list) or not frames:
                raise ValueError('Expected nonempty frame schedule')
            missing = [i for i in range(len(frames))
                       if not (root/'frames'/f'{i:05d}.jpg').is_file()]
            result['schedule_frames'] = len(frames)
            result['decoded_jpg_missing'] = len(missing)
            result['first_missing_decoded_frame_ids'] = missing[:10]
            result['fixed_schedule_decoded_rgb_complete'] = not missing
        except (ValueError, KeyError, TypeError) as e:
            result['blockers'].append('invalid_keyframe_manifest: '+str(e))
    result['payload_presence_only_pass'] = not result['blockers']
    result['notes'] = ['Metadata alone does not contain sensor measurements.',
                       'Bag contents, complete frame coverage, sensor calibration '
                       'and time association still require a decoding audit.']
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sources', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite an existing audit; choose a new output')
    dependencies = {m: importlib.util.find_spec(m) is not None for m in
                    ['numpy', 'scipy', 'cv2', 'torch', 'rosbag2_py', 'rclpy']}
    report = {'created_utc': datetime.now(timezone.utc).isoformat(),
              'kind': 'input_readiness_only',
              'host': platform.node(), 'platform': platform.platform(),
              'python': sys.version, 'dependencies': dependencies,
              'sources': [inspect_source(p) for p in args.sources],
              'benchmark_executed': False,
              'no_robot_or_ros_process_started': True,
              'implementation_status': 'B/C/D integration not completed',
              'remaining_gates': ['Reconnect target Dell',
                  'Decode and hash all required sensor inputs',
                  'Verify timestamps, intrinsics, distortion and extrinsics',
                  'Implement and validate B/C/D adapters and common evaluator',
                  'Pin model weights/dependencies and freeze run manifest',
                  'Execute all arms on target hardware with identical inputs']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps({'output': str(args.output), 'benchmark_executed': False,
                      'sources': [{'source': x['source'], 'blockers': x['blockers']}
                                  for x in report['sources']]}, indent=2))
    # Absence is a recorded blocked result, never an implicit alternate dataset.
    return 0 if all(s.get('payload_presence_only_pass') for s in report['sources']) else 2


if __name__ == '__main__':
    raise SystemExit(main())
