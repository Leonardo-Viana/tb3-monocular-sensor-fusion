"""Read-only endpoint diagnosis on a preserved short integration recording.

Refits the unchanged baseline and observes its local variables with a profiler.
No observations are removed and no baseline map or metric is overwritten.
"""
import argparse
import collections
import hashlib
import importlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def project(point, pose, K):
    p = pose[:3, :3].T @ (point - pose[:3, 3])
    q = K @ p
    return (q[:2] / q[2]).tolist(), float(p[2])


def pair_probe(a, b, uv_a, uv_b, poses, K, images):
    pa, pb = poses[a], poses[b]
    relative = np.linalg.inv(pb) @ pa
    ray = np.linalg.inv(K) @ np.r_[uv_a, 1.]
    depths = np.geomspace(.15, 6., 300)
    q = (relative[:3, :3] @ (depths[:, None] * ray).T).T + relative[:3, 3]
    pix = q @ K.T
    valid = q[:, 2] > .15
    pix = pix[valid, :2] / pix[valid, 2:]
    rotation_ray = K @ relative[:3, :3] @ ray
    rotation_prediction = rotation_ray[:2] / rotation_ray[2]
    out = {'frames': [a, b], 'observed_uv': [uv_a, uv_b],
           'rotation_only_prediction': rotation_prediction.tolist(),
           'nearest_sampled_depth_prediction_px': float(np.linalg.norm(pix - uv_b, axis=1).min()) if len(pix) else None,
           'relative_translation_m': float(np.linalg.norm(relative[:3, 3]))}
    if a not in images or b not in images:
        out['raw_images_available'] = False
        return out
    gray = [cv2.GaussianBlur(cv2.cvtColor(images[f], cv2.COLOR_BGR2GRAY), (5, 5), .9) for f in [a, b]]
    old = np.array(uv_a, np.float32).reshape(1, 1, 2)
    params = dict(winSize=(25, 25), maxLevel=4, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, .01))
    for name, initial, flags in [('baseline', None, 0), ('rotation_initialized', rotation_prediction.astype(np.float32).reshape(1, 1, 2), cv2.OPTFLOW_USE_INITIAL_FLOW)]:
        new, status, error = cv2.calcOpticalFlowPyrLK(gray[0], gray[1], old, initial, flags=flags, **params)
        back, reverse_status, _ = cv2.calcOpticalFlowPyrLK(gray[1], gray[0], new, None, **params)
        out[name] = {'uv': new.ravel().tolist(), 'forward_status': int(status[0, 0]),
                     'backward_status': int(reverse_status[0, 0]), 'photometric_error': float(error[0, 0]) if status[0, 0] else None,
                     'forward_backward_px': float(np.linalg.norm(back-old)),
                     'difference_from_saved_uv_px': float(np.linalg.norm(new.ravel()-uv_b))}
    out['raw_images_available'] = True
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('destination', type=Path)
    ap.add_argument('--runtime', type=Path, required=True)
    args = ap.parse_args()
    args.destination.mkdir(exist_ok=False, parents=True)
    sys.path.insert(0, str(args.runtime))
    method = importlib.import_module('stable_cached')
    cv2.setNumThreads(1)
    source = args.source
    data = json.loads((source/'keyframes.json').read_text())
    tracks = json.loads((source/'tracks.json').read_text())
    original = json.loads((source/'stable/stable_landmarks.json').read_text())
    raw = np.array([method.flat(np.array(f['T'])) for f in data['frames']])
    full = np.array([f['T'] for f in data['frames']])
    K = np.array(data['K'])
    stamps = np.array([f['stamp'] for f in data['frames']])
    selected = [p for p in original if max(p['test_errors_px'], default=0) > 8]
    groups = collections.defaultdict(list)
    for tid, tr in tracks.items():
        if len(tr['obs']) >= 6:
            groups[tr['obs'][0][0]//method.BLOCK].append((tid, tr))
    captured = {}
    def observe(frame, event, arg):
        if frame.f_code is method.fit_block.__code__ and event == 'return' and 'poses' in frame.f_locals:
            loc = frame.f_locals
            captured[loc['block']] = {'poses': {f: p.copy() for f, p in loc['poses'].items()},
                'support': {f: int(loc['support'][i]) for i, f in enumerate(loc['fids'])},
                'chosen': loc['chosen']}
    refit = {}
    started = time.monotonic()
    for block in sorted({p['block'] for p in selected}):
        sys.setprofile(observe)
        try:
            good, report, errors = method.fit_block(block, groups[block], raw, stamps, K)
        finally:
            sys.setprofile(None)
        refit.update({p['track_id']: p for p in good})
        print(json.dumps({'refit_block': report}), flush=True)
    # Use original compressed RGB from the bag, not lossy saved JPEGs, for LK reproduction.
    needed = set()
    for p in selected:
        obs = tracks[str(p['track_id'])]['obs']
        for f, error in zip(p['test_frames'], p['test_errors_px']):
            if error > 8:
                idx = next(i for i, x in enumerate(obs) if x[0] == f)
                for j in range(max(0, idx-1), min(len(obs), idx+2)):
                    needed.add(obs[j][0])
    images = {}
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source/'raw'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    stamp_to_id = {int(round(stamps[f]*1e9)): f for f in needed}
    while reader.has_next() and len(images) < len(needed):
        topic, payload, _ = reader.read_next()
        if topic != '/camera/image_raw/compressed':
            continue
        msg = deserialize_message(payload, CompressedImage)
        stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        if stamp in stamp_to_id:
            im = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            images[stamp_to_id[stamp]] = cv2.resize(im, (960, 540))
    del reader
    findings = []
    for p in selected:
        obs = tracks[str(p['track_id'])]['obs']
        fit = refit[p['track_id']]
        block = captured[p['block']]
        poses = block['poses']
        record = {'track_id': p['track_id'], 'block': p['block'], 'observations': [],
                  'reproduced_test_frames': fit['test_frames'], 'reproduced_test_errors_px': fit['test_errors_px'],
                  'max_difference_from_original_test_errors_px': float(np.max(np.abs(np.array(fit['test_errors_px'])-p['test_errors_px']))),
                  'pair_probes': []}
        for i, (f, uv) in enumerate(obs):
            predicted, depth = project(np.array(fit['local_xyz']), poses.get(f, raw[f]), K)
            prior, _ = project(np.array(fit['local_xyz']), raw[f], K)
            category = 'train' if f in p['train_frames'] else 'validation' if f in p['validation_frames'] else 'test'
            record['observations'].append({'frame': f, 'category': category, 'uv': uv,
                'optimized_prediction_uv': predicted, 'prior_prediction_uv': prior,
                'error_px': float(np.linalg.norm(np.array(predicted)-uv)), 'depth_m': depth,
                'training_pose_support': block['support'].get(f, 0)})
            if f in p['test_frames'] and p['test_errors_px'][p['test_frames'].index(f)] > 8:
                a, b = (obs[i-1], obs[i]) if i else (obs[i], obs[i+1])
                record['pair_probes'].append(pair_probe(a[0], b[0], a[1], b[1], full, K, images))
                # Independent tracks at that same optimized pose diagnose a general pose failure.
                other = []
                for tid, track, train, val, test in block['chosen']:
                    if int(tid) == p['track_id'] or int(tid) not in refit:
                        continue
                    for tf, pixel in train+val+test:
                        if tf == f:
                            pred, _ = project(np.array(refit[int(tid)]['local_xyz']), poses[f], K)
                            other.append(float(np.linalg.norm(np.array(pred)-pixel)))
                record.setdefault('other_accepted_tracks_at_bad_frame', []).append({'frame': f, 'errors_px': method.stats(other)})
        findings.append(record)
    result = {'exploratory_development_diagnosis': True, 'baseline_modified': False,
        'duration_s': time.monotonic()-started, 'raw_frames_requested': sorted(needed),
        'raw_frames_decoded': sorted(images), 'findings': findings,
        'source_sha256': {str(p): sha(p) for p in [source/'keyframes.json', source/'tracks.json', source/'stable/stable_landmarks.json', args.runtime/'stable_cached.py', args.runtime/'geometry.py']}}
    (args.destination/'diagnosis.json').write_text(json.dumps(result, indent=2)+'\n')
    for fid, im in images.items():
        cv2.imwrite(str(args.destination/f'raw_{fid:05d}.png'), im)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
