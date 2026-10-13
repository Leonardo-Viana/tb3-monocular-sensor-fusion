"""Replay original compressed RGB through the actual node's process method.

Uses the fixed recorded frame schedule, calibration, fused poses and graph.
No ROS node, simulator, sensor publisher or velocity publisher is started.
This development replay cannot measure live timing or independent accuracy.
"""
import argparse
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import types

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation


class Sink:
    def publish(self, message):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('destination', type=Path)
    ap.add_argument('--runtime', type=Path, required=True)
    args = ap.parse_args()
    args.destination.mkdir(parents=True, exist_ok=False)
    (args.destination/'frames').mkdir()
    sys.path.insert(0, str(args.runtime))
    from geometry import OPTICAL_TO_BASE
    spec = importlib.util.spec_from_file_location('replayed_monocular', args.runtime/'monocular.py')
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    data = json.loads((args.source/'keyframes.json').read_text())
    expected_tracks = json.loads((args.source/'tracks.json').read_text())
    cv2.setNumThreads(1)
    n = object.__new__(module.Monocular)
    n.latest = None; n.K = np.array(data['K']); n.last_stamp = -1.; n.last_T = None
    n.old_gray = None; n.tracking_stamp = None; n.tracking_T = None
    n.active = {}; n.tracks = {}; n.frames = []; n.next_id = 0
    n.nodes = {int(k): np.array(v) for k, v in data['graph'].items()}
    n.graph_revisions = 0; n.loop_edges = 0
    n.count = Counter(); n.reject = Counter(); n.landmarks = []; n.out = args.destination; n.preview = Sink()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.source/'raw'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    needed = {int(round(f['stamp']*1e9)): i for i, f in enumerate(data['frames'])}
    durations = []; reached = []
    began = time.monotonic()
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic != '/camera/image_raw/compressed':
            continue
        msg = deserialize_message(payload, CompressedImage)
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        if stamp not in needed:
            continue
        fid = needed[stamp]
        if fid != len(n.frames):
            raise RuntimeError(f'Missing or duplicate recorded frame: expected {len(n.frames)}, got {fid}')
        frame = data['frames'][fid]; T = np.array(frame['T']); tf = TransformStamped()
        q = Rotation.from_matrix(T[:3, :3]@OPTICAL_TO_BASE.T).as_quat()
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, T[:3, 3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
        n.tf = types.SimpleNamespace(lookup_transform=lambda *unused: tf)
        n.anchors = lambda unused: {int(k): np.array(v) for k, v in frame['anchors'].items()}
        n.latest = msg
        start = time.monotonic(); n.process(); durations.append(time.monotonic()-start)
        if len(n.frames) != fid+1:
            raise RuntimeError(f'Frame {fid} was unexpectedly skipped')
        reached.append(fid)
    if len(reached) != len(data['frames']):
        raise RuntimeError('Incomplete RGB replay')
    n.save()
    # Serialize exact preserved poses, so differences in coordinate conversion
    # cannot affect the subsequent paired reconstruction comparison.
    (args.destination/'keyframes.json').write_text(json.dumps(data))
    actual = json.loads((args.destination/'tracks.json').read_text())
    differences = sum(actual.get(k) != v for k, v in expected_tracks.items())
    report = {'development_replay': True, 'ros_nodes_started': False, 'velocity_published': False,
              'frames': len(n.frames), 'tracks': len(n.tracks), 'counts': dict(n.count),
              'duration_s': time.monotonic()-began, 'process_median_s': float(np.median(durations)),
              'process_p90_s': float(np.percentile(durations, 90)),
              'original_track_ids_missing_or_different': differences,
              'additional_track_ids': len(set(actual)-set(expected_tracks)),
              'exact_original_tracks_match': actual == expected_tracks,
              'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                  [args.source/'keyframes.json', args.source/'tracks.json', args.runtime/'monocular.py', args.runtime/'tracking_policy.py', *args.runtime.glob('ray_gate.py')]},
              'limitations': ['Fixed schedule and recorded fused poses; no live performance claim.',
                              'Previously inspected recording is development data, not independent evaluation.',
                              'Changed tracking changes IDs and their deterministic observation splits.']}
    (args.destination/'replay.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
