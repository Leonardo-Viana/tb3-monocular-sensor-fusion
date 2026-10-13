"""Summarize fixed tracking regressions and the predeclared fresh-run tail gate.

Reads preserved datasets; no estimator, map or acceptance threshold is changed.
"""
import argparse
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import time
import unittest

import cv2
import numpy as np


def read(p):
    return json.loads(p.read_text())


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('development',type=Path)
    ap.add_argument('live',type=Path)
    ap.add_argument('campaign',type=Path)
    ap.add_argument('destination',type=Path)
    ap.add_argument('--source',type=Path,required=True,help='Preserved 12 October output directory')
    args=ap.parse_args();args.destination.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(args.development/'runtime'))
    from ray_gate import correspondence_mask,ray_segment_distances
    data_root=args.source
    frames=read(data_root/'keyframes.json');tracks=read(data_root/'tracks.json')
    K=np.array(frames['K']);fixtures=[]
    for tid,a,b in [(7759,113,114),(18191,209,210)]:
        obs=dict(tracks[str(tid)]['obs']);A=np.array(frames['frames'][a]['T']);B=np.array(frames['frames'][b]['T'])
        accepted=correspondence_mask([obs[a]],[obs[b]],A,B,K)
        fixtures.append({'track_id':tid,'frame_a':a,'frame_b':b,'K':K.tolist(),
                         'previous_T':A.tolist(),'current_T':B.tolist(),
                         'previous_uv':obs[a],'current_uv':obs[b],
                         'forward_segment_distance_px':float(ray_segment_distances([obs[a]],[obs[b]],A,B,K)[0]),
                         'reverse_segment_distance_px':float(ray_segment_distances([obs[b]],[obs[a]],B,A,K)[0]),
                         'rejected':not bool(accepted[0])})
    spec=importlib.util.spec_from_file_location('ray_tests',args.development/'tools/test_ray_gate.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    log=io.StringIO();tested=unittest.TextTestRunner(stream=log,verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(module))
    source=args.campaign/'analysis_20261008/repeat_01_revisit'
    unique=read(source/'revisit_matches.json');pairs=read(source/'revisit_pairs.json')
    selected=[r for r in unique if sorted((r['block_a'],r['block_b']))==[7,9] and r['epipolar_supported'] and r['repeat_support']]
    support=[]
    for p in pairs:
        if (p['frame_a'],p['frame_b']) in [(483,609),(474,605)]:
            status='estimated' if p['image_spread_pass'] and p['mutual_ratio_matches']>=12 else 'not_tested_insufficient_matches_or_spread'
            support.append({**p,'epipolar_test_status':status})
    live=read(args.live/'analysis/INTEGRATION_AUDIT.json');validation=live['audit']['validation']
    tail={'at_least_30_scored_observations':validation['test_pixels']['n']>=30,
          'all_accepted_landmarks_scored':validation['test_unscored_landmarks']==0,
          'no_residual_above_8px':validation['test_pixels_over_8px']==0}
    frozen=(sha(args.development/'runtime/ray_gate.py')==sha(args.live/'repeat_01/ray_gate.py')
            and sha(args.development/'runtime/monocular.py')==sha(args.live/'repeat_01/monocular.py'))
    report={'created_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            'scope':'Development regression plus one fresh short execution in the same scene; no absolute 3D accuracy or sensor-ablation claim.',
            'geometry_tests':{'run':tested.testsRun,'passed':tested.wasSuccessful(),'log':log.getvalue()},
            'historical_failure_fixtures':fixtures,
            'submap_support':{'selected_pairs':selected,'image_pairs':support,
                 'interpretation':'The second image pair had only ten descriptor matches; no fundamental matrix was fitted. Its false support flag is not a failed geometric test. The approximately 32 cm discrepancy remains unresolved.'},
            'candidate_sources_match_frozen_live_sources':frozen,
            'live_integration_pass':live['integration_pass'],
            'predeclared_visual_tail_checks':tail,'predeclared_visual_tail_pass':all(tail.values()),
            'live_validation':validation,'live_vision_status':read(args.live/'repeat_01/output/vision_status.json'),
            'live_launch_environment_note':read(args.live/'launch_environment_note.json'),
            'source_sha256':{str(p):sha(p) for p in [data_root/'keyframes.json',data_root/'tracks.json',
                 source/'revisit_matches.json',source/'revisit_pairs.json',args.live/'PROTOCOL.md',
                 args.live/'analysis/INTEGRATION_AUDIT.json',args.development/'runtime/ray_gate.py',
                 args.development/'runtime/monocular.py',args.development/'tools/test_ray_gate.py']},
            'opencv_version':cv2.__version__}
    (args.destination/'followup.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.destination/'fixtures.json').write_text(json.dumps(fixtures,indent=2)+'\n')
    print(json.dumps({'geometry_tests_passed':tested.wasSuccessful(),'tests':tested.testsRun,
          'historical_bad_transitions_rejected':all(x['rejected'] for x in fixtures),
          'live_integration_pass':live['integration_pass'],'visual_tail_checks':tail,
          'source_freeze_matches':frozen,'test_pixels':validation['test_pixels']}),flush=True)


if __name__=='__main__':main()
