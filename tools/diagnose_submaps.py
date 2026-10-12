"""Read-only exploratory diagnosis of fixed revisit associations.

No new map is produced. No scale fit, distance gating or residual-based deletion.
Planar fits use disjoint landmark-connected components for train/test separation.
This diagnoses selected correspondences, not absolute accuracy or sensor benefit.
"""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np


def stats(values):
    a=np.asarray(values,float)
    return {'n':len(a),'median':float(np.median(a)),'p90':float(np.percentile(a,90)),
            'max':float(a.max())} if len(a) else {'n':0}


def planar_fit(a,b):
    # Map b onto a, using XY only. Height and scale are never adjusted.
    ac=a[:,:2].mean(0);bc=b[:,:2].mean(0)
    u,sv,vt=np.linalg.svd((b[:,:2]-bc).T@(a[:,:2]-ac))
    r=vt.T@u.T
    if np.linalg.det(r)<0:
        vt[-1]*=-1;r=vt.T@u.T
    t=ac-r@bc
    return r,t,sv


def connected_folds(rows):
    parent={}
    def find(x):
        parent.setdefault(x,x)
        if parent[x]!=x:parent[x]=find(parent[x])
        return parent[x]
    for r in rows:
        a,b=find(r['track_a']),find(r['track_b'])
        parent[max(a,b)]=min(a,b)
    groups=defaultdict(list)
    for i,r in enumerate(rows):groups[find(r['track_a'])].append(i)
    # Fixed deterministic ordering; balance folds without inspecting residuals.
    folds=[[],[],[]]
    for k,ids in sorted(groups.items(),key=lambda x:(-len(x[1]),x[0])):
        j=min(range(3),key=lambda j:(len(folds[j]),j))
        folds[j].extend(ids)
    return folds,len(groups)


def group_report(rows,points):
    a=np.array([points[r['track_a']]['xyz'] for r in rows])
    b=np.array([points[r['track_b']]['xyz'] for r in rows])
    la=np.array([points[r['track_a']]['local_xyz'] for r in rows])
    lb=np.array([points[r['track_b']]['local_xyz'] for r in rows])
    delta=b-a
    folds,components=connected_folds(rows)
    report={'n':len(rows),'independent_landmark_components':components,
            'global_separation_m':stats(np.linalg.norm(delta,axis=1)),
            'horizontal_m':stats(np.linalg.norm(delta[:,:2],axis=1)),
            'vertical_m':stats(abs(delta[:,2])),
            'before_graph_warp_separation_m':stats(np.linalg.norm(lb-la,axis=1)),
            'median_delta_xyz_m':np.median(delta,axis=0).tolist(),
            'folds':[]}
    for j,test in enumerate(folds):
        train=sorted(set(range(len(rows)))-set(test))
        result={'fold':j,'train_n':len(train),'test_n':len(test)}
        if len(train)<6 or len(test)<3:
            result['status']='insufficient_support';report['folds'].append(result);continue
        train_ids={rows[i][k] for i in train for k in ('track_a','track_b')}
        test_ids={rows[i][k] for i in test for k in ('track_a','track_b')}
        assert not train_ids & test_ids
        # Require measurable planar spread; do not gate on fitting residuals.
        spread=np.linalg.svd(b[train,:2]-b[train,:2].mean(0),compute_uv=False)
        if spread[-1]/np.sqrt(len(train))<.01:
            result.update(status='degenerate_xy_support',spread_singular_values=spread.tolist())
            report['folds'].append(result);continue
        rotation,translation,cross_sv=planar_fit(a[train],b[train])
        fixed=b[test].copy();fixed[:,:2]=b[test,:2]@rotation.T+translation
        shifted=b[test].copy();shifted[:,:2]+=np.mean(a[train,:2]-b[train,:2],axis=0)
        result.update(status='scored',landmark_ids_disjoint=True,
                      rotation_deg=float(np.degrees(np.arctan2(rotation[1,0],rotation[0,0]))),
                      translation_xy_m=translation.tolist(),
                      original_m=stats(np.linalg.norm(b[test]-a[test],axis=1)),
                      translation_only_m=stats(np.linalg.norm(shifted-a[test],axis=1)),
                      planar_rigid_m=stats(np.linalg.norm(fixed-a[test],axis=1)),
                      test_track_pairs=[[rows[i]['track_a'],rows[i]['track_b']] for i in test])
        report['folds'].append(result)
    return report


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('campaign',type=Path);ap.add_argument('output',type=Path)
    args=ap.parse_args();args.output.mkdir(exist_ok=False)
    result={'created_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
            'exploratory':True,'map_modified':False,'ground_truth_used':False,
            'limitations':['Frozen associations may be wrong in repeated textures.',
                           'Cross-validation separates landmark IDs, not independent journeys or spatial neighborhoods.',
                           'Small and unequal block coverage; three runs do not establish general robustness.',
                           'Diagnostic registration is not an implemented SLAM correction.'],
            'runs':[]}
    for number in range(1,4):
        root=args.campaign/('repeat_%02d'%number)/'output'
        audit=args.campaign/'analysis_20261008'/('repeat_%02d_revisit'%number)
        paths=[root/'stable/stable_landmarks.json',root/'keyframes.json',
               audit/'revisit_matches.json',audit/'revisit_all_image_matches.json']
        points={p['track_id']:p for p in json.loads(paths[0].read_text())}
        frames=json.loads(paths[1].read_text())['frames']
        unique=json.loads(paths[2].read_text());all_rows=json.loads(paths[3].read_text())
        strict=[r for r in unique if r['epipolar_supported'] and r['repeat_support']]
        by_pair=defaultdict(list)
        for r in all_rows:
            if r['epipolar_supported']:by_pair[tuple(sorted((r['track_a'],r['track_b'])))].append(r)
        repeated_epi=[]
        for r in strict:
            support=by_pair[tuple(sorted((r['track_a'],r['track_b'])))]
            if any(abs(frames[x['frame_a']]['stamp']-frames[y['frame_a']]['stamp'])>=.32
                   and abs(frames[x['frame_b']]['stamp']-frames[y['frame_b']]['stamp'])>=.32
                   for x in support for y in support):
                repeated_epi.append(r)
        groups=defaultdict(list)
        for r in strict:groups[(r['block_a'],r['block_b'])].append(r)
        block_reports=[]
        for blocks,rows in sorted(groups.items()):
            block_reports.append({'blocks':list(blocks),**group_report(rows,points)})
        report={'seed':22+number,'strict_pairs':len(strict),
                'strict_global_m':stats([r['distance_m'] for r in strict]),
                'repeated_epipolar_pairs':len(repeated_epi),
                'repeated_epipolar_global_m':stats([r['distance_m'] for r in repeated_epi]),
                'block_pairs':block_reports,
                'source_sha256':{str(p.relative_to(args.campaign)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}
        result['runs'].append(report)
        print(json.dumps({'seed':report['seed'],'strict_n':len(strict),
                          'repeated_epipolar_n':len(repeated_epi),
                          'blocks':[{'blocks':r['blocks'],'n':r['n'],'before':r['before_graph_warp_separation_m'],
                                     'after':r['global_separation_m'],
                                     'cv':[{k:v for k,v in f.items() if k!='test_track_pairs'} for f in r['folds']]}
                                    for r in block_reports]}),flush=True)
    (args.output/'diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
