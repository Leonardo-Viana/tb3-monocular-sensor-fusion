"""Sparse RGB reconstruction in rigid local submaps with reserved pixel tests.

Only recorded RGB tracks, calibration and fused pose priors are consumed.
Local bundle adjustment never consumes a later pose-graph revision. That
revision places each submap rigidly, preserving its internal ray geometry.
Test pixels are evaluated only AFTER point acceptance, and never reject points.
"""
import argparse, collections, hashlib, json, time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix
from geometry import OPTICAL_TO_BASE, triangulate, graph_warp

BLOCK = 64

def stats(v):
    v=np.asarray(v,dtype=float)
    return {'n':len(v),'median':float(np.median(v)), 'p90':float(np.percentile(v,90)), 'max':float(v.max())} if len(v) else {'n':0}

def partition(tid, obs):
    # Fixed per-track randomization, decided without looking at pixel residuals.
    order=np.random.default_rng(1729+int(tid)*7919).permutation(len(obs))
    n=max(1,len(obs)//5)
    test=set(order[:n]); validation=set(order[n:2*n])
    return tuple([v for i,v in enumerate(obs) if i in ix] for ix in
                 [set(range(len(obs)))-test-validation,validation,test])

def dlt(uv, Ts, K):
    rays=np.c_[uv,np.ones(len(uv))]@np.linalg.inv(K).T
    P=np.linalg.inv(Ts)[:,:3,:];A=np.stack([rays[:,0,None]*P[:,2]-P[:,0],rays[:,1,None]*P[:,2]-P[:,1]],axis=1).reshape(-1,4)
    h=np.linalg.svd(A,full_matrices=False)[2][-1]
    return h[:3]/h[3] if abs(h[3])>1e-9 else np.full(3,np.nan)

def errors(X, Ts, uv, K):
    loc=np.einsum('nij,nj->ni',Ts[:,:3,:3].transpose(0,2,1),X-Ts[:,:3,3]);q=loc@K.T
    e=np.linalg.norm(q[:,:2]/np.maximum(q[:,2,None],1e-6)-uv,axis=1)
    e[loc[:,2]<=.15]=1e6
    return e

def flat(T):
    T=T.copy();R=T[:3,:3]@OPTICAL_TO_BASE.T
    T[:3,:3]=Rotation.from_euler('z',np.arctan2(R[1,0],R[0,0])).as_matrix()@OPTICAL_TO_BASE
    T[2,3]=.117
    return T

def transform_for(data, anchor_id, graph=None):
    f=data['frames'][anchor_id];nodes={int(k):np.array(v) for k,v in (graph or data['graph']).items()}
    return graph_warp(np.eye(4),{int(k):np.array(v) for k,v in f['anchors'].items()},nodes)

def sensitivity(X, poses, uv, K, seed):
    # Engineering stress scenario, NOT calibrated posterior uncertainty.
    rng=np.random.default_rng(seed);d=[]
    for _ in range(32):
        P=poses.copy();P[:,:3,3]+=rng.normal(0,.002,(len(P),3))
        P[:,:3,:3]=Rotation.from_rotvec(rng.normal(0,.001,(len(P),3))).as_matrix()@P[:,:3,:3]
        v=dlt(uv+rng.normal(0,1.,uv.shape),P,K)
        d.append(np.linalg.norm(v-X))
    return float(np.percentile(d,90))

def fit_block(block, items, raw, stamps, K):
    chosen=[];points=[]
    for tid,t in items:
        train,val,test=partition(tid,t['obs'])
        if len(train)<4:continue
        ids=[f for f,p in train];uv=np.array([p for f,p in train]);P=raw[ids]
        if np.linalg.norm(P[:,None,:3,3]-P[None,:,:3,3],axis=2).max()<.15:continue
        X=dlt(uv,P,K)
        loc=np.einsum('nij,nj->ni',P[:,:3,:3].transpose(0,2,1),X-P[:,:3,3])
        if not np.isfinite(X).all() or np.any(loc[:,2]<.15) or np.any(loc[:,2]>6) or not -.3<X[2]<3:continue
        if np.median(errors(X,P,uv,K))>20:continue
        chosen.append((tid,t,train,val,test));points.append(X)
    if not chosen:return [],{'block':block,'input':len(items),'fit':0,'accepted':0},[]
    fids=sorted({f for _,_,tr,_,_ in chosen for f,p in tr});fm={f:i for i,f in enumerate(fids)}
    ci=[];pi=[];uv=[]
    for j,(_,_,tr,_,_) in enumerate(chosen):
        for f,p in tr:ci.append(fm[f]);pi.append(j);uv.append(p)
    ci=np.array(ci);pi=np.array(pi);uv=np.array(uv)
    nc=len(fids);npnt=len(points);no=len(uv);R0=raw[fids,:3,:3];C0=raw[fids,:3,3]
    edges=[(i,i+1) for i in range(nc-1) if 0<stamps[fids[i+1]]-stamps[fids[i]]<2.]
    edge=np.asarray(edges,int).reshape(-1,2)
    scale=np.sqrt(np.array([max(1,fids[b]-fids[a]) for a,b in edges]))[:,None]
    def decode(x):
        d=x[3*npnt:].reshape(nc,6)
        return x[:3*npnt].reshape(-1,3),Rotation.from_rotvec(d[:,:3]).as_matrix()@R0,C0+d[:,3:],d
    def residual(x):
        X,R,C,d=decode(x);loc=np.einsum('nij,nj->ni',R[ci].transpose(0,2,1),X[pi]-C[ci]);q=loc@K.T
        pixel=(q[:,:2]/np.maximum(q[:,2,None],.05)-uv)/1.2
        absolute=d/np.array([.025,.025,.025,.025,.025,.015])
        smooth=(d[edge[:,1]]-d[edge[:,0]])/(np.array([.0075,.0075,.0075,.01,.01,.0075])*scale)
        return np.r_[pixel.ravel(),absolute.ravel(),smooth.ravel()]
    sp=lil_matrix((2*no+6*nc+6*len(edges),3*npnt+6*nc),dtype=int)
    for i,(c,p) in enumerate(zip(ci,pi)):
        sp[2*i:2*i+2,3*p:3*p+3]=1;sp[2*i:2*i+2,3*npnt+6*c:3*npnt+6*c+6]=1
    for i in range(6*nc):sp[2*no+i,3*npnt+i]=1
    for i,(a,b) in enumerate(edges):
        for k in range(6):
            sp[2*no+6*nc+6*i+k,3*npnt+6*a+k]=1;sp[2*no+6*nc+6*i+k,3*npnt+6*b+k]=1
    bound=np.tile([.12,.12,.12,.15,.15,.08],nc)
    fit=least_squares(residual,np.r_[np.array(points).ravel(),np.zeros(6*nc)],jac_sparsity=sp.tocsr(),
      bounds=(np.r_[np.full(3*npnt,-np.inf),-bound],np.r_[np.full(3*npnt,np.inf),bound]),
      loss='soft_l1',f_scale=2.,x_scale='jac',max_nfev=600,ftol=1e-4)
    if not fit.success:
        return [],{'block':block,'input':len(items),'fit':npnt,'accepted':0,'converged':False,'nfev':fit.nfev,'rejections':{'optimizer_not_converged':npnt}},[]
    X,R,C,d=decode(fit.x);poses={f:raw[f].copy() for f in fids}
    for i,f in enumerate(fids):poses[f][:3,:3]=R[i];poses[f][:3,3]=C[i]
    support=collections.Counter(ci.tolist());good=[];test_errors=[];rejected=collections.Counter()
    for j,(tid,t,train,val,test) in enumerate(chosen):
        P=np.array([poses[f] for f,p in train]);xy=np.array([p for f,p in train]);e=errors(X[j],P,xy,K);ok=e<2.5
        if ok.sum()<4 or ok.mean()<.75:rejected['training_inliers']+=1;continue
        rr,why=triangulate(xy[ok],P[ok],K,min_baseline=.15,min_parallax_deg=2.)
        if rr is None:rejected[why]+=1;continue
        # Validation may select points. Test does not participate in acceptance.
        v=[(f,p) for f,p in val if f in fm and support[fm[f]]>=2]
        if not v:rejected['validation_pose_support']+=1;continue
        ve=errors(rr['xyz'],np.array([poses[f] for f,p in v]),np.array([p for f,p in v]),K)
        if np.median(ve)>3. or max(ve)>5.:rejected['validation_pixels']+=1;continue
        stress=sensitivity(rr['xyz'],P[ok],xy[ok],K,101+int(tid))
        distance=np.linalg.norm(rr['xyz']-P[ok,:3,3],axis=1).mean()
        if stress>min(.10,.1*distance):rejected['pose_sensitivity']+=1;continue
        rr={k:x.tolist() if isinstance(x,np.ndarray) else x for k,x in rr.items()}
        rr.update(track_id=int(tid),block=block,rgb=t['rgb'],local_xyz=rr['xyz'],anchor_frame=block*BLOCK,
          train_frames=[f for (f,p),o in zip(train,ok) if o],validation_frames=[f for f,p in v],
          validation_errors_px=ve.tolist(),stress_p90_m=stress)
        # Everything used to accept this landmark has now been computed.
        te=[(f,p) for f,p in test if f in fm and support[fm[f]]>=2]
        es=errors(np.array(rr['xyz']),np.array([poses[f] for f,p in te]),np.array([p for f,p in te]),K) if te else np.array([])
        rr.update(test_frames=[f for f,p in te],test_errors_px=es.tolist(),test_observations_unscored=len(test)-len(te))
        test_errors.extend(es.tolist());good.append(rr)
    return good,{'block':block,'input':len(items),'fit':npnt,'frames':nc,'accepted':len(good),'converged':bool(fit.success),'nfev':fit.nfev,'rejections':dict(rejected)},test_errors

def run(source, output):
    started=time.monotonic();output.mkdir(parents=True,exist_ok=True)
    data=json.loads((source/'keyframes.json').read_text());tracks=json.loads((source/'tracks.json').read_text());K=np.array(data['K'])
    raw=np.array([flat(np.array(f['T'])) for f in data['frames']]);stamps=np.array([f['stamp'] for f in data['frames']]);groups=collections.defaultdict(list)
    for tid,t in tracks.items():
        if len(t['obs'])>=6:groups[t['obs'][0][0]//BLOCK].append((tid,t))
    good=[];reports=[];test_errors=[];cache=output/'cache';cache.mkdir(exist_ok=True);reused=0
    method_hash=hashlib.sha256(Path(__file__).read_bytes()+Path(__file__).with_name('geometry.py').read_bytes()).hexdigest()
    for b,items in sorted(groups.items()):
        fids=sorted({int(f) for tid,t in items for f,uv in t['obs']})
        fingerprint=hashlib.sha256(json.dumps([method_hash,items,raw[fids].tolist(),stamps[fids].tolist(),K.tolist()],sort_keys=True).encode()).hexdigest()
        cp=cache/f'block_{b:04d}.json';old=json.loads(cp.read_text()) if cp.exists() else None
        if old and old['fingerprint']==fingerprint:
            pts,report,e=old['points'],old['report'],old['test_errors'];reused+=1
        else:
            pts,report,e=fit_block(b,items,raw,stamps,K)
            tmp=cp.with_suffix('.tmp');tmp.write_text(json.dumps({'fingerprint':fingerprint,'points':pts,'report':report,'test_errors':e}));tmp.replace(cp)
        print(json.dumps(report),flush=True)
        good.extend(pts);reports.append(report);test_errors.extend(e)
    for p in good:
        W=transform_for(data,p['anchor_frame']);p['xyz']=(W@np.r_[p['local_xyz'],1.])[:3].tolist()
    scored=[p for p in good if p['test_errors_px']]
    summary={'method':'Rigid RGB local submaps with metric fused pose priors and temporal regularization',
      'accepted_landmarks':len(good),'cached_blocks_reused':reused,'input_tracks':len(tracks),'frames':len(raw),'blocks':reports,
      'test_pixels':stats(test_errors),'test_scored_landmarks':len(scored),'test_unscored_landmarks':len(good)-len(scored),
      'test_landmark_medians_px':stats([np.median(p['test_errors_px']) for p in scored]),
      'test_pixels_over_3px':sum(e>3. for e in test_errors),'test_pixels_over_8px':sum(e>8. for e in test_errors),
      'stress_p90_distribution_m':stats([p['stress_p90_m'] for p in good]),
      'test_never_used_for_landmark_acceptance':True,'parameters_predeclared_before_test':True,
      'duration_s':time.monotonic()-started,'metric_ground_truth_used':False,
      'limitations':['Sparse landmarks only; no dense or complete 3D claim','Flat-floor motion prior',
        'Pixel tests reserve observations within existing tracks, not independent journeys',
        'No verified visual landmark association across separate revisits',
        'Global submap placement uses approximate XY pose-graph alignment',
        'Stress scenario is not calibrated posterior uncertainty',
        'RGB colors inherited from first tracked pixel']}
    summary['dataset_pixel_gate_pass']=len(test_errors)>=30 and np.median(test_errors)<=3. and np.percentile(test_errors,90)<=8. and all(r.get('converged',True) for r in reports if r['accepted'])
    summary['dataset_pixel_gate_pass']=bool(summary['dataset_pixel_gate_pass'])
    for name,value in [('stable_landmarks.json',good),('validation.json',summary),('anchors.json',{'graph':data['graph'],'frames':{str(p['anchor_frame']):data['frames'][p['anchor_frame']]['anchors'] for p in good}})]:
        dst=output/name;tmp=dst.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(dst)
    marker=output/'generation.json';tmp=marker.with_suffix('.tmp');tmp.write_text(json.dumps({'finished_wall':time.time(),'points':len(good),'frames':len(raw)}));tmp.replace(marker)
    with (output/'monocular_stable.ply').open('w') as f:
        f.write('ply\nformat ascii 1.0\nelement vertex '+str(len(good))+'\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for p in good:f.write(' '.join(map(str,[*p['xyz'],*p['rgb']]))+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='blocks'}),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('source',type=Path);parser.add_argument('output',type=Path);args=parser.parse_args();run(args.source,args.output)
