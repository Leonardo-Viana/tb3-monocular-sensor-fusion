"""Metric multi-view triangulation from image rays and fused camera poses.

No depth, point-cloud or lidar-range input is accepted by these functions.
"""
import numpy as np

OPTICAL_TO_BASE=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
CAMERA_OFFSET=np.array([.069,-.047,.117])

def triangulate(uvs,poses,K,min_baseline=.12,min_parallax_deg=1.5):
    if len(uvs)<3:return None,'short_track'
    uv=np.asarray(uvs);Ts=np.asarray(poses)
    centers=Ts[:,:3,3]
    baseline=np.linalg.norm(centers[:,None,:]-centers[None,:,:],axis=2).max()
    if baseline<min_baseline:return None,'baseline'
    normalized=(np.column_stack([uv,np.ones(len(uv))])@np.linalg.inv(K).T)
    rays=np.einsum('nij,nj->ni',Ts[:,:3,:3],normalized)
    rays/=np.linalg.norm(rays,axis=1)[:,None]
    angles=np.arccos(np.clip(rays@rays.T,-1,1))
    parallax=float(angles.max())
    if parallax<np.deg2rad(min_parallax_deg):return None,'parallax'
    A=[]
    for ray,T in zip(normalized,Ts):
        P=np.linalg.inv(T)[:3]
        A.extend([ray[0]*P[2]-P[0],ray[1]*P[2]-P[1]])
    _,_,V=np.linalg.svd(np.array(A));h=V[-1]
    if abs(h[3])<1e-9:return None,'infinite'
    X=h[:3]/h[3]
    local=np.einsum('nij,nj->ni',Ts[:,:3,:3].transpose(0,2,1),X-centers)
    if np.any(local[:,2]<.15) or np.any(local[:,2]>6):return None,'cheirality_or_range'
    predicted=local@K.T;predicted=predicted[:,:2]/predicted[:,2,None]
    error=np.linalg.norm(predicted-uv,axis=1)
    if np.median(error)>1.1 or error.max()>2.5:return None,'reprojection'
    # Conservative pixel uncertainty and ray intersection conditioning.
    H=np.sum(np.eye(3)[None,:,:]-rays[:,:,None]*rays[:,None,:],axis=0)
    sigma_px=max(1.,float(np.sqrt(np.mean(error**2))))
    distance=np.linalg.norm(X-centers,axis=1).mean()
    covariance=np.linalg.pinv(H)*(sigma_px*distance/K[0,0])**2
    sigma=float(np.sqrt(np.linalg.eigvalsh(covariance).max()))
    if sigma>.22 or sigma/distance>.10:return None,'uncertainty'
    if not -.08<X[2]<3.0:return None,'height'
    return {'xyz':X,'baseline_m':float(baseline),'parallax_deg':float(np.rad2deg(parallax)),
            'reprojection_px':float(np.sqrt(np.mean(error**2))),'sigma_depth_m':sigma},'accepted'

def graph_warp(T,anchors,nodes):
    """Local SE(2) reanchoring to updated SLAM graph XY nodes.

    This is not full camera bundle adjustment; all rays are retriangulated
    after the correction, and inconsistent tracks are rejected again.
    """
    pairs=[(p,nodes[k]) for k,p in anchors.items() if k in nodes]
    if not pairs:return T
    a=np.array([p[0] for p in pairs]);b=np.array([p[1] for p in pairs])
    R=np.eye(2)
    if len(pairs)>=2 and np.linalg.norm(a-a.mean(0))>.05:
        U,_,V=np.linalg.svd((a-a.mean(0)).T@(b-b.mean(0)))
        R=V.T@U.T
        if np.linalg.det(R)<0:V[-1]*=-1;R=V.T@U.T
    shift=b.mean(0)-R@a.mean(0)
    W=np.eye(4);W[:2,:2]=R;W[:2,3]=shift
    return W@T
