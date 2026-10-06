"""Finite camera-view coverage and reachable-frontier planning from ROS map only."""
import numpy as np
from scipy.ndimage import distance_transform_edt,binary_dilation,label
from scipy.spatial import cKDTree
from planning import dijkstra

def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi

def choose(grid,res,origin,pos,views,failed,home,return_home=False):
    occupied=grid>=50;free=(grid>=0)&(grid<30);clear=distance_transform_edt(~occupied)*res
    walk=free&(clear>.34);cells=np.argwhere(walk)
    if not len(cells):return None,None,{'reason':'no_safe_cells'}
    cell=np.floor((pos-origin)/res).astype(int)[::-1];ds=np.linalg.norm((cells-cell)*res,axis=1);start=tuple(cells[ds.argmin()])
    if ds.min()>.3:return None,None,{'reason':'start_clearance'}
    costs,prev=dijkstra(walk,start,clear);reachable=np.isfinite(costs)
    def world(c):return origin+(np.array(c)[::-1]+.5)*res
    candidates=[];front=walk&binary_dilation(grid<0,iterations=3)&reachable
    labs,n=label(front);front_groups=0
    for k in range(1,n+1):
        cs=np.argwhere(labs==k)
        if len(cs)<4:continue
        front_groups+=1;cs=sorted(cs,key=lambda c:costs[tuple(c)]);c=tuple(cs[len(cs)//3]);p=world(c)
        if np.linalg.norm(p-pos)>.18 and not any(np.linalg.norm(p-f[:2])<.25 for f in failed):
            candidates.append((5./(.8+costs[c]*res),c,None,'frontier'))
    # Stable world lattice (independent of map bounding-box changes).
    minxy=origin;maxxy=origin+np.array(grid.shape[::-1])*res;view_goals=[]
    for y in np.arange(np.ceil(minxy[1]/.60)*.60,maxxy[1],.60):
        for x in np.arange(np.ceil(minxy[0]/.60)*.60,maxxy[0],.60):
            c=tuple(np.floor((np.array([x,y])-origin)/res).astype(int)[::-1])
            if not(0<=c[0]<grid.shape[0] and 0<=c[1]<grid.shape[1] and reachable[c] and clear[c]>.37):continue
            for yaw in np.arange(8)*np.pi/4:view_goals.append((c,np.array([x,y]),yaw))
    past=np.asarray(views).reshape(-1,3);tree=cKDTree(past[:,:2]) if len(past) else None
    covered=0;blocked=0
    for c,p,yaw in view_goals:
        near=tree.query_ball_point(p,.44) if tree else []
        if near and np.any(np.abs(wrap(past[near,2]-yaw))<np.deg2rad(27)):
            covered+=1;continue
        if any(np.linalg.norm(p-f[:2])<.24 and (len(f)<3 or abs(wrap(yaw-f[2]))<.5) for f in failed):blocked+=1;continue
        # The robot translates between lattice sites; yaw-only motions are not
        # treated as stereo depth measurements. Actual depth needs baseline.
        score=.9/(.7+costs[c]*res)
        candidates.append((score,c,yaw,'visual_view'))
    metrics={'frontier_groups':front_groups,'frontier_cells':int(front.sum()),'view_goals_total':len(view_goals),'view_goals_covered':covered,'view_goals_blocked':blocked,'view_coverage':covered/max(1,len(view_goals)),'free_area_m2':float(free.sum()*res*res),'reachable_area_m2':float(reachable.sum()*res*res)}
    if return_home:
        cells=np.argwhere(reachable);goal=tuple(cells[np.argmin(np.linalg.norm(np.array([world(c) for c in cells])-home,axis=1))]);yaw=None;kind='return_home'
        if np.linalg.norm(pos-home)<.16:metrics['reason']='home_reached';return None,None,metrics
    elif candidates:
        _,goal,yaw,kind=max(candidates,key=lambda c:c[0])
    else:
        metrics['reason']='no_unattempted_reachable_views';return None,None,metrics
    path=[goal]
    while path[-1]!=start:
        if path[-1] not in prev:metrics['reason']='unreachable';return None,None,metrics
        path.append(prev[path[-1]])
    path.reverse();metrics['reason']=kind
    return np.array([world(c) for c in path]),yaw,metrics

def completion(metrics,gains):
    """Completion of a declared finite exploration model, never dense 3D proof."""
    if metrics.get('frontier_groups',1)>0:return None
    if metrics.get('view_coverage',0)>=.97:return 'reachable_view_coverage_complete'
    # Track births or moving voxels cannot establish physical-scene novelty.
    # A plateau requires verified associations across revisits; do not infer it here.
    if metrics.get('reason')=='no_unattempted_reachable_views':return 'reachable_view_set_exhausted'
    return None
