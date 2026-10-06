"""Occupancy-grid exploration; knows only the map produced from live lidar."""
import heapq,math
import numpy as np
from scipy.ndimage import distance_transform_edt,binary_dilation,label,uniform_filter

def dijkstra(walk,start,clearance):
    h,w=walk.shape;dist=np.full((h,w),np.inf);prev={};dist[start]=0.;heap=[(0.,start)]
    steps=[(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    while heap:
        d,p=heapq.heappop(heap)
        if d!=dist[p]:continue
        for dy,dx in steps:
            q=(p[0]+dy,p[1]+dx)
            if not (0<=q[0]<h and 0<=q[1]<w and walk[q]):continue
            if dx and dy and (not walk[p[0],q[1]] or not walk[q[0],p[1]]):continue
            nd=d+math.hypot(dx,dy)*(1+.05/max(clearance[q]-.22,.02))
            if nd<dist[q]:dist[q]=nd;prev[q]=p;heapq.heappush(heap,(nd,q))
    return dist,prev

def plan(grid,res,origin,position,visited,failed,home=None,return_home=False):
    occupied=grid>=50;free=(grid>=0)&(grid<30)
    clearance=distance_transform_edt(~occupied)*res
    walk=free&(clearance>.34)
    cell=np.floor((np.asarray(position)-origin)/res).astype(int)[::-1]
    positions=np.argwhere(walk)
    if not len(positions):return None,'no_free_cells',{}
    ds=np.linalg.norm((positions-cell)*res,axis=1);start=tuple(positions[ds.argmin()])
    if ds.min()>.3:return None,'start_clearance',{}
    costs,prev=dijkstra(walk,start,clearance);reachable=np.isfinite(costs)
    frontier=walk&binary_dilation(grid<0,iterations=3)&reachable
    labs,n=label(frontier);candidates=[]
    information=uniform_filter((grid<0).astype(float),size=25)
    def world(c):return origin+(np.array(c)[::-1]+.5)*res
    if not return_home:
        for k in range(1,n+1):
            coords=np.argwhere(labs==k)
            if len(coords)<3:continue
            ranked=sorted(coords,key=lambda c:costs[tuple(c)])
            c=tuple(ranked[len(ranked)//3]);p=world(c)
            if np.linalg.norm(p-position)<.22:continue
            if any(np.linalg.norm(p-v)<.3 for v in failed):continue
            score=(.3+information[c])/(.7+costs[c]*res)
            candidates.append((score,c,'frontier'))
        if not candidates:
            for y,x in np.argwhere(reachable&(clearance>.36))[::15]:
                c=(int(y),int(x));p=world(c)
                if np.linalg.norm(p-position)<.45:continue
                if any(np.linalg.norm(p-v)<.5 for v in visited):continue
                if any(np.linalg.norm(p-v)<.3 for v in failed):continue
                candidates.append((.2/(1+costs[c]*res),c,'visual_coverage'))
    if candidates:
        _,goal,kind=max(candidates,key=lambda v:v[0])
    elif home is not None:
        coords=np.argwhere(reachable)
        k=np.argmin(np.linalg.norm(np.array([world(c) for c in coords])-home,axis=1))
        goal=tuple(coords[k]);kind='return_home'
        if np.linalg.norm(world(goal)-position)<.18:return None,'home_reached',{'frontier_cells':int(frontier.sum())}
    else:return None,'no_reachable_frontier',{}
    path=[goal]
    while path[-1]!=start:
        if path[-1] not in prev:return None,'path_missing',{}
        path.append(prev[path[-1]])
    path.reverse()
    return np.array([world(c) for c in path]),kind,{'frontier_cells':int(frontier.sum()),'reachable_cells':int(reachable.sum()),'known_cells':int((grid>=0).sum()),'free_area_m2':float(free.sum()*res*res)}
