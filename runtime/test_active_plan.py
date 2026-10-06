import numpy as np
from active_plan import choose,completion
from scipy.ndimage import distance_transform_edt
# Closed room: all reachable free cells, but no frontier into the outside.
g=np.full((80,80),-1);g[5:75,5:75]=0;g[5,5:75]=100;g[74,5:75]=100;g[5:75,5]=100;g[5:75,74]=100;g[32:42,32:42]=100
r=.06;o=np.array([-1.,-1.]);pos=np.array([0.,0.]);home=pos.copy()
p,y,m=choose(g,r,o,pos,[],[],home)
assert p is not None and m['frontier_groups']==0 and m['view_coverage']==0
ix=np.floor((p-o)/r).astype(int);assert np.all(g[ix[:,1],ix[:,0]]==0)
clear=distance_transform_edt(g<50)*r;assert np.all(clear[ix[:,1],ix[:,0]]>.34)
# A heading observed at a position covers that camera view, not its opposite.
views=[[x,y,a] for x in np.arange(-.6,3.6,.6) for y in np.arange(-.6,3.6,.6) for a in np.arange(8)*np.pi/4]
p,y,m=choose(g,r,o,pos,views,[],home)
assert m['view_coverage']>.97 and completion(m,[])=='reachable_view_coverage_complete'
# Information saturation alone cannot declare completion before visual coverage.
assert completion({'frontier_groups':0,'view_coverage':.2},[0,0,0]) is None
assert completion({'frontier_groups':1,'view_coverage':1},[0,0,0]) is None
assert completion({'frontier_groups':0,'view_coverage':.91},[.001,0,.002]) is None
# A newly opened boundary must produce a frontier, not completion.
g[5,20:60]=0
_,_,m=choose(g,r,o,pos,views,[],home)
assert m['frontier_groups']>0
print('PASS: inflated paths, directional coverage, hidden frontier and honest completion rules')
