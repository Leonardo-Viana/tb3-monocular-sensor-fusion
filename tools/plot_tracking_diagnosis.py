"""Render observed RGB and diagnostic projections; never alters sensor data."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('diagnosis',type=Path)
ap.add_argument('images',type=Path)
ap.add_argument('output',type=Path)
args=ap.parse_args()
d=json.loads(args.diagnosis.read_text())
fig,axes=plt.subplots(2,2,figsize=(12,7.1),layout='constrained')
for row,(finding,ids) in enumerate(zip(d['findings'],[(113,114),(209,210)])):
    by_id={o['frame']:o for o in finding['observations']}
    for col,fid in enumerate(ids):
        ax=axes[row,col];o=by_id[fid]
        ax.imshow(plt.imread(args.images/f'raw_{fid:05d}.png'))
        u,v=o['uv'];x,y=o['optimized_prediction_uv']
        ax.plot(u,v,'rx',markersize=10,markeredgewidth=2)
        if 0<=x<960 and 0<=y<540:
            ax.plot(x,y,'o',markeredgecolor='#00c7df',markerfacecolor='none',markersize=12,markeredgewidth=2)
            if np.hypot(x-u,y-v)>8:
                ax.annotate('',xy=(x,y),xytext=(u,v),arrowprops={'arrowstyle':'<->','color':'#00c7df','lw':1.8})
        else:
            ax.annotate('Fitted feature is outside the image',xy=(0,max(15,y)),xytext=(210,28),
                        color='#003e48',fontsize=9,bbox={'facecolor':'white','alpha':.9,'edgecolor':'none'},
                        arrowprops={'arrowstyle':'->','color':'#00c7df','lw':2})
        ax.set_title(f'Track {finding["track_id"]} | frame {fid} | residual {o["error_px"]:.2f} px',fontsize=11)
        ax.set(xlim=(0,960),ylim=(540,0));ax.set_axis_off()
fig.suptitle('Endpoint correspondence failures reproduced from recorded RGB',fontsize=15)
fig.supxlabel('Red cross: saved observation. Cyan circle: fitted projection (diagnostic only). Working resolution: 960 × 540.',fontsize=10)
fig.savefig(args.output,dpi=160)
