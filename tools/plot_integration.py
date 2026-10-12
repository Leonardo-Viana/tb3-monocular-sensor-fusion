"""Render the published integration PLY directly; never fills missing geometry."""
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('ply', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
lines = args.ply.read_text().splitlines()
points = np.loadtxt(lines[lines.index('end_header') + 1:])
xyz, rgb = points[:, :3], points[:, 3:] / 255.
fig = plt.figure(figsize=(11, 5), layout='constrained')
ax = fig.add_subplot(121, projection='3d')
ax.scatter(*xyz.T, c=rgb, s=5, depthshade=False)
ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title='Monocular RGB landmarks')
ax.view_init(elev=24, azim=-65)
ax.set_box_aspect(np.maximum(np.ptp(xyz, axis=0), .5))
ax = fig.add_subplot(122)
ax.scatter(xyz[:, 0], xyz[:, 1], c=rgb, s=5)
ax.set(xlabel='X (m)', ylabel='Y (m)', title='Top view of the same points')
ax.set_aspect('equal')
ax.grid(alpha=.2)
fig.suptitle(f'Short integration test | {len(points):,} sparse landmarks | 12 October 2026', fontsize=14)
fig.supxlabel('Observed RGB colors; no surfaces filled. Estimated metric coordinates; no ground-truth accuracy claim.', fontsize=9)
fig.savefig(args.output, dpi=180)
