"""Reproduce replication figures from the compact evidence ZIP; no robot access.

Usage: python3 tools/plot_campaign.py data/publication-evidence-20261007.zip paper
Plot rendering is an offline operation, outside the measured Dell mission intervals.
"""
import argparse
import json
from pathlib import Path
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable


def main(archive, out):
    out.mkdir(exist_ok=True)
    with zipfile.ZipFile(archive) as z:
        summary = json.loads(z.read('analysis/ANALYSIS.json'))
        data = [json.loads(z.read(f'analysis/repeat_{i:02d}_plot_data.json')) for i in range(1, 4)]
        revisit = [json.loads(z.read(f'analysis/repeat_{i:02d}_revisit/revisit_matches.json')) for i in range(1, 4)]
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'pdf.fonttype': 42})
    fig = plt.figure(figsize=(13.5, 8.2), layout='constrained')
    axes3 = []
    norm = Normalize(vmin=0, vmax=1)
    for i, (d, report) in enumerate(zip(data, summary['runs'])):
        xyz = np.array(d['xyz']); rgb = np.array(d['rgb']) / 255
        path = np.array(d['path']); m = d['occupancy']
        grid = np.array(m['data']).reshape(m['height'], m['width'])
        yy, xx = np.where(grid >= 65)
        x = (xx+.5)*m['resolution']+m['position'][0]
        y = (yy+.5)*m['resolution']+m['position'][1]
        ax = fig.add_subplot(2, 3, i+1, projection='3d'); axes3.append(ax)
        ax.scatter(x, y, np.zeros_like(x), s=.5, c='#90989d', alpha=.2)
        ax.scatter(*xyz.T, c=xyz[:, 2], norm=norm, cmap='viridis', s=2, depthshade=False,
                   rasterized=True)
        ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)', xlim=(-1.2, 5),
               ylim=(-2.4, 3.5), zlim=(-.1, 1.05),
               title=f"Seed {report['seed']} · {len(xyz):,} sparse landmark IDs")
        ax.set_box_aspect((1, 1, .45)); ax.view_init(32, -63)
        ax = fig.add_subplot(2, 3, i+4)
        extent = [m['position'][0], m['position'][0]+m['width']*m['resolution'],
                  m['position'][1], m['position'][1]+m['height']*m['resolution']]
        ax.imshow(np.ma.masked_where(grid < 0, grid), origin='lower', extent=extent,
                  cmap='Greys', vmin=0, vmax=100, alpha=.4)
        ax.scatter(xyz[:, 0], xyz[:, 1], s=3, c=rgb, rasterized=True)
        ax.plot(path[:, 0], path[:, 1], color='#3478a0', lw=.7, alpha=.8)
        ax.scatter(0, 0, marker='*', s=80, c='#c14e25', zorder=6)
        metric = report['controller']['map_metrics']
        ax.set(xlabel='x (m)', ylabel='y (m)', aspect='equal', xlim=(-1.2, 5), ylim=(-2.4, 3.5),
               title=f"View lattice: {metric['view_goals_covered']}/{metric['view_goals_total']} ({100*metric['view_coverage']:.1f}%)")
    fig.colorbar(ScalarMappable(norm=norm, cmap='viridis'), ax=axes3, shrink=.67,
                 label='Reconstructed height (m)')
    fig.suptitle('Monocular mapping on the Dell · three completed simulated trials\n'
                 'Top: points colored by height. Bottom: original RGB colors, estimated path and planar LiDAR reference.',
                 fontsize=13)
    for extension in ('png', 'pdf'):
        fig.savefig(out / ('replication-maps-20261007.' + extension), dpi=180)
    plt.close(fig)
    fig, axes = plt.subplots(2, 2, figsize=(11.5, 8), layout='constrained')
    palette = ['#2265a4', '#b85b19', '#258367']
    for d, report, matches, color in zip(data, summary['runs'], revisit, palette):
        r = d['resources']; t = np.array([x['monotonic'] for x in r]); t = (t-t[0])/60
        label = f"Seed {report['seed']}"
        axes[0, 0].plot(t, [x['mem_available_mib']/1024 for x in r], color=color, label=label)
        axes[0, 1].plot(t, [x['system_cpu_percent'] for x in r], color=color, lw=.8, alpha=.8, label=label)
        nav = [(t[i], x['observed_rtf']) for i, x in enumerate(r)
               if x['controller_phase'] in ('frontier', 'visual_view', 'returning')
               and x['observed_rtf'] is not None and .5 <= x['sample_interval_s'] <= 8]
        axes[1, 0].plot(*np.array(nav).T, color=color, lw=.8, alpha=.9, label=label)
        strict = np.sort([100*x['distance_m'] for x in matches if x['epipolar_supported'] and x['repeat_support']])
        if len(strict):
            axes[1, 1].step(strict, np.arange(1, len(strict)+1)/len(strict), where='post',
                            color=color, label=f'{label} (n={len(strict)})')
    axes[0, 0].set(title='Available system RAM', xlabel='Wall time from trial start (min)', ylabel='GiB available', ylim=(0, 4.5))
    axes[0, 0].axhline(450/1024, color='#777', ls='--', lw=.8)
    axes[0, 1].set(title='System CPU · all 12 logical CPUs', xlabel='Wall time from trial start (min)', ylabel='System utilization (%)', ylim=(0, 100))
    axes[1, 0].set(title='Simulation speed during navigation phases', xlabel='Wall time from trial start (min)', ylabel='Simulated / wall seconds', ylim=(.75, 1.01))
    axes[1, 0].axhline(1, color='#777', ls='--', lw=.8)
    axes[1, 1].set(title='Exploratory revisit consistency · both checks', xlabel='3D separation of associated landmark IDs (cm)', ylabel='Empirical cumulative fraction', xlim=(0, 40), ylim=(0, 1.04))
    for ax in axes.ravel():
        ax.grid(alpha=.18); ax.legend(fontsize=8, frameon=False)
    fig.suptitle('Resource feasibility does not establish global map accuracy\n'
                 'Approximately 5 s resource samples. Revisit candidates are not independent ground truth.', fontsize=13)
    for extension in ('png', 'pdf'):
        fig.savefig(out / ('replication-diagnostics-20261007.' + extension), dpi=180)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path); parser.add_argument('output', type=Path)
    args = parser.parse_args(); main(args.archive, args.output)
