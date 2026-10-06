"""Extract descriptive tables from saved evidence; no reconstruction or ROS."""
import argparse
import csv
import io
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check committed summaries without writing')
    args = parser.parse_args()
    source = ROOT / 'experiments/2026-10-02-guarded/COMPARISON.json'
    comparison = json.loads(source.read_text())
    rows = []
    for name in ('baseline', 'guarded_trial'):
        run = comparison[name]
        status, stable = run['status'], run['stable']
        pixel = stable['test_pixels']
        epi = run['revisit']['epipolar_subset']
        rows.append(dict(
            experiment=name,
            view_coverage_fraction=status['map_metrics']['view_coverage'],
            accepted_landmark_ids=stable['accepted_landmarks'],
            fitted_keyframes=stable['frames'], reserved_pixel_observations=pixel['n'],
            median_px=pixel['median'], p90_px=pixel['p90'], maximum_px=pixel['max'],
            observations_above_8px=stable['test_pixels_over_8px'],
            total_wall_s=run['all_segments_wall_s'],
            estimated_travel_m=run['all_segments_estimated_travel_m'],
            reached_goals=run['all_segments_goals'],
            epipolar_pairs=epi['landmark_pairs'],
            epipolar_block_pairs=len(epi['block_pairs']),
            epipolar_median_m=epi['euclidean_m']['median'],
        ))
    payload = {'interpretation': 'Descriptive development comparison; unmatched starts/routes/budgets; internal pixel tests; no ground-truth accuracy claim.', 'source': str(source.relative_to(ROOT)), 'runs': rows}
    csv_buffer = io.StringIO(newline='')
    writer = csv.DictWriter(csv_buffer, fieldnames=list(rows[0]), lineterminator='\n')
    writer.writeheader(); writer.writerows(rows)
    lines = ['# Descriptive mission comparison', '', payload['interpretation'], '',
             '| Metric | Baseline | Guarded repeat |', '| --- | ---: | ---: |']
    fields = [('view_coverage_fraction', 'Planned view coverage, fraction'),
              ('accepted_landmark_ids', 'Accepted landmark IDs'),
              ('reserved_pixel_observations', 'Reserved pixel observations'),
              ('median_px', 'Median reprojection error, px'),
              ('p90_px', '90th percentile, px'), ('maximum_px', 'Maximum, px'),
              ('observations_above_8px', 'Observations above 8 px'),
              ('total_wall_s', 'Combined controller wall time, s'),
              ('estimated_travel_m', 'Estimated travel, m'),
              ('reached_goals', 'Reached goals, all segments'),
              ('epipolar_pairs', 'Epipolar-supported revisit pairs'),
              ('epipolar_block_pairs', 'Block pairs supporting that check'),
              ('epipolar_median_m', 'Median candidate revisit discrepancy, m')]
    def display(value):
        return f'{value:.4f}' if isinstance(value, float) else str(value)
    for key, label in fields:
        lines.append(f'| {label} | {display(rows[0][key])} | {display(rows[1][key])} |')
    lines += ['', 'Pixel units refer to 960 × 540 working images. Coverage is not surface completeness. The baseline totals include its initial timeout segment. Revisit subsets have different spatial support; consult the English report before interpreting differences.', '']
    outputs = {'summary.json': json.dumps(payload, indent=2) + '\n', 'summary.csv': csv_buffer.getvalue(), 'summary.md': '\n'.join(lines)}
    for name, content in outputs.items():
        target = ROOT / 'results' / name
        if args.check:
            if not target.exists() or target.read_text() != content:
                raise SystemExit('Summary missing or stale: ' + str(target))
        else:
            target.write_text(content)
    print('PASS: summaries match saved evidence' if args.check else 'Wrote JSON, CSV and Markdown summaries')

if __name__ == '__main__':
    main()
