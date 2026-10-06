# Descriptive mission comparison

Descriptive development comparison; unmatched starts/routes/budgets; internal pixel tests; no ground-truth accuracy claim.

| Metric | Baseline | Guarded repeat |
| --- | ---: | ---: |
| Planned view coverage, fraction | 0.9737 | 0.9750 |
| Accepted landmark IDs | 4020 | 4029 |
| Reserved pixel observations | 9859 | 8920 |
| Median reprojection error, px | 0.2807 | 0.2400 |
| 90th percentile, px | 0.8547 | 0.7478 |
| Maximum, px | 264.9458 | 7.6885 |
| Observations above 8 px | 1 | 0 |
| Combined controller wall time, s | 2109.0000 | 1955.0000 |
| Estimated travel, m | 31.9211 | 37.7019 |
| Reached goals, all segments | 74 | 99 |
| Epipolar-supported revisit pairs | 89 | 52 |
| Block pairs supporting that check | 4 | 1 |
| Median candidate revisit discrepancy, m | 0.0810 | 0.0581 |

Pixel units refer to 960 × 540 working images. Coverage is not surface completeness. The baseline totals include its initial timeout segment. Revisit subsets have different spatial support; consult the English report before interpreting differences.
