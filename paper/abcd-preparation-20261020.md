# A/B/C/D comparison preparation and current blocker

20 October 2026. **No comparative A/B/C/D result is available yet.**

Status rechecked on 22 October 2026: the target Dell still appears offline.
The protocol and three utility files are already published; publication of this
status report and the preserved 20 October check outputs was resumed today.

The four treatments were authorized after the structural-fusion design review.
The target `orion-Vostro-3710` was reported offline by Desktop Commander when
execution was attempted. The portable copies inspected here contain selected
RGB images, maps, metadata and reconstruction outputs; no full MCAP/DB3 payload
was found in the inspected source trees. Metadata cannot replace those inputs.
This is an access/data blocker, not a measured hardware-capacity failure.

## Work completed

- Prepared a [comparison protocol](../docs/abcd-protocol-20261020.md) that defines
  A as the preserved baseline, B as XFeat points/matching, C as B plus verified
  structural LiDAR constraints, and D as C plus MobileSAM mask support. The
  initial B arm does not also add a line matcher; that would be a separate
  extension. The initial D arm does not claim dynamic-object understanding.
- Identified candidate upstream source revisions; exact model-weight hashes and
  target-compatible dependency locks remain pending installation on the Dell.
- Implemented isolated [geometry helpers](../tools/structural_constraints.py):
  calibrated camera rays, intersections with finite vertical-wall hypotheses,
  rejection of grazing/behind-camera/out-of-support candidates, ambiguity checks
  against independent visual estimates, and same-image mask support.
- Ran [14 analytic development checks](../results/abcd-geometry-checks-20261020.json),
  all passing. They include frame-change invariance, the known-height example,
  tilted scan endpoints, foreground disagreement, competing structures, image
  boundaries and duplicate mask-support pixels. Test code is
  [available here](../tools/test_structural_constraints.py).
- Ran the [input audit](../tools/preflight_abcd.py), retaining its
  [blocked result](../results/abcd-input-audit-20261020.json). Its exit code was 2,
  as intended for missing inputs. The audit starts no ROS or robot process.

## What the checks do not establish

The geometry helpers propose candidates; they cannot determine whether an
observed wall is vertical, prove visibility, infer scene height or solve
cross-sensor association by themselves. A segmentation mask is not a metric
surface. Plane residual scales in the helper are declared engineering scales,
not calibrated total uncertainty. Analytic tests are not real-image benchmarks.

XFeat/MobileSAM inference adapters, scan-segment extraction, temporal association,
plane-factor optimization and the common A/B/C/D evaluator are **not integrated**.
The live runtime is unchanged and does not import the new helpers. No package
was installed on the Dell, no robot was moved, and no new map was reconstructed.
No latency, RAM, accuracy gain or publication-readiness claim follows from this
preparation. The older acquisition-loss/global-consistency limitations remain.

## Next executable sequence

1. Reconnect Desktop Commander on the original Dell and audit the retained raw
   recording, source hashes, timestamps, intrinsics and extrinsics.
2. Complete and validate the adapters in an isolated environment; reproduce A
   before introducing B, C and D. Freeze the executable manifest after development
   and before fresh validation, retaining any development changes explicitly.
3. Replay identical inputs through all four arms, recording failures, geometric
   quality, coverage, latency and resource cost. Offline replay and full-stack
   Gazebo measurements are distinct scenarios; neither can stand in for the other.
4. Use new scenes/trajectories for independent validation after development.

To repeat the portable checks from the repository root:

```bash
python3 -m unittest discover -s tools -p test_structural_constraints.py -v
python3 tools/preflight_abcd.py /absolute/path/to/recorded/output --output /new/path/input-audit.json
```

The first command requires NumPy. The second uses Python's standard library,
refuses to overwrite an existing report and checks input presence only. It does
not claim that finding a bag proves its contents are valid or all arms are ready.

All experimental outcomes remain pending. Reconnecting the target provides the
missing execution environment; it does not itself complete B/C/D integration.
