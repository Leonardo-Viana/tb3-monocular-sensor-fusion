# Bounded-ray tracking follow-up: frozen short live test

The candidate was developed after inspecting the 12 October integration recording.
That recording and all earlier replication runs are development data. No result
from replaying them is an independent accuracy benchmark.

## Candidate fixed before the new recording

The unchanged Lucas–Kanade tracker proposes a correspondence. In addition to its
existing status, photometric, forward/backward and image-border checks, the
candidate projects the old ray into the new camera using the recorded/fused
poses. Optical depth must lie between 0.15 and 6 m in both views, matching the
reconstructor's working range. The observation must be within 12 pixels of that
projected segment in both directions, at 960 × 540. No map residual, reserved
test residual, simulator pose, scene geometry, depth image or compass is used.
This is a heuristic allowance for pose/pixel error, not a calibrated probability.
Rotation initialization of LK is deliberately not added in this comparison.

## One new engineering test

- Fresh output root: `/home/lpviana/tb3_ray_gate_live_20261013`.
- Original Dell i5-12400, 8 GiB RAM, Intel UHD 730.
- Archived textured scene used only by Gazebo; simulation seed 26.
- ROS domain 77, localhost-only discovery, partition `tb3_ray_gate_live_20261013`.
- 180 s navigation budget; final fitting may run afterward; outer deadline 1200 s.
- Same controller, planning and reconstruction thresholds; only the tracking
  gate changes. One RViz, Gazebo server without GUI.
- Same memory/disk guards: start with at least 1700 MiB available and 9 GiB free;
  stop below 450 MiB available or 6 GiB free.
- Record all configured sensor topics and five-second resource samples.
- All 31 existing integration checks must pass. Retain every failed check,
  transport loss and shutdown escalation.
- Additional visual-tail criterion fixed now: at least 30 scored reserved
  observations, no unscored accepted landmarks, and no residual above 8 px.
  Report all counts above 3/8 px and maximum, even if aggregate pixel gates pass.
- Preserve the entire run if the visual-tail criterion fails. Do not tune the
  threshold or repeat the test to replace a failed result.

The protocol file is saved and hashed on the Dell before launch. GitHub upload
may occur later and is not external preregistration. This is a fresh recording
in a previously studied scene, not an independent scene or a paired comparison
against a live baseline. It tests short integration and the declared pixel tail;
it cannot establish absolute 3D accuracy, full coverage, global submap alignment,
sensor contribution, or publication readiness. The earlier 32 cm discrepancy
remains a separately reported unresolved result.
