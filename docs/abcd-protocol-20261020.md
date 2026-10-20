# A/B/C/D comparison: implementation and evaluation protocol

Prepared 20 October 2026. **Status: development protocol, not a completed
comparison or an externally preregistered study.** The target Dell is offline;
the portable archive lacks full raw sensor recordings. No A/B/C/D benchmark
has been executed. The numerical association choices below are initial
development settings, not thresholds optimized on new test data.

## Question and treatments

Can learned visual correspondence, verified planar-LiDAR structure and image
segmentation improve sparse monocular mapping under the original Dell budget?

| Arm | Proposed treatment | Control |
| --- | --- | --- |
| A | Existing Shi–Tomasi/LK tracker with bounded-ray gate and stable reconstruction | Preserve behavior at project commit `12f8713beadfcf7a2b9c746a55b2d9f3e8484d83` |
| B | XFeat sparse points/descriptors with mutual matching; same metric pose inputs and reconstruction policy | No direct LiDAR depth or segmentation; image lines are deferred to a separate B-lines extension |
| C | B plus independently supported finite wall/edge constraints from 2D scan segments | No scene geometry, inferred arbitrary wall height or unverified associations |
| D | C plus MobileSAM region support on selected frames | Geometry checks remain mandatory; masks are not class labels, depth, object identity or proof of motion |

This specifies the point-feature option of the earlier "points and/or lines"
proposal so B changes one frontend family. B includes both detection and
matching; a result cannot be attributed to detection alone. Optional detector-
only or line-feature studies receive separate names and are not silently folded
into B. D initially tests segmentation-supported association; dynamic-scene
rejection requires a separately validated motion model and moving-object data.

## Preserve and identify inputs

- Treat all recordings examined before this protocol as development data.
  Begin with the short 13 October recording after auditing its retained data.
  Identify the actual absolute source path on the Dell instead of guessing it.
- Recover original compressed RGB, CameraInfo, LaserScan, raw IMU, wheel data,
  TF and fused-pose/graph history. Depth, compass, `Imu.orientation`, simulator
  truth and world geometry are excluded from estimator/controller inputs.
- Check storage hashes, per-topic timestamps/rates/gaps, RGB frame coverage,
  optical-frame conventions, distortion and camera/LiDAR/IMU/wheel extrinsics.
  Do not silently substitute saved preview JPEGs or invent missing scans.
- Freeze source hashes, the selected frame-time schedule, calibration and pose
  history before comparing arms. Use one decoder and identical decoded pixels.
  Keep whole source runs together in development/test splits.
- Build a common frame-pair/region evaluation list before seeing arm outputs.
  Internal per-track reserved pixels are secondary diagnostics: different
  trackers create different IDs, tracks and observation splits. Lower residuals
  on a smaller selected set alone do not establish better mapping.

## Model and software identity

Candidate source revisions verified on 20 October:

- XFeat: `verlab/accelerated_features` at
  `e92685f57f8318b18725c5c8c0bd28c7fe188d9a`.
- MobileSAM: `ChaoningZhang/MobileSAM` at
  `f706ad9c4eb7f219c00d9050e46328518ffb65d2`; use the original `vit_t` model,
  not a silent switch to MobileSAMv2.

Model-weight SHA-256 values, compatible Python/PyTorch versions and the complete
dependency lock are **pending actual installation**. Do not use an unpinned
`torch.hub` remote load in an experiment. Install in an isolated environment;
retain the working ROS environment. Store model licenses and training provenance.
No cloud inference or outside GPU contributes to an online-Dell claim.

## Implementation gates

1. Reproduce A on the retained recording using the actual current node replay.
   Validate any refactoring against serialized tracks before replacing a path.
2. Integrate XFeat without changing shared image resolution (960 x 540), motion
   schedule, reconstruction acceptance thresholds or pose history. Initial cap:
   450 points/frame. Use mutual descriptor matches with explicit one-to-one
   track assignment; record matcher thresholds in the frozen run manifest.
   Preserve the bounded-ray and temporal-discontinuity checks.
3. Extract finite scan segments with uncertainty, gap/occlusion handling and
   time-aligned projection. A scan segment alone cannot prove a vertical wall.
   Require independent training-view evidence; reject competing structures,
   grazing rays and unsupported extents. Do not use validation/test observations
   to build plane assignments. Initial geometry checks in the companion module:
   incidence cosine >= 0.10, ray distance 0.15–6 m, 3 cm horizontal support
   margin and 10 cm agreement with an independent visual training estimate.
   The ray distance is Euclidean; the old reconstructor's optical-z bound remains
   separate. These gates are hypotheses, not calibrated uncertainty models.
4. Implement robust plane factors and replay the same backend with zero added
   factors to check compatibility. A geometry helper is not that integration.
   Account for shared LiDAR/pose information; do not treat their errors as
   independent and report an overconfident posterior. A first conditional-pose
   analysis must explicitly forgo a full joint-fusion/covariance claim.
5. For D, initially evaluate one frame in five, with no result-dependent frame
   selection. Use geometric scan-support prompts, bounded candidate counts and
   no manually supplied object/scene labels. Require feature/multiple scan
   projections within the same mask and away from its boundary, in addition to
   C's checks. Without a validated mask propagation mechanism, use masks only
   on their own frame. Log embedding, prompt and mask processing costs.
6. Freeze all completed adapters, model hashes, thresholds, actual source paths,
   planned orders and evaluator inputs in a machine-readable run manifest before
   fresh validation. No implementation is declared ready merely from this file.

The companion `tools/structural_constraints.py` implements only ray/finite-wall
geometry, rejection of ambiguous visual support, and same-frame mask support.
It does not implement scan extraction, XFeat/MobileSAM inference, association
across time, factor optimization or ROS integration. It is not imported by the
unchanged live runtime.

## Execution and resources

Run one arm at a time on the original i5-12400 / 8 GiB / UHD 730 Dell. First use
offline replay without Gazebo or RViz to isolate estimator cost; report it as
offline. Then measure a separately labeled full-stack scenario with identical
simulator, recorder and visualization settings. Never compare an offline arm
against another running under Gazebo and call that a model speedup.

For timing, use three measured repetitions in counterbalanced orders
`A B C D`, `B D A C`, `D C B A`. Report cold-start/model-load time separately;
perform the same declared warmup before steady-state measurements. Timing repeats
of one recording are not independent accuracy trials. Start with OpenCV and
PyTorch intraop threads fixed to one, record interop/BLAS settings, and profile
any alternative thread count as a separate configuration.

Record per-stage wall/CPU time, p50/p90/p99 latency, process peak RSS/PSS where
available, total system memory, swap, drops and processed/skipped frame counts.
Keep a lossless fixed-schedule comparison separate from a paced-replay test with
bounded queues and explicit frame dropping. Cached model outputs may support
debugging but cannot supply an end-to-end inference timing measurement.

Reuse the earlier engineering guards unless an explicit protocol revision is
made: at least 1700 MiB available and 9 GiB free before starting; stop below
450 MiB available or 6 GiB free. Record OOM, timeout, missing dependency and
insufficient memory as failures. An offline device is an access blocker, not
evidence that a model exceeded the hardware budget.

## Analysis and limits

- Report accepted and rejected correspondences, manually audited association
  precision on the common evaluation list, distribution of track lengths,
  accepted points and scored observations; never rank by point count alone.
- Report reserved pixel residuals including tails, missing observations, scale,
  cross-revisit consistency, duplicate structures and false surface creation.
- If independent metric reference data exist, use a separate evaluator for
  trajectory ATE/RPE and surface accuracy/completeness. Do not feed truth into
  estimation, navigation, association or parameter tuning on the held-out set.
  Without that reference, label absolute accuracy/completeness unmeasured.
- Fixed fused poses isolate reconstruction/frontend changes and cannot establish
  improved trajectory accuracy or the independent contribution of IMU/wheels.
  Sensor-removal ablations are a different experiment from A/B/C/D.
- Test repeated structures, foreground occluders and slanted surfaces in fresh
  scenes after development. Keep trajectories identical for replay comparison;
  closed-loop navigation runs are a later study because actions alter inputs.
- Preserve every arm, failure and rerun. No best-run selection, claims of
  statistical significance from one bag, or claim of full 3D completion.

C/D are **LiDAR-constrained monocular reconstruction**. A/B retain RGB-derived
landmarks supported by multisensor poses. No arm uses RGB-D, a compass or
generated hidden surfaces.
