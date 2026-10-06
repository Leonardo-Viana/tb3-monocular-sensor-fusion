# Guarded monocular mapping: completed prospective repeat

Experiment date: 2 October 2026. Platform: simulated TurtleBot3 Waffle on an Intel i5-12400 desktop with 8 GiB RAM and integrated Intel graphics. This is an experimental report for a future article, not a claim of validated dense reconstruction.

## Outcome

The autonomous mission finished with 156 of 160 planned reachable views covered (97.5%), no remaining detected frontier groups, 99 reached goals, eight failed goals, and no reverse recovery. Three view goals were marked blocked. The controller returned to within 0.106 m of its estimated home position and stopped. Controller elapsed wall time, including final refinement, was 1,955 s (32.6 min); estimated travel was 37.70 m. Neither distance is a ground-truth measurement.

The saved sparse map contains 4,029 accepted landmark IDs from 1,948 fitted keyframes (1,949 captured). IDs are not necessarily unique physical scene points. Of 31 reconstruction blocks, block 3 did not converge; its 259 fit candidates contributed no accepted landmarks. Finite view coverage measures the exploration policy's planned observations, not reconstructed surface completeness.

Final verification at 18:46:10 UTC confirmed a last audited velocity command of zero, a paused Gazebo simulation, one RViz instance, and successful supervisor, recorder finalizer, and evaluator exits. Minimum recorded available memory was 741.6 MiB. Motion publication is capped at 20 Hz and emergency-zero publication at 10 Hz. The isolated guard regression produced 35 zero commands in four seconds under a high-rate clock publisher.

## Sensor roles and exclusions

All reconstructed 3D landmark coordinates originate from monocular RGB tracks and multi-view triangulation. A planar filter fuses wheel velocities and raw gyro Z, with gyro bias estimation. The resulting heading affects integrated XY displacement. Accelerometer magnitude supports stationary-update gating; raw gyro XY and gravity support online tilt estimation. Acceleration is not double-integrated for position. Planar LiDAR SLAM supplies map-frame pose corrections and occupancy for navigation.

The stable image fitter starts with yaw-only poses and a fixed camera-height prior, then permits regularized local pose corrections. It does not implement IMU preintegration or a jointly optimized visual-inertial SLAM backend. No RGB-D/depth, magnetometer, IMU orientation field, simulator ground-truth pose, or world geometry was consumed as estimator or controller input. LiDAR returns were not extruded into 3D. The simulator was reset for initialization only.

## Predeclared tracking change

The guarded tracker ends active tracks across image gaps greater than 0.65 simulation seconds, camera rotations greater than 0.18 radians, or invalid/nonmonotonic timestamps and poses. It compares against the last processed frame. During this run, it reset 140 times for gaps and ten times for rotation. Such resets can reduce false associations but also fragment tracks. Feature-start and observation counters are not counts of independent physical landmarks.

This change followed diagnosis of an original 264.95-pixel test outlier: optical flow had associated repeated texture across a 1.089-second, 17.48-degree transition despite a small forward-backward residual. Critical estimator, tracker, planner, and command-guard source hashes were frozen before motion and matched after the run. Geometry thresholds and the revisit audit were not retuned during the repeat. English reports and the figure were finalized afterward.

## Reserved pixel observations

Pixel errors below refer to 960 by 540 working images, resized from the original 1920 by 1080 RGB stream. Test observations were reserved within existing tracks and were not used for landmark acceptance. These are internal reprojection tests, not independent-journey tests or absolute metric accuracy.

| Metric | Preserved baseline | Guarded repeat |
| --- | ---: | ---: |
| Accepted landmark IDs | 4,020 | 4,029 |
| Fitted keyframes | 1,822 | 1,948 |
| Reserved test observations | 9,859 | 8,920 |
| Median reprojection error, px | 0.281 | 0.240 |
| 90th percentile, px | 0.855 | 0.748 |
| Maximum error, px | 264.946 | 7.689 |
| Observations above 3 px | 23 | 14 |
| Observations above 8 px | 1 | 0 |
| Accepted IDs without test observations | 2 | 0 |

All test outliers remain in the reported statistics. A comparably large outlier did not recur in this mission. This single adaptive run does not isolate a causal tracker benefit.

## Consistency across revisits

The unchanged exploratory audit uses RGB descriptor matches between temporally separated views, with optional epipolar and repeated-image checks. It does not select matches by 3D distance or perform ICP/3D registration. Candidate correspondences may still be false. Separate landmark IDs thought to observe the same feature are compared in the saved global coordinates.

| Association subset | Baseline pairs | Repeat pairs | Baseline median / p90 / max, cm | Repeat median / p90 / max, cm |
| --- | ---: | ---: | --- | --- |
| Raw RGB candidates | 162 | 77 | 8.80 / 15.89 / 154.03 | 6.02 / 16.88 / 113.17 |
| Epipolar check | 89 | 52 | 8.10 / 13.99 / 18.35 | 5.81 / 7.85 / 11.18 |
| Repeated-image check | 21 | 13 | 4.84 / 13.36 / 13.97 | 5.99 / 8.73 / 17.97 |
| Both checks | 19 | 10 | 4.60 / 13.43 / 13.97 | 5.97 / 8.66 / 8.74 |

The repeat's 52 epipolar pairs cover only blocks 8 and 9; the baseline covers four block pairs. The stricter repeat subset also covers only blocks 8 and 9, and its median discrepancy is higher than the baseline's. Thus the smaller epipolar median does not establish improved global consistency. The raw candidates retain a 1.13 m tail, which may include false matches. Sparse support and repeated textures remain material limitations.

Candidate views require at least 40 simulation seconds separation, estimated camera separation no greater than 0.60 m, optical-direction difference no greater than 30 degrees, and at least 12 accepted landmarks per image. SIFT descriptors use recorded training/validation pixels with a fixed 24-pixel scale and mutual ratio below 0.70. The additional fundamental-matrix check uses a 2-pixel RANSAC threshold, at least 12 image matches, and adequate image spread. Test pixels are not reused to select these associations. These checks are diagnostics, not certified associations or geometric ground truth.

## Comparison limits

Only one adaptive mission per configuration is available. Routes, budgets, initial physical poses, and view sets were not matched. The baseline began from an existing paused simulation after a prior pilot; the repeat began after an authored-world reset. No ground-truth pose was read to equalize these starts. The baseline also includes a preserved synchronous-refinement timeout segment, followed by an asynchronous continuation. Its combined segments total 2,109 s, 74 reached goals, and 31.92 m estimated travel, compared with 1,955 s, 99 goals, and 37.70 m here. These figures are descriptive, not an efficiency benchmark or controlled ablation.

The repeat used asynchronous refinement from the start with a 2,400-second navigation budget. An access-review usage-limit delay occurred before the mission; the repeat had not started then. A temporary remote-monitoring connector outage occurred during the mission without restarting the robot process. Neither interruption was hidden as a successful run. The original dataset remains unchanged, verified by hashes.

## Evidence and preservation

The Dell retains the complete new run at `/home/lpviana/tb3_guarded_trial_20261002`. Raw MCAP files total 4,077,517,114 bytes. Traversed per-topic message counts match recorder metadata, and the first and last recorded RGB images decode at 1920 by 1080 pixels. This checks recorded-file integrity, not zero acquisition loss. SHA-256 hashes are in `output/FINAL_REPORT.json`.

The portable archive contains source code, the protocol, source verification, final PLY and landmark JSON, numerical validation, occupancy snapshot, command audit, revisit summaries, and this report. Full MCAP files, JPEG frames, keyframes, and tracks remain on the Dell and are required for complete reprocessing. `AUTO_COMPARISON.md` and `COMPARISON.json` preserve the automated comparison. `mission_map.png` and `mission_map.pdf` show the saved sparse estimate, without smoothing, meshing, or invented surfaces. Blue-grey ground-plane marks in the 3D panel are a LiDAR occupancy reference projected to z=0, not reconstructed scene height. Landmark RGB colors are inherited from their tracked image pixels and were not photometrically recalibrated.

## Next research step

The completed repeat supports operation of the guarded monocular reconstruction and demonstrates the absence of the prior extreme reprojection failure in this run. It does not establish a dense, globally consistent, or absolutely accurate map. The next step is broader cross-revisit association and global consistency evaluation, followed by matched repeated trials and explicit ablations of the pose sensors. Any global correction should be validated on reserved associations and retain the original map as a separate baseline.
