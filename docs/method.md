# Sensor roles and interpretation

The planar EKF uses wheel linear/angular velocities and raw gyro Z, with a gyro-bias state. Heading affects integrated XY displacement. Accelerometer magnitude gates stationary updates; raw gyro XY and gravity support online tilt estimation. Acceleration is not double-integrated into position.

Planar LiDAR SLAM consumes the filtered odometry transform and laser scans, producing map-frame pose corrections and the navigation occupancy map. RGB image tracks are triangulated using metric pose priors. Local submaps are refined with geometric and held-out-observation checks.

The stable reconstruction starts from yaw-only poses and a fixed camera height, then allows regularized local pose corrections. It does not optimize IMU preintegration factors jointly with images. Cross-revisit association remains an exploratory audit rather than an established global landmark graph.

The guarded tracker resets across processed-frame gaps above 0.65 simulated seconds or camera rotation above 0.18 rad, and invalid/nonmonotonic time/pose. Reconstruction thresholds were frozen during the prospective repeat. Asynchronous refinement and a zero-command publication rate cap were active from the start of that repeat.

## What the metrics mean

- Reserved-pixel errors are within-track reprojection checks at 960 × 540, not absolute 3D errors.
- View coverage is coverage of a finite directional sampling plan, not coverage of all scene surfaces.
- Landmark IDs may correspond to repeated reconstructions of the same physical feature.
- Estimated distance and position use the fused trajectory, not surveyed truth.
- Revisit discrepancies concern candidate RGB associations; repeated texture can create false matches.
- One adaptive run per condition does not establish statistical replication or isolate the contribution of each sensor.

No forbidden sensor or simulator input is introduced by the repository organization. Historical code and evidence remain unchanged.
