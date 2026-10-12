# Short Gazebo integration validation — 12 October 2026

Purpose: validate the revised shutdown path with live simulated RGB, planar LiDAR, raw IMU and wheel odometry on the original Dell Vostro 3710.

This is an engineering integration test, not an additional full-map replication, accuracy benchmark or sensor ablation. Historical recordings remain unchanged.

## Frozen execution choices

- One fresh run in /home/lpviana/tb3_integration_20261012/repeat_01.
- Read-only reuse of the archived textured scene; Gazebo seed 23.
- Revised runtime from the 12 October shutdown follow-up.
- The supervisor now accepts TB3_MISSION_SECONDS; default remains 2400. Set 180 for this test.
- ROS domain 76, localhost-only discovery, Gazebo partition tb3_integration_20261012.
- Use the existing campaign monitor with an outer mission deadline of 1200 seconds.
- Navigation stops at the short budget or an earlier declared controller stop; final fitting and saving can take additional time.
- The same reconstruction, planning, correspondence thresholds and sensor policy are used.
- Gazebo server plus one RViz process; viewers are closed by the supervisor.
- Existing resource gates apply: at least 1700 MiB available and 9 GiB disk before starting; stop below 450 MiB available or 6 GiB free disk during monitoring.
- Record five-second resource samples, all process outcomes and raw sensor topics.

## Required observations for a pass

1. All required simulated sensor streams are received; autonomous motion is observed.
2. The controller finishes, sends zero velocity and produces terminal audit records.
3. Final keyframes/tracks parse, save_complete.json is present, and stable reconstruction/validation outputs exist and agree in count.
4. No unexpected process exit, traceback, duplicated ROS shutdown or cleanup escalation occurs.
5. The simulator is paused before outer cleanup and all owned mission processes are gone afterward.
6. Retain low-resource aborts, recording losses, unexpected stops and failed checks. Do not redefine a failed run as successful.

The expected short-run stopping reason is resource_budget_reached_incomplete or time_budget_reached_incomplete, rather than complete coverage. Report the actual reason and map scope. Passing does not resolve the previously observed approximately 32 cm submap discrepancy, or establish publication readiness.
