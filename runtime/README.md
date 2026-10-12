# Runtime

The revised shutdown path subsequently passed a short live Gazebo integration
on 12 October; see the [integration report](../paper/integration-report-20261012.md).
`TB3_MISSION_SECONDS` permits a shorter navigation budget (1–2400 seconds;
default 2400), followed by final reconstruction and cleanup.

Part of the TurtleBot3 monocular sensor-fusion research project. See the [main README](../README.md) for scope, usage and limitations.

As of 12 October 2026, this directory includes cooperative shutdown fixes. It is
no longer byte-identical to the guarded-run archive. Historical archives and
their metrics are unchanged. Estimator thresholds and triangulation are unchanged.

The mission supervisor owns sensor shutdown. Signals request a cooperative
stop; the vision node completes its final save before destroying the ROS context.
Each JSON/PLY file is replaced atomically, but the set of files is not a
crash-atomic transaction. The supervisor now closes its viewer and RViz as well.

The launcher honors explicit ROS_DOMAIN_ID and GZ_PARTITION environment values;
its historical defaults remain 74 and tb3_texture_pilot_20260929. Always use an
isolated simulation environment and a fresh directory prepared with
tools/prepare_run.py. Never execute inside an archived experiment.

See the [follow-up report](../paper/followup-report-20261012.md) for isolated
regression evidence and the integration report above for the subsequent live
test. The full three-run mapping campaign has not been repeated with this revision.
