# Reproduction levels

## 1. Inspect and verify the distributed evidence

From the repository root:

```bash
python3 tools/restore_experiments.py
python3 tools/verify_evidence.py
python3 tools/summarize_experiments.py --check
```

Both tools use only the Python standard library. The first compares SHA-256 hashes with the migration manifest and checks that each mission's saved PLY and JSON point counts match its validation report. It also checks that the working runtime equals the archived guarded version. This verifies the migration, not scientific accuracy. Future deliberate runtime edits must update that comparison policy without changing archived hashes.

To regenerate the compact tables from saved metrics:

```bash
python3 tools/summarize_experiments.py
```

## 2. Regenerate plots and numerical reconstruction

Analysis uses NumPy, SciPy, OpenCV, Matplotlib and Pillow; see `environments/analysis-requirements.txt`. Recorded baseline versions are provided separately. They are observed versions, not a complete dependency lock for every run.

The original plotting scripts live next to their experiment data and may overwrite the corresponding saved figures. Work on a copy when regenerating historical outputs. The pilot archive includes numerical tracks/keyframes but only selected JPEG images. The autonomous archives omit full tracks, keyframes and JPEG inputs, so they do not support full reconstruction or fresh revisit matching by themselves.

Full inputs remain under these original Dell directories:

- `/home/lpviana/tb3_texture_experiment_20260929`
- `/home/lpviana/tb3_autonomous_texture_20261001`
- `/home/lpviana/tb3_guarded_trial_20261002`

Restore them into a separate working copy before rerunning `stable_cached.py` or `audit_revisits.py`. Preserve the original outputs and predeclare changes. `evaluate_trial.py` is historical and references the original baseline path; use the repository summary tool for portable comparisons.

## 3. Prepare a new simulated mission

```bash
python3 tools/prepare_run.py my-new-run
```

This copies the current runtime and scene-authoring inputs into `runs/my-new-run/`, refuses an existing destination, and records source hashes. It does not start Gazebo or ROS. The `runs/` directory is ignored by Git; archive selected evidence under a new experiment ID after completion.

The tested platform used Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. Pilot records identify `turtlebot3_gazebo` 2.3.7, `ros_gz_bridge` 1.0.24 and `slam_toolbox` 2.8.5. The exact original binary packages remain external dependencies. This repository is a preserved Python-script workflow, not an installed ROS/colcon package or a containerized distribution.

In a prepared run, regenerate scene asset paths with:

```bash
python3 runs/my-new-run/scene/build_worlds.py
```

The world builder uses its `source/` copies for authoring only and creates installation-specific absolute mesh/texture paths. The simulator still needs the upstream TurtleBot3 collision and camera model resources. `GZ_SIM_RESOURCE_PATH` must include the corresponding installed models directory.

The preserved guarded supervisor expects an already paused `texture_trial` world, `GZ_PARTITION=tb3_texture_pilot_20260929`, ROS domain 74, and a sourced Jazzy environment. Its file docstring mentions the older domain 73; the executable environment dictionary and recorded manifest specify 74. This stale historical comment has been preserved, not silently rewritten.

The launcher controls simulator pause, starts sensor bridges and a velocity controller, and refuses an existing completed output directory. Create a fresh protocol and verify isolation before using it; never run two velocity controllers in the same simulated domain. The old pilot controller has a documented command-flood defect and is retained for audit only. Use the guarded runtime for future development.

No Gazebo, ROS, physical robot or autonomous motion was started during this repository migration. A fresh end-to-end run from this layout has not been validated.
