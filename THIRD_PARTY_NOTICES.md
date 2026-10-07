# Third-party notices and provenance

License inventory checked on 7 October 2026. Scope: the distributed repository, three experiment ZIP archives and preserved preparation Git bundle. See [LICENSING.md](LICENSING.md) for original project contributions. Frozen numerical evidence is unchanged.

## ROBOTIS TurtleBot3 simulation assets — Apache-2.0

Upstream: https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/2.3.7
Upstream tree: `9be186fb03d84ed4f293e5c0db71d8c05bbc91f3`.
The complete upstream license is retained in [LICENSE-TURTLEBOT3.txt](LICENSE-TURTLEBOT3.txt). No upstream NOTICE file was present in that release tree.

The following files are in `tb3_texture_experiment.zip`, under `source/` (also restored under `experiments/2026-09-30-texture-pilot/source/`). All six are byte-identical to upstream version 2.3.7:

| Archived name | Upstream path under turtlebot3_gazebo/ |
| --- | --- |
| package.xml | package.xml |
| original.world | worlds/turtlebot3_world.world |
| robot.sdf | models/turtlebot3_waffle/model.sdf |
| world_model.sdf | models/turtlebot3_world/model.sdf |
| wall.dae | models/turtlebot3_world/meshes/wall.dae |
| hexagon.dae | models/turtlebot3_world/meshes/hexagon.dae |

Package authors named upstream: Darby Lim, Pyo, Ryan Shim and Hyungyu Kim. Waffle and World model metadata identify Taehun Lim (Darby). Original package metadata remains in the archive.

**Modifications:** `worlds/meshes/wall.dae` and `worlds/meshes/hexagon.dae` add UV coordinates without changing original numerical vertex arrays. `worlds/uniform.sdf` and `worlds/textured.sdf` assemble and adapt the upstream models, rename the world/robot, set the initial robot pose, add procedural materials and use a generated floor visual. The original scene geometry remains subject to the upstream license; project-authored adaptations are also Apache-2.0. The generator is `build_worlds.py`; seed and changes are recorded in `scene_manifest.json`. The original procedural texture images and research results use CC-BY-4.0 as set out in LICENSING.md.

## OpenRobotics Fuel models — CC0-1.0

The pilot `source/sun.sdf` and `source/ground.sdf` originate from Fuel, not from the TurtleBot3 package. Model metadata names Nate Koenig as author. Both archived files match the corresponding installed Fuel `model.sdf` byte for byte.

| File | Source | Version | License |
| --- | --- | --- | --- |
| sun.sdf | https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun | 3 | CC0-1.0 |
| ground.sdf | https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground%20Plane | 5 | CC0-1.0 |

The Fuel API reports Creative Commons Zero v1.0 Universal for each model. See [LICENSE-CC0-1.0.txt](LICENSE-CC0-1.0.txt). The generated worlds include these components and adapt the floor visual as described above.

## Dependencies and excluded material

ROS 2, Gazebo, SLAM Toolbox, NumPy, SciPy, OpenCV, Matplotlib, Pillow and other separately installed dependencies retain their own licenses. Their binaries and source trees are not bundled. The presence of an import does not assign this project's license to the imported package. Upstream robot collision/visual resources referenced by URI but absent from the archives must be obtained separately under their own terms.

Trademarks and logos retain their owners' rights. References in the research notes do not confer licenses over cited works. Paper authorship, affiliations, venue and archival DOI remain separate publication decisions.

See [UPSTREAM_VERIFICATION.json](UPSTREAM_VERIFICATION.json) for file hashes and comparison evidence. Keep this file, NOTICE and license texts with extracted archives or restored historical Git trees; old private-development notes are historical, not the current licensing policy.
