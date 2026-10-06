# Third-party provenance and release status

The pilot's `source/` includes copies of TurtleBot3/Gazebo scene and robot assets from the installed `turtlebot3_gazebo` package. Its preserved `package.xml` identifies the upstream package and license declaration. Original names, metadata and source copies are retained. The world builder modifies texture coordinates/materials and generates procedural textures; it does not replace the underlying asset ownership.

Upstream project: https://github.com/ROBOTIS-GIT/turtlebot3_simulations

ROS 2, Gazebo, SLAM Toolbox, NumPy, SciPy, OpenCV, Matplotlib, Pillow and other dependencies retain their own licenses. Dependency binaries are not bundled.

This private development collection does not assign a new blanket license to upstream assets or to the project. Before public release, select the original-code license and complete the upstream license/notice inventory. Paper authorship and citation metadata are also pending.
