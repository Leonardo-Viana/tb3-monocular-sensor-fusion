# Visual SLAM, multisensor fusion and autonomous mapping under a CPU and memory budget

**Focused literature review — 14 October 2026**

Follow-up: [practical vision models and planar-LiDAR structural fusion](structural-fusion-design-20261014.md) examines YOLO, SAM, Roboflow, OpenCV, learned local features and the assumptions needed to constrain RGB landmarks using scan lines.

This review supports the TurtleBot3 monocular sensor-fusion experiment. It covers selected foundational papers and recent work available by the review date; it is not an exhaustive systematic review or a reproduced leaderboard. Sources are papers, author project pages, official repositories and Nav2 documentation. Publication venues are distinguished from preprints. Hardware suitability below is an engineering assessment, not a measured result on our Dell.

The target remains a Dell Vostro 3710 with an i5-12400, 8 GiB of RAM and Intel UHD 730, running Gazebo, ROS 2 Jazzy and the full robot pipeline. Inputs are one RGB camera, planar LiDAR, wheel measurements and raw gyroscope/accelerometer measurements. Depth/RGB-D, compass, IMU orientation estimates and simulator truth are excluded from estimation and control. The 3D structure must be supported by monocular observations; planar LiDAR supports localization and navigation.

The accompanying [BibTeX bibliography](literature-20261014.bib) uses the citation keys shown below. This document reports a literature review and source inspection, **not a new robot execution or a successful integration of the cited systems**.

## 1. What the different research problems establish

- **Visual odometry (VO)** estimates incremental camera motion. Good local tracking alone does not establish global consistency after revisiting a location.
- **Visual-inertial odometry (VIO)** adds IMU dynamics and bias estimation. Metric scale depends on observability, calibration and motion; adding an IMU is not an unconditional cure for monocular ambiguity.
- **SLAM** maintains localization and a map, commonly with place recognition, geometrically verified loop closures and global correction. A local VIO filter can be part of SLAM without itself implementing the complete global backend.
- **Map representation** determines what is stored: landmarks, planes, occupancy, signed-distance fields, radiance fields, Gaussians or semantic scene graphs. Attractive rendering and accurate collision geometry are different objectives.
- **Navigation and active SLAM** decide actions. Reaching a goal, exploring free space and obtaining observations that make a 3D reconstruction well constrained require different success criteria.

These distinctions matter here: our sparse RGB reconstruction, planar localization and autonomous controller are coupled components, but their combination is not yet a validated globally consistent visual-inertial SLAM system.

## 2. Selected literature and relevance

### Classical and visual-inertial estimation

| Reference | Contribution relevant to this study | Fit and qualification |
| --- | --- | --- |
| [LSD-SLAM](https://cvg.cit.tum.de/research/vslam/lsdslam), ECCV 2014 (`engel2014lsd`) | Direct, semi-dense monocular SLAM; useful historical reference for photometric reconstruction. | Historical baseline; its old ROS environment makes immediate Jazzy adoption unattractive. |
| [DSO](https://cvg.cit.tum.de/research/vslam/dso), TPAMI 2018 (`engel2018dso`) | Direct sparse odometry, with photometric calibration central to its formulation. | A VO reference, not a complete substitute for global loop closure and navigation. |
| [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), T-RO 2018 (`qin2018vins`) | Sliding-window visual-inertial optimization, initialization and relocalization, with a pose-graph facility. | Useful optimization baseline; porting and planar-motion behavior must be assessed. |
| [OpenVINS](https://docs.openvins.com/), ICRA 2020 (`geneva2020openvins`) | Filter-based visual-inertial estimation, calibration and bounded feature processing; official ROS 2 support. | Strong CPU-oriented baseline candidate. Its optional loosely coupled loop-closure output is not equivalent to feeding global corrections into the local filter. |
| [Basalt](https://cvg.cit.tum.de/research/vslam/basalt), RA-L/ICRA 2020 (`usenko2020basalt`) | Recovers nonlinear factors from local VIO information for global mapping. | Especially useful for designing the interface between bounded local estimation and global consistency. Monocular deployment details were not validated here. |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), T-RO 2021 (`campos2021orbslam3`) | Visual and visual-inertial SLAM, including monocular operation and multiple maps. | Mature comparison target. The official ROS examples document an older environment; native Jazzy integration and whole-system memory use are not established by the paper. |

CPU-oriented does not mean that a complete Gazebo + SLAM + recorder + navigation workload will fit comfortably in 8 GiB. Compilation, wrappers, vocabulary loading, image resolution and mapping threads all contribute to the actual budget.

### Fusion for wheeled robots and weak visual structure

| Reference | Finding or method | Implication |
| --- | --- | --- |
| [VINS on Wheels](https://mars.cs.umn.edu/papers/KejianWu_VINSonWheels.pdf), ICRA 2017 (`wu2017wheels`) | Analyzes visual-inertial observability on a wheeled platform, including scale degeneracy under straight-line and circular-arc motion. | Wheel measurements are directly relevant to metric estimation. This does not imply that every planar trajectory is equally degenerate. |
| [Visual-Inertial-Wheel Odometry with Online Calibration](https://pgeneva.com/downloads/papers/Lee2020IROS.pdf), IROS 2020 (`lee2020viwo`) | Joint camera/IMU/wheel estimation and wheel spatial, temporal and intrinsic calibration; analyzes degenerate motions. | One of the closest methodological references to our sensors and motion. Calibration and excitation must be explicit experimental variables. |
| [Monocular VIO with Planar Regularities](https://github.com/rpng/ov_plane), ICRA 2023 (`chen2023planes`) | Uses visual point/plane structure without requiring an additional depth sensor. | Promising indoor extension. The released package has an older ROS build; planes still require image evidence and assumptions must be stated. |
| [MINS](https://github.com/rpng/MINS), JFR 2025 (`lee2025mins`) | Filter-based asynchronous multisensor fusion and online calibration, including camera, IMU, wheels and LiDAR. The repository now contains ROS 2 instructions and a Jazzy Dockerfile. | Strong integration candidate. **A direct adapter for TurtleBot3 planar LaserScan was not verified.** Begin by evaluating camera–IMU–wheel support, then assess planar LiDAR constraints. |
| [PL-VIWO](https://arxiv.org/abs/2503.00551), 2025 preprint (`zhang2025plviwo`) | Point/line visual-inertial-wheel estimation and motion-consistency checks. | A closely related line-feature approach, with an author-linked code repository. |
| [PL-VIWO2](https://arxiv.org/abs/2509.21563), September 2025 preprint, v1 (`zhang2025plviwo2`) | Point/line processing, planar-constrained wheel preintegration and consistency checks using IMU and wheel measurements. | Highly relevant design ideas. The reviewed version promises code on acceptance; this review does not establish a released implementation or a later publication status. |

[On-manifold IMU preintegration](https://arxiv.org/abs/1512.02363) (`forster2017preintegration`) provides a standard way to summarize inertial measurements between states while retaining bias dependence in optimization. It is a possible replacement for parts of our current pose-prior approach, not a feature already implemented in our reconstructor.

[FAST-LIVO2](https://arxiv.org/abs/2408.14035) (`zheng2024fastlivo2`) and [R3LIVE](https://arxiv.org/abs/2109.07982) (`lin2022r3live`) are valuable fusion references, but their geometric mapping uses 3D LiDAR. R3LIVE explicitly separates LiDAR-derived geometry from visual coloring. They cannot be presented as drop-in solutions satisfying our requirement that scene height comes from monocular vision and that only a planar LiDAR is available.

### Learned VO, foundation models and dense mapping

| Reference | Direction represented | Qualification for our experiment |
| --- | --- | --- |
| [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM), NeurIPS 2021 (`teed2021droid`) | Learned matching and recurrent dense bundle adjustment. | A major learned baseline, but the official GPU requirements are unlike our integrated-graphics machine. No Dell failure was measured in this review. |
| [DPVO](https://github.com/princeton-vl/DPVO), NeurIPS 2023 (`teed2023dpvo`), and [DPV-SLAM](https://www.ecva.net/papers/eccv_2024/papers_ECCV/html/272_ECCV_2024_paper.php), ECCV 2024 (`lipson2024dpvslam`) | Learned sparse patches; the SLAM extension adds loop-closure mechanisms. | Lower GPU demand relative to some dense learned systems does not imply a CPU implementation suitable for UHD 730. VO and SLAM results should be distinguished. |
| [MASt3R-SLAM](https://edexheim.github.io/mast3r-slam/), CVPR 2025 (`murai2025mast3r`) | Dense monocular SLAM using a pretrained two-view geometry model and global optimization. | Uses RGB input but also learned geometry priors. The official implementation and reported setup use CUDA/GPU computing; published throughput is not transferable to this Dell. |
| [VGGT-SLAM 2.0](https://www.roboticsproceedings.org/rss22/p051.html), RSS 2026 (`maggio2026vggt`) | A revised graph for aligning foundation-model submaps and attention-assisted loop verification. | Important current reference for submap consistency. Reported onboard operation uses Jetson Thor, not a comparable CPU-only budget. The [repository](https://github.com/MIT-SPARK/VGGT-SLAM) contains the 2.0 release; the proceedings abstract still uses future-tense code wording. |
| [Gaussian Splatting SLAM / MonoGS](https://openaccess.thecvf.com/content/CVPR2024/html/Matsuki_Gaussian_Splatting_SLAM_CVPR_2024_paper.html), CVPR 2024 (`matsuki2024monogs`) | A Gaussian map representation, including a monocular RGB configuration. | Useful dense-map comparison; rendering quality does not itself establish accurate surfaces, known free space or map completeness. |

There are two different meanings of “RGB-only 3D”: multiview triangulation from the current observations, and reconstruction assisted by geometry learned from external training data. Both can use only an RGB sensor at deployment. They should be separate experimental arms, with training data, pretrained weights and learned priors disclosed. A predicted surface is not automatically a measured surface. We should retain the current triangulation arm as the interpretable baseline.

Sparse points, lines and planes offer a more plausible first extension for this hardware than online dense neural reconstruction. This is a resource-based hypothesis requiring profiling, not a claim that classical methods always outperform learned methods. Untextured or occluded surfaces may remain unknown even after sensor fusion; a planar range measurement does not observe wall height.

### Navigation, active perception and structured maps

[Hydra](https://www.roboticsproceedings.org/rss18/p050.pdf), RSS 2022 (`hughes2022hydra`), organizes geometry and semantics into hierarchical scene graphs. It illustrates how a map can support higher-level reasoning, but adding semantic hierarchy would not repair an incorrect geometric alignment in our existing submaps.

[ViNT](https://general-navigation-models.github.io/vint/), CoRL 2023 (`shah2023vint`), and [NoMaD](https://general-navigation-models.github.io/nomad/), ICRA 2024 (`sridhar2024nomad`), demonstrate learned visual navigation and exploration. They are relevant alternative policies, not evidence that a metrically accurate, complete 3D map is produced. Their navigation results do not substitute for our mapping metrics or a resource measurement on the Dell.

[Navigation World Models](https://www.amirbar.net/nwm/), CVPR 2025 (`bar2025nwm`), predicts action-conditioned future observations to evaluate trajectories. This represents a significant direction in learned planning. The authors also document failures in unfamiliar environments, and their official planning example uses multiple GPUs. It is a research comparison for future work, not an immediate lightweight replacement for our controller; generated future images must not be treated as observed map geometry.

The [Active SLAM review](https://www.mdpi.com/1424-8220/23/19/8097), Sensors 2023 (`ahmed2023active`), organizes action selection around information and uncertainty. For our experiment, the useful translation is to choose views that improve geometric constraints: observable features, sufficient translational baseline, revisits with overlap, and feasible travel cost. Pure rotation can help recognize a place but does not by itself create triangulation baseline.

[Nav2 Jazzy documentation](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/) (`nav2jazzy2026`) provides the practical planning/controller framework. A Smac 2D planner with a profiled controller is a reasonable candidate; Regulated Pure Pursuit and MPPI have different costs and behavior. A higher-level view-selection policy can propose goals while the navigation layer retains obstacle checks. A planar scan cannot certify the absence of obstacles outside its scan plane; unknown 3D space must not silently become free space.

## 3. What changes in our interpretation of the current system

The [13 October report](tracking-report-20261013.md) and implementation inspection establish a planar wheel/gyro estimator, accelerometer-based checks and tilt handling, planar LiDAR localization, and RGB triangulation with metric pose priors. The stable reconstruction initializes with yaw-only poses and fixed camera height before regularized local fitting. It does **not** currently perform joint raw-IMU preintegration and visual optimization with a persistent cross-revisit landmark graph.

The global placement code uses nearby planar pose-graph nodes to estimate a local rigid XY correction. This is an engineering approximation. It is a plausible contributor to the retained approximately 31.85 cm submap discrepancy, but the discrepancy's cause has not been established. The recently added bounded-ray gate addresses temporal correspondence failures; it does not establish correct global submap placement.

Consequently, the next architectural target should be a coherent local/global estimator interface: bounded local visual–inertial–wheel estimation, explicit submap anchoring, and robust, verified constraints across revisits. Reusing fused odometry as an independent factor while also adding the same raw wheel/IMU measurements can double-count information. Covariance and marginalization choices need to be specified rather than treating every available pose estimate as independent evidence.

This is a proposed design direction, not a claim that swapping a package will automatically remove drift or that a particular graph formulation has already been validated.

## 4. Prioritized experimental programme

| Priority | Proposed work | Evidence needed before the next claim |
| --- | --- | --- |
| 1 | Isolate acquisition losses by topic and workload phase; check timestamps, camera/IMU/wheel extrinsics and time offsets. | Recorded rates, gaps, age and queue/drop diagnostics; reproducible calibration. The earlier 344 lost messages have no isolated cause yet. |
| 2 | Profile OpenVINS as a VIO reference and MINS camera–IMU–wheel as a fusion candidate on preserved bags, with pinned versions. | Build/port outcome, pose accuracy where truth is available, tracking failures, CPU, peak memory, latency and replay rate. Do not install every candidate at once. |
| 3 | Replace approximate global submap placement with a specified graph/anchor model and independently verified revisits. | Improvement on fresh trajectories and independent correspondences, not only fitting the eight already inspected historical pairs. |
| 4 | Test line and planar constraints as a separate extension inspired by PL-VIWO and ov_plane. | Low-texture versus textured scenes; effect on geometry, false associations and compute cost. Report any remaining assumption-dependent surfaces. |
| 5 | Add observation-aware exploration using parallax, overlap and revisit utility. | Compare against the existing view schedule/frontier policy under the same time/resource budget; report both map quality and mission completion. |
| 6 | Consider a learned RGB geometry baseline only as a separately budgeted experiment. | State training provenance and compute location. Offline processing on another GPU cannot be counted as online performance on the Dell. |

No threshold, speedup, accuracy gain or memory fit in this programme is promised in advance. Pure visual reconstruction, VIO, VIWO and LiDAR-supported variants require clear ablations. Removing a sensor can also change initialization or observability; such failures are results, not reasons to silently omit a comparison.

## 5. Evaluation and the publication claim

[Delmerico and Scaramuzza's monocular VIO benchmark](https://rpg.ifi.uzh.ch/docs/ICRA18_Delmerico.pdf), ICRA 2018 (`delmerico2018benchmark`), already compares accuracy and computational demands across hardware. More directly, **[SLAMSqueezeBench](https://arxiv.org/abs/2609.19533)** (`hefny2026squeeze`), submitted in September 2026, proposes constrained compute/memory evaluation and finite camera buffers with frame drops. Its indexed abstract describes nine systems. **Only indexed abstract/first-page material was accessible in this review; full-text retrieval failed.** No detailed result, peer-reviewed acceptance or released implementation is inferred.

Therefore, “running SLAM on a weak computer” is not a sufficient novelty claim. A more defensible research question is whether a transparent monocular geometric pipeline, aided by wheel/IMU/planar-LiDAR constraints and observation-aware exploration, can improve the tradeoff between global map consistency, acquisition integrity and coverage under a fixed whole-system budget. This remains a hypothesis, not an established contribution or publication-readiness statement.

Recommended reporting:

1. **Trajectory:** ATE and RPE with declared time association, alignment and units. For a metric fusion estimator, use a primary metric that does not rescale away scale error; a supplementary Sim(3)-aligned monocular score must be labeled separately.
2. **Geometry:** accuracy and completeness against independent reference geometry, when available; submap discrepancy, duplicate structures and false surfaces. Simulator truth may be used by an isolated evaluator, never fed to estimation, planning or tuning on the held-out test set.
3. **Visual evidence:** reprojection errors, accepted/rejected tracks, triangulation conditioning and repeatability. Held-out pixels within already inspected tracks are not independent scene-level validation. Rendering/color quality should be reported separately.
4. **Navigation:** collisions, reached/failed goals, recoveries, loop closures, return-to-start error, elapsed wall and simulation time, termination reason and unfinished coverage. View coverage and geometric surface coverage are different quantities.
5. **Resources:** total pipeline and per-process CPU, peak RAM/PSS, swap, disk growth, I/O, image age, per-topic drops, and latency distributions including tail values. Simulation real-time factor is not camera-processing latency. Sampled memory maxima must be described as sampled, not continuous peaks.
6. **Experimental design:** fixed-bag comparisons for estimator attribution, then separate closed-loop runs because the policy changes the observations. Predeclare budgets, freeze parameters before fresh trials, repeat independent trajectories and worlds, and retain failures. Compare identical computing/sensing conditions rather than mixing CPU-only and external-GPU results.

The current study retains meaningful evidence of a correspondence repair and a short successful integration. It still has unresolved acquisition loss, global alignment and completeness limitations. Literature support does not turn those limitations into resolved results.

## 6. Reading order and audit scope

For implementation decisions, read `lee2020viwo`, `geneva2020openvins`, `lee2025mins`, `chen2023planes` and `zhang2025plviwo2` first; then `campos2021orbslam3`, `usenko2020basalt` and `ahmed2023active`. Use `murai2025mast3r`, `maggio2026vggt` and `matsuki2024monogs` to position the study against current dense reconstruction. Revisit `hefny2026squeeze` when its full text is retrievable before making novelty claims.

Access depth varied: selected methods/results and documentation were inspected for the main integration candidates; publication/project pages established other papers' scope and metadata. SLAMSqueezeBench remains explicitly an abstract/first-page-level item. The review does not compare leaderboard numbers across incompatible datasets or claim that all source code was audited. Repository compatibility notes are snapshots as of 14 October 2026; commit pinning is still required before implementation.
