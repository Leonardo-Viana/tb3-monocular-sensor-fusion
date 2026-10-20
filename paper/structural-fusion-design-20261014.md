# Learned features, segmentation and planar-LiDAR constraints for monocular mapping

**Engineering addendum — 14 October 2026**

Prepared on 14 October; publication resumed on 20 October 2026. The XFeat, LIMAP, YOLO26 and SAM 3 project pages were rechecked on 20 October. This limited refresh does not constitute a new audit of every dependency or reference.

This note follows Leonardo Viana's proposal to use practical computer-vision models as feature providers and combine their observations with lines and corners extracted from the TurtleBot3's planar LiDAR. It extends the [literature review](literature-review-20261014.md), whose initial selection did not systematically inspect the YOLO/SAM/Roboflow/OpenCV ecosystems. The proposal is technically meaningful, but feature association, structural assumptions and measurement provenance must be explicit.

This is a source-backed design assessment. No package was installed, model benchmarked, robot moved or new map reconstructed for this addendum. Compatibility and resource judgments are proposed priorities, not measured performance on the Dell.

## 1. Practical tools and their actual roles

| Tool or family | Verified capability and source | Proposed role and boundary |
| --- | --- | --- |
| Ultralytics YOLO26 / YOLO11 | Current [YOLO26 documentation](https://docs.ultralytics.com/models/yolo26) describes detection, segmentation and pose tasks; [OpenVINO export](https://docs.ultralytics.com/integrations/openvino) supports Intel deployment. | Segment potentially useful regions and help reject moving objects. The desired structural classes must exist in the selected model or be trained. A box center is not a stable physical landmark; a class label is not object identity. |
| SAM 3 / SAM 3.1 | The [official repository](https://github.com/facebookresearch/sam3) documents text/visual prompts, image/video segmentation and tracking; the March 2026 update adds joint multi-object tracking. The standard installation specifies a CUDA-compatible GPU. | Produce region/instance hypotheses and masks. Temporal mask identity is useful context but does not guarantee pixel-level 3D correspondence. The standard setup does not match the Dell's integrated graphics. |
| MobileSAM / FastSAM | Author repositories for [MobileSAM](https://github.com/ChaoningZhang/MobileSAM) and [FastSAM](https://github.com/CASIA-LMC-Lab/FastSAM) provide lighter segmentation alternatives and CPU execution paths/examples. | Candidate segmentation on selected keyframes. Full automatic-mask generation may cost much more than one prompted mask; profile the exact mode. No real-time claim is made here. |
| Roboflow | [Workflows](https://docs.roboflow.com/workflows/blocks/blocks) compose inference/processing; [Supervision](https://supervision.roboflow.com/latest/) handles detections, masks, keypoints, tracking and evaluation, including integrations with RF-DETR and other models. | Useful application infrastructure and annotation/evaluation tooling. Roboflow is not a single SLAM model. Object-track IDs and detector mAP do not measure geometric correspondence accuracy. No data were uploaded to a cloud inference service. |
| OpenCV | The [official release listing](https://opencv.org/releases/) includes 4.14.0 and 5.0.0. Current [3D geometry documentation](https://docs.opencv.org/5.0/main_modules/d_projection.html) covers pose/projection/triangulation; the [line detector](https://docs.opencv.org/4.12.0/db/d73/classcv_1_1LineSegmentDetector.html) is already available in the 4.x family. | Calibration, distortion correction, optical flow, point/line extraction, robust fitting and triangulation remain useful building blocks. A new major version is not itself a SLAM algorithm. Preserve the working ROS environment and evaluate upgrades separately. |
| XFeat + optional LighterGlue | [VeRLab's implementation](https://github.com/verlab/accelerated_features) provides learned local keypoints/descriptors, CPU inference and an optional smaller learned matcher. | Particularly relevant first learned frontend for the Dell. Start with bounded keypoint counts and simple matching; profile LighterGlue separately. Published laptop-CPU results do not establish throughput with Gazebo and recording active. |
| SuperPoint + LightGlue; GlueStick | [LightGlue](https://github.com/cvg/LightGlue) matches local descriptors; [GlueStick](https://github.com/cvg/GlueStick) jointly matches points and lines. | Directly address correspondence, unlike a generic object detector. Include as comparison options with independent resource accounting. |
| LIMAP | [The author toolbox](https://github.com/cvg/limap) supports structured 3D mapping/SfM with points, lines, planes and related primitives; its current documentation includes 2026 developments and COLMAP interoperability. | A strong source of line reconstruction and optimization methods. It is not automatically a real-time ROS 2 node or a ready-made 2D-LiDAR adapter. |
| Community YOLO3D variants | [ruhyadi/YOLO3D](https://github.com/ruhyadi/YOLO3D) combines object detection with 3D box estimation; [niconielsen32/YOLO-3D](https://github.com/niconielsen32/YOLO-3D) combines YOLO and depth estimation. | Inspect the source of depth, class/dimension priors and supported scenes. A predicted 3D bounding box does not establish a reconstructed scene surface. Projects sharing the YOLO3D name can have different sensors and assumptions. |
| SAM 3D Objects | The [official project](https://github.com/facebookresearch/sam-3d-objects) generates object shape/pose/texture from masked image objects; its [setup](https://github.com/facebookresearch/sam-3d-objects/blob/main/doc/setup.md) specifies an NVIDIA GPU with at least 32 GB VRAM. | Useful learned object priors in a separate future comparison. Not a practical default online configuration for this Dell, and generated hidden surfaces are not observations. |

The distinction is functional: **geometric features** support repeatable correspondence; **semantic outputs** help select and associate regions; **learned 3D predictions** supply priors. These functions can cooperate without treating them as interchangeable measurements.

## 2. What one planar scan can and cannot determine

With calibrated intrinsics and distortion removed, a visual observation defines a ray. In a common coordinate frame,

$$P(\lambda)=C+\lambda r,$$

where $C$ is the camera center and $r=R K^{-1}\tilde u$ is its viewing direction. The scalar $\lambda$ is unknown. Normalizing $r$ changes the interpretation of $\lambda$, not the reconstructed point.

A scan provides metric points on its scanning plane. It does not directly give the distance of an arbitrary image feature above or below that plane. Merely seeing a feature at approximately the same bearing is insufficient.

There are three useful cases:

1. **Direct common surface point:** project a calibrated LiDAR return into the camera. A verified same-surface association can provide a sparse metric depth observation, accounting for occlusions and the different viewpoints. Geometrically projecting a return does not prove that a nearby detected corner is the same physical point.
2. **Shared structure:** infer a wall or vertical edge supported by a scan segment and associate visual evidence with that structure. This adds an explicit structural hypothesis beyond the scan itself.
3. **No justified cross-sensor association:** retain ordinary multiview triangulation using motion supported by wheel/IMU/LiDAR localization. The camera feature still benefits from better pose estimation without receiving a direct range factor.

Angles measured between image segments are not directly comparable to angles between LiDAR segments: perspective projection and relative sensor pose must be modeled.

## 3. A concrete wall example

For a horizontal scan in a gravity-aligned frame, suppose a robustly fitted line is

$$a x+b y+c=0.$$

If evidence supports a **vertical planar wall**, it defines a candidate 3D plane with $n=(a,b,0)^T$ and equation $n^T P+c=0$. In the general tilted-scan case, construct the vertical candidate plane from the fitted 3D line and the estimated gravity direction rather than simply setting its normal's z component to zero.

If a visual feature is associated with this same wall, ray–plane intersection gives

$$\lambda=-\frac{n^T C+c}{n^T r},\qquad P=C+\lambda r.$$

This is a derived geometric constraint, not a result reported by one of the model repositories. It explains why the proposal works: the RGB pixel supplies bearing/elevation while the measured wall location and structural hypothesis supply metric range. The initial estimate can subsequently be refined using multiple views and uncertain sensor factors.

For example, with camera center $(0,0,0.3)$ m, ray direction $(1,0.4,0.2)$ and an associated wall at $x=3$ m, the intersection is $(3,1.2,0.9)$ m. These are illustrative values, not experimental measurements.

A real wall junction can similarly provide a vertical edge from a LiDAR corner. Its projection supplies a line constraint for the camera; a matched visual point on that edge can constrain height. However, the endpoint of a scan segment may be an occlusion or a range discontinuity rather than a physical corner. An intersection of extended lines is not automatically a visible wall junction.

The hypothesis becomes unstable when the ray is nearly parallel to the plane. Incorrect plane assignment, slanted/curved surfaces, doors that move, glass, objects in front of a wall, inaccurate extrinsics, timing errors and accelerometer contamination of gravity estimation can all produce confident-looking but wrong depths. Use finite segment support, visibility, positive depth, residual tests, multiview agreement and propagated uncertainty. Semantic confidence alone is not a calibrated geometric covariance.

## 4. Proposed estimator arrangement

- **LiDAR frontend:** fit finite segments with robust residuals, uncertainty and persistent associations; propose junctions only when supported. Do not impose right angles on every scene.
- **Visual frontend:** keep the existing geometric tracker as a baseline; compare XFeat matching and OpenCV line extraction. Use segmentation to support region association and reject independently moving features, with residual-based motion checks.
- **Cross-sensor association:** calibrated projection, visibility and temporal consistency first; semantic labels and structural layout provide additional evidence. Repeated doors or identical pillars require multiple hypotheses or rejection rather than forced matches.
- **Estimator/backend:** combine visual reprojection, optional point-to-plane/line constraints, raw IMU and wheel factors, and selected planar LiDAR factors with appropriate covariance. Keep local odometry continuous and handle global corrections coherently. Avoid treating a pose prior and the raw measurements that generated it as independent information.
- **Map:** store points/lines/planes with provenance and uncertainty. Render a wall surface only where evidence and the declared model support it; do not extrude every scan segment to an arbitrary height.

These are proposed modules, not a claim that they are implemented. An object class does not imply motion, and a stationary object can later move; semantics should adjust hypotheses rather than determine the entire geometric model.

## 5. Experimental boundary and first comparison

The existing baseline obtains 3D landmarks by RGB triangulation, using the other sensors to support camera pose. Adding ray–wall or direct range factors would create **LiDAR-constrained monocular reconstruction**: still one RGB camera, no RGB-D and no compass, but some landmark depth would now be constrained directly by LiDAR and structural assumptions. This must be a separately labeled arm. It should not be described as depth inferred exclusively from RGB. Learned feature descriptors alone do not make the reconstructed depth a learned-depth prediction.

Proposed incremental comparison on preserved recordings, followed by fresh trials:

| Arm | Change from the prior arm | Main question |
| --- | --- | --- |
| A | Preserved existing tracker and reconstruction | Reference behavior and resource cost |
| B | Learned local points and/or image lines; no direct LiDAR-to-landmark depth | Do better correspondences reduce outliers and improve revisits? |
| C | Verified LiDAR structural factors | Do metric structural constraints improve geometry without excessive false associations? |
| D | Segmentation-based association/motion support | Does semantics add benefit beyond geometric matching? |

Initially run one candidate at a time in an isolated environment. Bound image resolution, feature count and asynchronous queues. Run semantic inference on selected keyframes; track between them. OpenVINO/ONNX are candidates for YOLO deployment, with CPU and integrated-GPU variants measured separately because Gazebo also uses graphics resources. No throughput or memory fit is guaranteed beforehand.

Score correspondence precision, residual tails, trajectory and submap consistency, geometric accuracy/completeness, failures under wrong structural assumptions, end-to-end latency, memory and per-topic loss. Include slanted walls, visually repeated structures and foreground occluders so that the vertical-wall assumption is genuinely tested. Preserve unknown space when associations fail.

## 6. Nearby work and source limitations

[Semantic Fusion Algorithm of 2D LiDAR and Camera Based on Contour and Inverse Projection](https://doi.org/10.3390/s25082526), Sensors 2025, is closely related to contour-based cross-sensor correspondence and semantic labeling. The indexed abstract/method description was accessible; direct full-text access was blocked during this review. Its semantic fusion result must not be cited as proof of complete 3D SLAM.

[Direct 3D mapping with a 2D LiDAR using sparse reference maps](https://doi.org/10.1016/j.ophoto.2025.100109), ISPRS Open Journal of Photogrammetry and Remote Sensing, volume 19 (2026), is a useful counterexample to broad claims about “2D LiDAR becoming 3D”: the indexed abstract explicitly uses a 6-DoF trajectory and external sparse 3D reference maps. Those inputs are different from our fixed-height planar robot experiment.

The engineering shortlist is therefore **OpenCV + XFeat first, robust planar-LiDAR association next, and lightweight segmentation only where it improves a measured failure mode**. LIMAP and GlueStick inform the structured-geometry design; SAM 3D and other generative 3D tools belong to a clearly separated future resource/prior comparison. All repository and version statements are a snapshot as of 14 October 2026, not a completed dependency audit.
