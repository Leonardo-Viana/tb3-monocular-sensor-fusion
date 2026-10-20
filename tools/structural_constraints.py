"""Experimental geometry for the proposed C/D arms; not a ROS estimator.

Inputs must already be in one metric, gravity-aligned coordinate frame.
This module proposes ray/wall intersections. It cannot establish that a visual
feature belongs to a wall, that a wall is vertical, or that a ray is unoccluded.
The unchanged runtime does not import this module. Apache-2.0.
"""
from dataclasses import dataclass
import numpy as np


def vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f'Expected {size} finite coordinates')
    return a


@dataclass(frozen=True)
class Wall:
    """A vertical-plane HYPOTHESIS with finite horizontal support.

    p0/p1 are measured 3D scan-segment endpoints, not wall-height bounds.
    up is estimated independently from calibrated raw IMU/robot geometry.
    sigma_plane_m is a declared effective residual scale, not a posterior.
    """
    p0: np.ndarray
    p1: np.ndarray
    up: np.ndarray
    normal: np.ndarray
    tangent: np.ndarray
    width: float
    sigma_plane_m: float


def vertical_wall(p0, p1, up=(0., 0., 1.), sigma_plane_m=.03):
    p0, p1, up = vector(p0, 3), vector(p1, 3), vector(up, 3)
    if np.linalg.norm(up) < 1e-12:
        raise ValueError('Gravity direction is undefined')
    up = up / np.linalg.norm(up)
    delta = p1 - p0
    horizontal = delta - up * np.dot(delta, up)
    width = float(np.linalg.norm(horizontal))
    if width < 1e-6:
        raise ValueError('Segment has no horizontal extent')
    if not np.isfinite(sigma_plane_m) or sigma_plane_m <= 0:
        raise ValueError('Residual scale must be positive')
    tangent = horizontal / width
    normal = np.cross(tangent, up)
    return Wall(p0.copy(), p1.copy(), up, normal, tangent, width,
                float(sigma_plane_m))


def camera_ray(uv, K, T_world_optical):
    """Return center/unit ray for an UNDISTORTED pixel and optical-to-world T."""
    uv = vector(uv, 2)
    K, T = np.asarray(K, float), np.asarray(T_world_optical, float)
    if K.shape != (3, 3) or T.shape != (4, 4):
        raise ValueError('Expected K(3,3), T(4,4)')
    if not np.isfinite(K).all() or not np.isfinite(T).all():
        raise ValueError('Nonfinite calibration')
    if not np.allclose(K[2], [0, 0, 1]) or K[0, 0] <= 0 or K[1, 1] <= 0:
        raise ValueError('Invalid pinhole intrinsics')
    R = T[:3, :3]
    if (not np.allclose(T[3], [0, 0, 0, 1]) or
        not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or
        not np.isclose(np.linalg.det(R), 1., atol=1e-6)):
        raise ValueError('T must be a proper rigid transform')
    ray = R @ np.linalg.solve(K, np.r_[uv, 1.])
    return T[:3, 3].copy(), ray / np.linalg.norm(ray)


def intersect_wall(center, direction, wall, min_cos=.10,
                   min_range=.15, max_range=6., support_margin=.03):
    """Return a geometry-only candidate; never an accepted map landmark.

    Ranges are Euclidean ray distances, not optical z depth. No vertical
    extent is invented. sigma_range_plane_only omits pose/pixel/association
    uncertainty and must not be used as total measurement covariance.
    """
    center, ray = vector(center, 3), vector(direction, 3)
    if not (0 < min_cos <= 1 and 0 <= min_range < max_range and
            support_margin >= 0 and
            np.isfinite([min_cos, min_range, max_range, support_margin]).all()):
        raise ValueError('Invalid intersection bounds')
    norm = np.linalg.norm(ray)
    if norm < 1e-12:
        raise ValueError('Zero ray')
    ray = ray / norm
    denom = float(np.dot(wall.normal, ray))
    if abs(denom) < min_cos:
        return None, 'grazing_or_parallel'
    distance = float(np.dot(wall.normal, wall.p0 - center) / denom)
    if distance <= 0:
        return None, 'behind_camera'
    if not min_range <= distance <= max_range:
        return None, 'range'
    point = center + distance * ray
    along = float(np.dot(point - wall.p0, wall.tangent))
    if not -support_margin <= along <= wall.width + support_margin:
        return None, 'outside_segment_support'
    return {'xyz': point, 'range_m': distance,
            'incidence_cos': abs(denom),
            'sigma_range_plane_only_m': wall.sigma_plane_m / abs(denom),
            'status': 'candidate_requires_association_and_visibility'}, 'candidate'


def select_supported_candidate(candidates, visual_xyz, max_disagreement_m=.10):
    """Conservative association gate for an independent visual training estimate.

    Repeated/overlapping wall hypotheses are rejected when more than one fits;
    a future association module must deduplicate a physical wall explicitly.
    visual_xyz cannot have been initialized from these same plane constraints.
    This gate cannot rescue a point without visual evidence.
    """
    X = vector(visual_xyz, 3)
    if not np.isfinite(max_disagreement_m) or max_disagreement_m <= 0:
        raise ValueError('Invalid association tolerance')
    supported = [(i, c) for i, c in enumerate(candidates)
                 if c is not None and
                 np.linalg.norm(vector(c['xyz'], 3) - X) <= max_disagreement_m]
    if not supported:
        return None, 'no_visual_support'
    if len(supported) != 1:
        return None, 'ambiguous_structure'
    return supported[0], 'supported_candidate_requires_visibility'


def same_mask_support(mask, feature_uv, projected_scan_uv, min_support=3,
                      boundary_px=2):
    """One binary mask, one image: region support only, not semantic identity.

    Returns False for out-of-view samples or a feature near a mask boundary.
    Must not be used to assert a wall, remove a class as 'dynamic', or skip
    multiview/occlusion checks. Mask inference cost is outside this function.
    """
    mask = np.asarray(mask)
    if mask.ndim != 2 or mask.dtype != np.bool_:
        raise ValueError('Expected a boolean HxW mask')
    if not isinstance(boundary_px, int) or boundary_px < 0 or min_support < 1:
        raise ValueError('Invalid mask support bounds')
    feature = vector(feature_uv, 2)
    samples = np.asarray(projected_scan_uv, float)
    if samples.ndim != 2 or samples.shape[1] != 2 or not np.isfinite(samples).all():
        raise ValueError('Expected finite Nx2 scan projections')
    h, w = mask.shape
    # Check continuous coordinates before rounding, then check integer bounds.
    if not (0 <= feature[0] < w and 0 <= feature[1] < h):
        return False
    x, y = np.rint(feature).astype(int)
    b = boundary_px
    if x-b < 0 or y-b < 0 or x+b >= w or y+b >= h:
        return False
    if not mask[y-b:y+b+1, x-b:x+b+1].all():
        return False
    inside = ((samples[:, 0] >= 0) & (samples[:, 0] < w) &
              (samples[:, 1] >= 0) & (samples[:, 1] < h))
    xy = np.rint(samples[inside]).astype(int)
    xy = xy[(xy[:, 0] < w) & (xy[:, 1] < h)]
    # Repeated projections into one pixel do not provide independent support.
    xy = np.unique(xy, axis=0)
    return bool(len(xy) and mask[xy[:, 1], xy[:, 0]].sum() >= min_support)
