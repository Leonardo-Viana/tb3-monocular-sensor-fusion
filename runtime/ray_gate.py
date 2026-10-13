"""Conservative two-view ray feasibility from fused camera poses.

This rejects temporal associations, never creates 3D points. The optical-depth
interval matches triangulation's working range. The 12 px tolerance is an
engineering allowance at 960 x 540, not a calibrated probabilistic bound.
"""
import numpy as np

MIN_DEPTH_M = .15
MAX_DEPTH_M = 6.
MAX_RAY_DISTANCE_PX = 12.


def ray_segment_distances(previous_uv, current_uv, previous_T, current_T, K):
    """Distance to the projection of a bounded ray, with depth valid in both views.

    Camera poses map optical-camera coordinates to the same world frame.
    A positive-depth ray projects to a straight image segment; minimize the
    distance to that entire segment analytically, with no sampled-depth gaps.
    Nonfinite or infeasible projections return infinity (fail closed).
    """
    old = np.asarray(previous_uv, float).reshape(-1, 2)
    new = np.asarray(current_uv, float).reshape(-1, 2)
    if old.shape != new.shape:
        raise ValueError('Correspondence arrays must have matching shapes')
    result = np.full(len(old), np.inf)
    matrices = [np.asarray(x, float) for x in (previous_T, current_T, K)]
    if (matrices[0].shape != (4, 4) or matrices[1].shape != (4, 4)
            or matrices[2].shape != (3, 3) or not all(np.isfinite(x).all() for x in matrices)):
        return result
    previous, current, calibration = matrices
    try:
        transform = np.linalg.inv(current) @ previous
        finite_old = np.isfinite(old).all(axis=1)
        safe_old = np.where(finite_old[:, None], old, 0.)
        rays = np.c_[safe_old, np.ones(len(old))] @ np.linalg.inv(calibration).T
    except np.linalg.LinAlgError:
        return result
    direction = rays @ transform[:3, :3].T
    translation = transform[:3, 3]
    lo = np.full(len(old), MIN_DEPTH_M)
    hi = np.full(len(old), MAX_DEPTH_M)
    dz = direction[:, 2]
    moving = abs(dz) > 1e-12
    a = np.zeros(len(old)); b = np.zeros(len(old))
    np.divide(MIN_DEPTH_M-translation[2], dz, out=a, where=moving)
    np.divide(MAX_DEPTH_M-translation[2], dz, out=b, where=moving)
    lo[moving] = np.maximum(lo[moving], np.minimum(a[moving], b[moving]))
    hi[moving] = np.minimum(hi[moving], np.maximum(a[moving], b[moving]))
    valid = np.isfinite(old).all(axis=1) & np.isfinite(new).all(axis=1)
    valid &= (lo <= hi) & (moving | ((translation[2] >= MIN_DEPTH_M) & (translation[2] <= MAX_DEPTH_M)))
    indices = np.flatnonzero(valid)
    if not len(indices):
        return result
    start = (direction[indices]*lo[indices, None]+translation) @ calibration.T
    end = (direction[indices]*hi[indices, None]+translation) @ calibration.T
    start = start[:, :2]/start[:, 2:]
    end = end[:, :2]/end[:, 2:]
    segment = end-start
    length_squared = np.sum(segment*segment, axis=1)
    fraction = np.zeros(len(indices))
    np.divide(np.sum((new[indices]-start)*segment, axis=1), length_squared,
              out=fraction, where=length_squared > 1e-18)
    nearest = start+np.clip(fraction, 0, 1)[:, None]*segment
    distance = np.linalg.norm(new[indices]-nearest, axis=1)
    result[indices] = np.where(np.isfinite(distance), distance, np.inf)
    return result


def correspondence_mask(previous_uv, current_uv, previous_T, current_T, K):
    forward = ray_segment_distances(previous_uv, current_uv, previous_T, current_T, K)
    reverse = ray_segment_distances(current_uv, previous_uv, current_T, previous_T, K)
    return np.maximum(forward, reverse) <= MAX_RAY_DISTANCE_PX
