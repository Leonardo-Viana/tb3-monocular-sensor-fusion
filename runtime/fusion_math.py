"""Planar EKF and robust 2-D registration. No simulator ground-truth input."""
import math
import numpy as np
from scipy.spatial import cKDTree


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def rot2(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def transform2(points, pose):
    return points @ rot2(pose[2]).T + pose[:2]


class PlanarEKF:
    """State [x,y,yaw,v,w,gyro_bias]; wheel velocities and gyro updates.

    Linear acceleration is deliberately not double-integrated. Its magnitude
    gates zero-motion updates; roll/pitch are used in the camera projection.
    Covariances below are conservative engineering settings, not calibrated.
    """
    def __init__(self):
        self.x = np.zeros(6)
        self.P = np.diag([.0025, .0025, .005, .01, .01, .001])
        self.last_t = None

    def predict(self, t):
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0 or dt > .5:
            return
        yaw, v, w = self.x[2:5]
        a = yaw + .5 * w * dt
        c, s = math.cos(a), math.sin(a)
        F = np.eye(6)
        F[0, 2], F[1, 2] = -v*s*dt, v*c*dt
        F[0, 3], F[1, 3] = c*dt, s*dt
        F[0, 4], F[1, 4] = -.5*v*s*dt*dt, .5*v*c*dt*dt
        F[2, 4] = dt
        self.x[:3] += [v*c*dt, v*s*dt, w*dt]
        self.x[2] = wrap(self.x[2])
        self.P = F @ self.P @ F.T + np.diag([1e-5,1e-5,2e-5,.02,.02,1e-7])*dt

    def update(self, z, H, R, angle_index=None, gate=25):
        innovation = np.asarray(z) - H @ self.x
        if angle_index is not None:
            innovation[angle_index] = wrap(innovation[angle_index])
        S = H @ self.P @ H.T + R
        if innovation @ np.linalg.solve(S, innovation) > gate:
            return False
        K = np.linalg.solve(S, H @ self.P).T
        self.x += K @ innovation
        self.x[2] = wrap(self.x[2])
        A = np.eye(6) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        return True

    def wheel(self, v, w, gyro=None, stationary=False):
        H = np.zeros((2,6)); H[0,3] = H[1,4] = 1
        self.update([v,w], H, np.diag([.012**2,.025**2]), gate=100)
        if gyro is not None:
            G = np.zeros((1,6)); G[0,4] = G[0,5] = 1
            self.update([gyro], G, np.array([[.008**2]]), gate=100)
        if stationary:
            self.update([0,0], H, np.diag([.003**2,.004**2]), gate=25)

    def pose(self, pose, R):
        H = np.zeros((3,6)); H[:3,:3] = np.eye(3)
        return self.update(pose, H, R, angle_index=2, gate=20)


def icp2(source, target, guess, max_distance=.22, iterations=15):
    """Trimmed point-to-point ICP, initialized from wheel/IMU EKF.

    Returns a global source pose, conservative residual and overlap.
    Rejects nearly collinear geometry instead of making it overconfident.
    """
    if min(len(source),len(target)) < 35:
        return None
    tree = cKDTree(target)
    pose = np.asarray(guess).copy()
    for _ in range(iterations):
        p = transform2(source, pose)
        d, idx = tree.query(p)
        valid = d < max_distance
        if valid.sum() < 35 or valid.mean() < .45:
            return None
        trim = np.quantile(d[valid], .85)
        valid &= d <= trim
        a, b = p[valid], target[idx[valid]]
        ac, bc = a.mean(0), b.mean(0)
        U, sv, Vt = np.linalg.svd((a-ac).T @ (b-bc))
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1; R = Vt.T @ U.T
        da = math.atan2(R[1,0],R[0,0])
        shift = bc - R @ ac
        pose[:2] = R @ pose[:2] + shift
        pose[2] = wrap(pose[2]+da)
        if abs(da)<1e-5 and np.linalg.norm(shift)<1e-4:
            break
    d, idx = tree.query(transform2(source,pose))
    mask = d < max_distance
    rmse = float(np.sqrt(np.mean(d[mask]**2)))
    geometry = np.linalg.eigvalsh(np.cov(source[mask].T))
    if rmse>.08 or geometry[0]/max(geometry[-1],1e-9)<.015:
        return None
    if np.linalg.norm(pose[:2]-guess[:2])>.25 or abs(wrap(pose[2]-guess[2]))>.20:
        return None
    return pose, rmse, float(mask.mean())
