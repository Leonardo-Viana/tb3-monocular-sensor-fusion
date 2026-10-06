"""Candidate temporal-track guard; only timestamps and fused camera rotation."""
import numpy as np

MAX_GAP_S = .65
MAX_ROTATION_RAD = .18

def transition_allowed(previous_stamp, current_stamp, previous_T, current_T):
    dt=float(current_stamp-previous_stamp)
    if not np.isfinite(dt) or dt<=0:return False,'non_monotonic_time'
    if dt>MAX_GAP_S:return False,'image_gap'
    a=np.asarray(previous_T,float);b=np.asarray(current_T,float)
    if a.shape!=(4,4) or b.shape!=(4,4) or not np.isfinite(a).all() or not np.isfinite(b).all():
        return False,'invalid_pose'
    relative=a[:3,:3].T@b[:3,:3]
    angle=float(np.arccos(np.clip((np.trace(relative)-1.)/2.,-1.,1.)))
    if angle>MAX_ROTATION_RAD:return False,'camera_rotation'
    return True,'continuous'
