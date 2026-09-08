
# Some utility transform functions
from scipy.spatial.transform import Rotation 
import numpy as np


def quat_to_R(q):
    return Rotation.from_quat(q, scalar_first=True).as_matrix()

def R_to_quat(R):
    r = Rotation.from_matrix(R)
    return r.as_quat(scalar_first=True)

def axis_angle_to_R(axis, theta):
    axis = axis / np.linalg.norm(axis) # Normalizing to a unit vector
    rot_vec = axis * theta # In this context, theta is current joint angle
    R_matrix = Rotation.from_rotvec(rot_vec).as_matrix()
    return R_matrix


def homog(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def homog_to_R_p(T):
    R = T[:3, :3].copy() 
    p = T[:3, 3].copy()
    return R,p


def rotate_quat(quat, axis, deg):

    axis_map = {
        'x': np.array([1.0, 0.0, 0.0]),
        'y': np.array([0.0, 1.0, 0.0]),
        'z': np.array([0.0, 0.0, 1.0]),
    }
    if axis not in axis_map:
        raise ValueError(f"axis must be one of 'x', 'y', 'z', got {axis!r}")

    theta = np.deg2rad(deg)
    R_delta = axis_angle_to_R(axis_map[axis], theta)
    R_current = quat_to_R(np.asarray(quat))

    R_new = R_delta @ R_current
    return R_to_quat(R_new)

