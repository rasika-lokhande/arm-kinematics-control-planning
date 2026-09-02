
from scipy.spatial.transform import Rotation 
import numpy as np

from src.sim_interface import SimInterface
from utils.config import config



def quat_to_R(q):
    return Rotation.from_quat(q, scalar_first=True).as_matrix()

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



def build_Ti(sim, bid, jid, theta):
    body_pos = sim.model.body_pos[bid]
    body_quat = sim.model.body_quat[bid]

    # Build T_fixed
    T_fixed = homog(quat_to_R(body_quat),
                   body_pos)

    # Build T_variable

    jnt_pos = sim.model.jnt_pos[jid]
    jnt_axis = sim.model.jnt_axis[jid]

    T_variable = (homog(np.eye(3), jnt_pos) 
                  @ homog(axis_angle_to_R(jnt_axis,theta), np.zeros(3)) 
                  @ homog(np.eye(3), -jnt_pos))

    # Build Ti

    Ti = T_fixed @ T_variable

    return Ti, T_fixed, jnt_pos, jnt_axis


def forward_kinematics(sim:SimInterface, theta_list):

    jids = sim.get_joint_id_list()
    bids = sim.get_body_id_list()

    T = np.eye(4,4)
    joint_frames = [] 

    for jid, bid, theta in zip(jids, bids, theta_list):

        Ti, T_fixed, jnt_pos, jnt_axis = build_Ti(sim, bid, jid, theta)

        T_WF = T @ T_fixed # Body pose in world frame
        jnt_pos_wf = T_WF[:3, :3] @ jnt_pos + T_WF[:3, 3] # Joint position in world frame
        jnt_axis_wf = T_WF[:3, :3] @ jnt_axis # Joint axis in world frame
        joint_frames.append((jnt_pos_wf, jnt_axis_wf))

        T = T @ Ti 

    site_id = sim.model.site("attachment_site").id
    hand_bid = sim.model.site_bodyid[site_id]

    T_hand = homog(quat_to_R(sim.model.body_quat[hand_bid]), 
                   sim.model.body_pos[hand_bid])
    
    T_site = homog(quat_to_R(sim.model.site_quat[site_id]), 
                   sim.model.site_pos[site_id])

    T = T @ T_hand @ T_site


    return T, joint_frames






if __name__ == '__main__':

    sim = SimInterface(config['MODEL_PATH'])
    q = [1.2, 0.4, -1.0, -0.8, -0.9, 1,1, 0.1]
    T, joint_frames = forward_kinematics(sim, theta_list=q)

    print(T, joint_frames, sep="\n")

    pass


