
# Integration script to run the entire planning and control pipeline
from src.planner_rtt_star import rtt_star, extract_path
from src.path_smoothing import node_to_q_path, smooth_path
from src.sim_interface import SimInterface
from utils.config import config
from scripts.validate_planner import generate_blocked_pair
import numpy as np
import time
import mujoco

from src.kinematics import forward_kinematics
from src.ik_solver import run_ik_multistart
from src.controller import controller
from src.motion_planner import plan_path
from src.transforms import homog_to_R_p, R_to_quat
from src.q_validity_check import is_valid, check_collision, check_joint_limits

def get_valid_target_q(sim, target_pos, target_quat, max_attempts=5):
    for attempt in range(max_attempts):
        ik_result = run_ik_multistart(sim, target_pos, target_quat)
        q_goal = np.array(ik_result[0])
        converged = ik_result[3]

        if converged and is_valid(sim, q_goal):
            return q_goal

    return None  # never found a valid, reachable goal


def move_to_target(sim:SimInterface, target_pos, target_quat, failure_retries=3, viewer=None, q_start=None):


    if q_start is None:
        q_start = np.array(sim.get_current_joint_angles_list())


    q_goal = get_valid_target_q(sim, target_pos, target_quat)
    print(q_goal)
        
    path = plan_path(sim, q_start, q_goal, smoothing=True)
    
    success = False

    for _ in range(failure_retries):

        for waypoint, q in enumerate(path):

            T_wp, _ = forward_kinematics(sim, q)
            wp_R, wp_pos = homog_to_R_p(T_wp)
            wp_quat = R_to_quat(wp_R)

            is_last = (waypoint == len(path) - 1)

            controller_result = controller(
                sim, wp_pos, wp_quat,
                Kp_pos=380, Kp_rot=300,
                pos_tol=0.005 if is_last else 0.03,   # loose for intermediate, tight for final
                rot_tol=0.035 if is_last else 0.15,
                settle_window=0.1 if is_last else 0.0,  # don't require settling mid-path
                viewer=viewer
            )

            #controller_result = controller(sim, wp_pos, wp_quat, Kp_pos = 380, Kp_rot=300, viewer=viewer)

            if not controller_result['is_converged']:
                print(f"Warning: did not converge for waypoint {waypoint}")
                print(f"Finding new path...")
                q_current = np.array(sim.get_current_joint_angles_list())
                path = plan_path(sim, q_current, q_goal, smoothing=True)
                break
            else:
                print(f"Reached waypoint {waypoint}")
                if waypoint == len(path) - 1:
                    success = True
                    print("TASK COMPLETE")

        if success:
            break

        
        
       
   

if __name__ == '__main__':

    np.random.seed(10)

    from scripts.validate_planner import generate_blocked_pair
    sim = SimInterface(config['SCENE_NARROW_PATH'])
    
    # Test target 
    q_start, q_goal = generate_blocked_pair(sim)
    sim.set_joint_angles(q_start)
    T_target, _ = forward_kinematics(sim, q_goal)
    target_R, target_pos = homog_to_R_p(T_target)
    target_quat = R_to_quat(target_R)


    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        move_to_target(sim, target_pos, target_quat, viewer=viewer, q_start = q_start)

    
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)

















