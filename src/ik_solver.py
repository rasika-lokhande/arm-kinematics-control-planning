from src.sim_interface import SimInterface
from scipy.spatial.transform import Rotation 
from src.kinematics import homog, forward_kinematics, homog_to_R_p, quat_to_R
from utils.config import config
import numpy as np



def compute_jacobian(sim, jnt_axis:np.array, jnt_pos:np.array, p_ee:np.array): 

    J_list = []

    for j in range(sim.model.njnt):
        a_j = jnt_axis[j] # axis of the joint j
        p_j = jnt_pos[j]  # position of the joint j

        J_j = np.append(np.cross(a_j, (p_ee - p_j)) , a_j)

        J_list.append(J_j)

    J = np.column_stack(J_list)

    return J



def run_ik(sim, target_ee_pos, target_ee_quat, theta_init=None,
           lam=0.1, pos_tol=0.001, rot_tol=0.01, max_iter=500, alpha=1.0):

    theta = theta_init if theta_init is not None else np.zeros(sim.model.njnt)
    converged = False
    iter_conv = max_iter

    pos_err_history = []
    rot_err_history = []


    for iter in range(max_iter):

        # Step 1: Run forward kinematics get end-effector pose for current theta

        T_ee, joint_frames = forward_kinematics(sim, 
                                                theta_list=theta)

        # Step 2: Calculate error between target ee pose and ee pose for current theta

        # Get R and pos from T_ee
        current_ee_R , current_ee_pos = homog_to_R_p(T_ee)

        # Get R from target_quat
        target_ee_R = quat_to_R(target_ee_quat)

        # Calculate error
        error_pos = target_ee_pos - current_ee_pos
        error_R = target_ee_R @ current_ee_R.T
        error_rotvec = Rotation.from_matrix(error_R).as_rotvec()
        error = np.append(error_pos, error_rotvec)

        pos_err_history.append(np.linalg.norm(error_pos))
        rot_err_history.append(np.linalg.norm(error_rotvec))


        if np.linalg.norm(error_pos) < pos_tol and np.linalg.norm(error_rotvec) < rot_tol:
            converged = True
            iter_conv = iter
            break

        # Step 3: Compute Jacobian
        jnt_axis = np.array([frame[1] for frame in joint_frames])
        jnt_pos = np.array([frame[0] for frame in joint_frames])

        J = compute_jacobian(sim, jnt_axis, jnt_pos, current_ee_pos)


        # Step 4: Turn error into joint nudge
        I = np.identity(J.shape[0])
        delta_theta = J.T @ np.linalg.inv(J @ J.T + lam**2 * I) @ error
        theta = theta + alpha * delta_theta
        # TO DO: add joint limit clamps

    return theta, error_pos, error_rotvec, converged, iter_conv, pos_err_history, rot_err_history


def run_ik_multistart(sim, target_ee_pos, target_ee_quat, n_restarts=3,
                       lam=0.1, pos_tol=0.005, rot_tol=0.035, max_iter=300, alpha=1.0):


    theta_inits = [np.zeros(sim.model.njnt)]
    for restart in range(n_restarts - 1):
        theta_inits.append(np.random.uniform(sim.model.jnt_range[:, 0], sim.model.jnt_range[:, 1]))


    best_result = None
    best_score = np.inf

    for theta_init in theta_inits:
        result = run_ik(sim, target_ee_pos, target_ee_quat, theta_init=theta_init,
                         lam=lam, pos_tol=pos_tol, rot_tol=rot_tol, max_iter=max_iter, alpha=alpha)
        theta, error_pos, error_rotvec, converged, iter_conv, pos_hist, rot_hist = result

        if converged:
            return result

        score = np.linalg.norm(error_pos) + np.linalg.norm(error_rotvec)
        if score < best_score:
            best_score = score
            best_result = result

    return best_result









    

    







