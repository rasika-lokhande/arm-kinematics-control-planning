import time
import numpy as np
from src.sim_interface import SimInterface
from src.transforms import homog_to_R_p, quat_to_R, R_to_quat
from src.ik_solver import compute_error, compute_jacobian
from src.kinematics import forward_kinematics



def pid(error_pos, error_rot, Kp_pos, Ki_pos, Kd_pos, Kp_rot, Ki_rot, Kd_rot,
        previous_I_pos, previous_I_rot, previous_error_pos, previous_error_rot, dt):

    # P = error * Kp
    # I = previous_I + (error * dt * Ki) 
    # D = ((error - previous_error) / dt ) * Kd

    P_pos = error_pos * Kp_pos
    I_pos = previous_I_pos + (error_pos * dt * Ki_pos)
    D_pos = ((error_pos - previous_error_pos) / dt) * Kd_pos
    task_vel_pos = P_pos + I_pos + D_pos

    P_rot = error_rot * Kp_rot
    I_rot = previous_I_rot + (error_rot * dt * Ki_rot)
    D_rot = ((error_rot - previous_error_rot) / dt) * Kd_rot
    task_vel_rot = P_rot + I_rot + D_rot

    task_vel = np.append(task_vel_pos, task_vel_rot)
    return task_vel, I_pos, I_rot

def compute_overshoot(err_history, tol):

    err = np.asarray(err_history)
    below = err < tol
    if not below.any():
        return None  # never dipped below tolerance

    first_below = np.argmax(below)  # index of first crossing under tol

    after = err[first_below:]
    if np.all(after < tol):
        return 0.0  # stayed within tolerance once reached - no overshoot

    return float(after.max() - tol)  # how far it bounced back out



def controller(sim: SimInterface, target_ee_pos, target_ee_quat,
               Kp_pos=5.0, Ki_pos=0.0, Kd_pos=0.0,
               Kp_rot=5.0, Ki_rot=0.0, Kd_rot=0.0,
               lam=0.01, pos_tol=0.005, rot_tol=0.035, max_iter=5000,
               settle_window = 0.1, viewer=None):
 
    previous_error_pos = np.zeros(3)
    previous_error_rot = np.zeros(3)
    previous_I_pos = np.zeros(3)
    previous_I_rot = np.zeros(3)
    dt = sim.model.opt.timestep
    #qvel_max = np.array(config["QVEL_MAX"])
    settle_iters = max(1, round(settle_window / dt))

    pos_err_history = []
    rot_err_history = []
    is_converged = False
    iter_conv = None
    settle_count = 0
    first_crossing_iter = None


    for i in range(max_iter):

        # Step 1: Compute error

        # Get current joint angles -> current theta_vector
        current_theta = sim.get_current_joint_angles_list()

        T_ee, joint_frames = forward_kinematics(sim, current_theta)
        current_ee_R , current_ee_pos = homog_to_R_p(T_ee)

        target_ee_R = quat_to_R(target_ee_quat)
        error_pos, error_rotvec = compute_error(target_ee_pos, target_ee_R, current_ee_pos, current_ee_R)
        error = np.append(error_pos, error_rotvec)
        pos_err_history.append(np.linalg.norm(error_pos))
        rot_err_history.append(np.linalg.norm(error_rotvec))

 
        # Check for convergence
        if np.linalg.norm(error_pos) < pos_tol and np.linalg.norm(error_rotvec) < rot_tol:
            if settle_count == 0:
                first_crossing_iter = i # Record first crossing of this streak
            settle_count += 1
            if settle_count >= settle_iters:
                is_converged = True
                iter_conv = first_crossing_iter
                break
        else:
            settle_count = 0
            first_crossing_iter = None


        # Step 2: Compute task_vel using PID. 

        task_vel, new_I_pos, new_I_rot = pid(
                    error_pos, error_rotvec,
                    Kp_pos, Ki_pos, Kd_pos,
                    Kp_rot, Ki_rot, Kd_rot,
                    previous_I_pos, previous_I_rot,
                    previous_error_pos, previous_error_rot,
                    dt,
                )

        # Step 3: Compute qvel from task_vel. (qvel = vector of velocities of each joint -> delta_theta/dt)

        jnt_axis = np.array([frame[1] for frame in joint_frames])
        jnt_pos = np.array([frame[0] for frame in joint_frames])
        J = compute_jacobian(sim, jnt_axis, jnt_pos, current_ee_pos)
        I = np.identity(J.shape[0])
        qvel = J.T @ np.linalg.inv(J @ J.T + lam**2 * I) @ task_vel
        
       
    

        # Step 4: Calculate target_theta (for this time step) from qvel
        
        delta_theta = qvel * dt
        target_theta = current_theta + delta_theta


        # Step 5: Send the cmd to the simulation and update simulation step

        sim.data.ctrl[:] = target_theta
        sim.step()

        if viewer is not None:
            viewer.sync()
            time.sleep(dt)

        # Update running state for next iteration
        previous_error_pos = error_pos.copy()
        previous_error_rot = error_rotvec.copy()
        previous_I_pos = new_I_pos.copy()
        previous_I_rot = new_I_rot.copy()

    pos_overshoot = compute_overshoot(pos_err_history, pos_tol)
    rot_overshoot = compute_overshoot(rot_err_history, rot_tol)
    theta_final = sim.get_current_joint_angles_list()

    return {
    "theta_final": theta_final,       
    "is_converged": is_converged,
    "iter_conv": iter_conv,
    "first_crossing_iter": first_crossing_iter,
    "pos_overshoot": pos_overshoot,
    "rot_overshoot": rot_overshoot,
    "pos_err_history": pos_err_history,
    "rot_err_history": rot_err_history,
}
        

    
