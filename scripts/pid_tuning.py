import numpy as np
from src.sim_interface import SimInterface
from utils.config import config
from src.controller import controller
from src.kinematics import forward_kinematics
from src.transforms import homog_to_R_p, R_to_quat
import matplotlib.pyplot as plt
import pandas as pd
import mujoco
import time



JOINT_LIMITS_LOW = config['JOINT_LIMITS_LOW']
JOINT_LIMITS_HIGH = config['JOINT_LIMITS_HIGH']
seed = 42





def generate_target(sim, ntargets:int,
                            frac_low=0.6, frac_high=0.85, margin=0.05, seed=seed):
    """
    Sample a target far from home, but guaranteed within joint limits.
    Each joint moves 60-85%(default) of the distance from home to its limit,
    in a randomly chosen direction (toward high or low limit).
    """
    theta_home = sim.theta_home
    n = len(theta_home)
    joint_limits_low  = np.array(JOINT_LIMITS_LOW)
    joint_limits_high = np.array(JOINT_LIMITS_HIGH)
    rng = np.random.default_rng(seed)

    test_target_poses = []
    test_theta = []
    while len(test_target_poses) < ntargets:
        target_theta = np.zeros(n)
        signs = rng.choice([-1, 1], size=n)
        for j in range(n):
            if signs[j] > 0: available = (joint_limits_high[j] - margin) - theta_home[j]
            else: available = theta_home[j] - (joint_limits_low[j] + margin)
            frac = rng.uniform(frac_low, frac_high)
            target_theta[j] = theta_home[j] + signs[j] * frac * available
        T_target, _ = forward_kinematics(sim, target_theta)
        target_ee_R, target_ee_pos = homog_to_R_p(T_target)
        target_ee_quat = R_to_quat(target_ee_R)
        test_target_poses.append((target_ee_pos, target_ee_quat))
        test_theta.append(target_theta)

    return test_target_poses, test_theta

def plot_convergence(controller_result, pos_tol, rot_tol, kp_pos, kp_rot, target_pose, t_idx=None):

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    iter_conv = controller_result.get("iter_conv")
    is_converged = controller_result.get("is_converged")

    def mark_convergence(ax, err_history):
        if not is_converged or iter_conv is None:
            return
        ax.axvline(iter_conv, color="green", linestyle=":", label=f"converged (iter {iter_conv})")
        ax.plot(iter_conv, err_history[iter_conv], "go", zorder=5)

    axes[0].plot(controller_result["pos_err_history"], label="pos error")
    axes[0].axhline(pos_tol, color="red", linestyle="--", label=f"tolerance ({pos_tol})")
    mark_convergence(axes[0], controller_result["pos_err_history"])
    axes[0].set_title(f"Position error at kp_pos = {kp_pos}")
    axes[0].set_xlabel("iteration")
    axes[0].legend()

    axes[1].plot(controller_result["rot_err_history"], label="rot error")
    axes[1].axhline(rot_tol, color="red", linestyle="--", label=f"tolerance ({rot_tol})")
    mark_convergence(axes[1], controller_result["rot_err_history"])
    axes[1].set_title(f"Rotation error at kp_rot = {kp_rot}")
    axes[1].set_xlabel("iteration")
    axes[1].legend()



    if t_idx is not None:
        fig.suptitle(f't_idx: {t_idx}')
    else:
        fig.suptitle(f'Target: {target_pose}')

    plt.tight_layout()


    if t_idx is not None:
        plt.savefig(f"{config['PID_TUNING_PLOT_DIR']}/target{t_idx}.png")
    plt.show()

def print_summary(target_pose, controller_result, kp_pos, kp_rot):
    pos, quat = target_pose[:3], target_pose[3:]
    converged = controller_result["is_converged"]
    status = "CONVERGED" if converged else "NOT CONVERGED"

    print("=" * 40)
    print("Controller run summary")
    print("-" * 40)
    print(f"{'target pos':<14}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]")
    print(f"{'target quat':<14}: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
    print(f"{'gains':<14}: Kp_pos = {kp_pos} | Kp_rot = {kp_rot}")
    print("-" * 40)
    print(f"{'status':<14}: {status} (iter {controller_result['iter_conv']})")
    print(f"{'final pos err':<14}: {controller_result['pos_err_history'][-1]:.5f}")
    print(f"{'final rot err':<14}: {controller_result['rot_err_history'][-1]:.5f}")
    print("=" * 40)




def tune_kp(sim, target_pose_list:list, 
            kp_pos_values:np.ndarray, kp_rot_values:np.ndarray,
            ki_pos=0.0, ki_rot=0.0, kd_pos=0.0, kd_rot=0.0,
            pos_tol = 0.005, rot_tol=0.035, max_iter = 15000):

    sim.reset()
    tuning_result = []
    run_id = time.strftime('%Y%m%d_%H%M%S')

    def sweep(target_pose):
        target_ee_pos, target_ee_quat = target_pose

        def format_tuning_result():
            row = [t_idx,
                target_ee_pos, target_ee_quat,
                kp_pos, kp_rot,
                controller_result['is_converged'],
                controller_result['iter_conv'],
                controller_result['pos_overshoot'],
                controller_result['rot_overshoot'],
                controller_result['pos_err_history'][-1],
                controller_result['rot_err_history'][-1]]
            return row

        for kp_pos in kp_pos_values:
            for kp_rot in kp_rot_values:
                sim.set_joint_angles(sim.theta_home)
                controller_result = controller(sim, target_ee_pos, target_ee_quat,
                                    Kp_pos=kp_pos, Ki_pos=ki_pos, Kd_pos=kd_pos,
                                    Kp_rot=kp_rot, Ki_rot=ki_rot, Kd_rot=kd_rot,
                                    pos_tol=pos_tol, rot_tol=rot_tol,
                                    max_iter=max_iter)

                # Compute overshoot
                # pos_overshoot = compute_overshoot(controller_result["pos_err_history"], pos_tol)
                # rot_overshoot = compute_overshoot(controller_result["rot_err_history"], rot_tol)
                # print(pos_overshoot,rot_overshoot)
                print_summary(np.append(target_ee_pos, target_ee_quat), controller_result, kp_pos, kp_rot)
                tuning_result_row = format_tuning_result()
                tuning_result.append(tuning_result_row)

    def save_tuning_result():
        columns = [
            "t_idx",
            "target_ee_pos", "target_ee_quat", 
            "Kp_pos", "Kp_rot",
            "is_converged", "iter_conv", 
            "pos_overshoot", "rot_overshoot",
            "final_pos_err", "final_rot_err",
        ]
        
      
        base_path = config['PID_TUNING_RESULTS_PATH']
        save_path = base_path.replace('.csv', f'_{run_id}.csv') if base_path.endswith('.csv') else f"{base_path}_{run_id}"
        pd.DataFrame(tuning_result, columns=columns).to_csv(save_path)
      
        


    for t_idx, target_pose in enumerate(target_pose_list):
        sweep(target_pose)
    save_tuning_result()
    

    
def run_controller(
        sim, target_ee_pos, target_ee_quat,
        Kp_pos=50, Ki_pos=0.0, Kd_pos=0.0,
        Kp_rot=50, Ki_rot=0.0, Kd_rot=0.0,
        pos_tol=0.005, rot_tol=0.035,
        max_iter=15000, t_idx = None
    ):
    """Run controller with specific parameter values and disploy plot and print summary"""
    target_pose = np.append(target_ee_pos,target_ee_quat) # 7D vector
    result = controller(
        sim, target_ee_pos, target_ee_quat,
        Kp_pos=Kp_pos, Ki_pos=Ki_pos, Kd_pos=Kd_pos,
        Kp_rot=Kp_rot, Ki_rot=Ki_rot, Kd_rot=Kd_rot,
        pos_tol=pos_tol, rot_tol=rot_tol,
        max_iter=max_iter,
    )
    
    print_summary(target_pose, result, Kp_pos, Kp_rot)
    plot_convergence(result, pos_tol, rot_tol, Kp_pos, Kp_rot, target_pose, t_idx)





def generate_tuning_targets(sim):

    frac_range = [(0.1, 0.3), (0.3,0.5), (0.5, 0.7)]
    targets = []
    for t_idx, frac in enumerate(frac_range):
        target_pose, _ = generate_target(sim, 1, frac_low = frac[0], frac_high=frac[1], margin = 0.05, seed = seed + t_idx)
        targets.append(target_pose[0])
    return targets









if __name__ == "__main__":
    sim = SimInterface(config['MODEL_PATH'])

    targets = generate_tuning_targets(sim)

    kp_pos_values= np.logspace(np.log10(10), np.log10(100), num=5)   
    kp_rot_values = np.logspace(np.log10(10), np.log10(100), num=5)   

    # tune_kp(sim, targets, kp_pos_values, kp_rot_values)





    run_controller(sim, targets[0][0], targets[0][1],
                   Kp_pos = 380, Kp_rot= 300, Ki_rot=0.0, Kd_rot = 0.0, t_idx = 0)



  


