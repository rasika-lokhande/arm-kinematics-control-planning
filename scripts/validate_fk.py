import numpy as np
import csv
import mujoco
from src.sim_interface import SimInterface
from utils.config import config
from src.kinematics import forward_kinematics
from scipy.spatial.transform import Rotation 
import matplotlib.pyplot as plt

sim = SimInterface(config['MODEL_PATH'])
results = []



def run_trials(n):
    np.random.seed(0)
    errors = []
    
    for trial in range(n):
            
        theta_test = np.random.uniform(-1.5, 1.5, size=7)

        # Push joint angles into simulator
        sim.data.qpos[:7] = theta_test
        mujoco.mj_forward(sim.model, sim.data)

        # Read MuJoCo's ground-truth end-effector pose
        ee_site_id = sim.model.site("attachment_site").id
        p_true = sim.data.site_xpos[ee_site_id].copy()          # position, world frame
        R_true = sim.data.site_xmat[ee_site_id].reshape(3, 3).copy()  # rotation, world frame


        # Run my FK on same angles
        T_mine, _ = forward_kinematics(sim, theta_test)
        p_mine = T_mine[:3, 3]
        R_mine = T_mine[:3, :3]

        # Compare
        pos_error = np.linalg.norm(p_mine - p_true)
        # orientation error via relative rotation 
        R_err = Rotation.from_matrix(R_true.T @ R_mine)
        ori_error_deg = np.rad2deg(np.linalg.norm(R_err.as_rotvec()))


        errors.append((trial, pos_error, ori_error_deg))
        print(f"Trial {trial}: pos={pos_error:.3e} m, ori={ori_error_deg:.3e} deg")

        results.append({
            "trial": trial,
            "theta": theta_test.tolist(),
            "pos_error_m": pos_error,
            "ori_error_deg": ori_error_deg,
        })

    with open("results/fk_validation.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["trial", "theta", "pos_error_m", "ori_error_deg"])
        writer.writeheader()
        writer.writerows(results)

    
    max_pos = max(r["pos_error_m"] for r in results)
    max_ori = max(r["ori_error_deg"] for r in results)
    print(f"Saved {len(results)} trials to results/fk_validation.csv")
    print(f"Max position error: {max_pos:.3e} m | Max orientation error: {max_ori:.3e} deg")





def plot_fk_validation(results, pos_threshold=0.001, ori_threshold=0.5,
                        save_path="results/fk_validation.png"):
    """
    Plot FK validation errors (position + orientation) across test trials.

    Parameters
    ----------
    results : list of dict
        Each dict must have keys: "trial", "pos_error_m", "ori_error_deg"
    pos_threshold : float
        Position error threshold in meters (default 1mm)
    ori_threshold : float
        Orientation error threshold in degrees (default 0.5deg)
    save_path : str
        Where to save the resulting PNG
    """
    trials = [r["trial"] for r in results]
    pos_errors = [r["pos_error_m"] for r in results]
    ori_errors = [r["ori_error_deg"] for r in results]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

    ax1.scatter(trials, pos_errors, color="tab:blue")
    ax1.set_yscale("log")
    ax1.set_xlabel("Trial")
    ax1.set_ylabel("Position error (m)")
    ax1.set_title("FK position error per trial")
    ax1.axhline(pos_threshold, color="red", linestyle="--",
                label=f"{pos_threshold*1000:.1f} mm threshold")
    ax1.legend()
    ax1.grid(True, which="both", alpha=0.3)

    ax2.scatter(trials, ori_errors, color="tab:green")
    ax2.set_yscale("log")
    ax2.set_xlabel("Trial")
    ax2.set_ylabel("Orientation error (deg)")
    ax2.set_title("FK orientation error per trial")
    ax2.axhline(ori_threshold, color="red", linestyle="--",
                label=f"{ori_threshold}° threshold")
    ax2.legend()
    ax2.grid(True, which="both", alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {save_path}")





if __name__ == '__main__':
    run_trials(100)
    plot_fk_validation(results)