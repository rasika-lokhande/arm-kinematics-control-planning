import csv
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from src.ik_solver import run_ik_multistart
from src.kinematics import forward_kinematics, homog_to_R_p
from src.sim_interface import SimInterface
from utils.config import config


def plot_convergence(pos_err_history, rot_err_history, pos_tol, rot_tol, title="IK Convergence",
                      save_path="results/ik_convergence.png"):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)

    ax1.plot(pos_err_history, marker='o', markersize=3)
    ax1.axhline(pos_tol, color='r', linestyle='--', label=f'tol = {pos_tol}')
    ax1.set_ylabel("Position error (m)")
    ax1.set_yscale('log')
    ax1.legend()
    ax1.set_title(title)

    ax2.plot(rot_err_history, marker='o', markersize=3, color='orange')
    ax2.axhline(rot_tol, color='r', linestyle='--', label=f'tol = {rot_tol}')
    ax2.set_ylabel("Orientation error (rad)")
    ax2.set_xlabel("Iteration")
    ax2.set_yscale('log')
    ax2.legend()

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {save_path}")


def plot_failed_trials(failed_trials, pos_tol, rot_tol, n_plots=6, ncols=3,
                        save_path="results/ik_failed_trials.png"):
    """
    failed_trials: list of dicts, one per failed trial, each with keys:
                   'trial_idx', 'pos_hist', 'rot_hist'
    n_plots: max number of failed trials to plot in the grid
    ncols: number of columns in the grid
    """
    if len(failed_trials) == 0:
        print("No failed trials.")
        return

    trials_to_plot = failed_trials[:n_plots]
    n = len(trials_to_plot)
    ncols = min(ncols, n)
    nrows = -(-n // ncols)  # ceil division

    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 3 * nrows), squeeze=False)

    for i, trial in enumerate(trials_to_plot):
        ax1 = axes[i // ncols][i % ncols]
        ax2 = ax1.twinx()

        ax1.plot(trial['pos_hist'], marker='o', markersize=3, label='pos err')
        ax1.axhline(pos_tol, color='b', linestyle='--', linewidth=1)
        ax1.set_ylabel("Position error (m)", color='tab:blue')
        ax1.set_yscale('log')

        ax2.plot(trial['rot_hist'], marker='o', markersize=3, color='orange', label='rot err')
        ax2.axhline(rot_tol, color='orange', linestyle='--', linewidth=1)
        ax2.set_ylabel("Orientation error (rad)", color='tab:orange')
        ax2.set_yscale('log')

        ax1.set_xlabel("Iteration")
        ax1.set_title(f"Trial {trial['trial_idx']}")

    # hide any unused axes in the grid
    for j in range(n, nrows * ncols):
        axes[j // ncols][j % ncols].axis('off')

    fig.suptitle(f"Failed IK Trials (showing {n} of {len(failed_trials)})")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {save_path}")


def plot_convergence_summary(results, max_iter, save_path="results/ik_convergence_summary.png"):
    """
    results: list of dicts, one per trial, each with keys:
             'converged' (bool), 'iter_conv' (int)
    """
    iters = [r['iter_conv'] for r in results if r['converged']]
    converged_flags = [r['converged'] for r in results]

    n_total = len(results)
    n_converged = sum(converged_flags)

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.hist(iters, bins=20, edgecolor='black')
    ax.set_xlabel("Iterations to convergence")
    ax.set_ylabel("Number of trials")
    ax.set_title(f"IK Convergence: {n_converged}/{n_total} converged "
                 f"({100*n_converged/n_total:.0f}%)")
    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {save_path}")


def plot_error_distribution(results, pos_tol, rot_tol, save_path="results/ik_error_distribution.png"):
    """
    Scatter of final position vs. orientation error per trial, split by outcome.

    results: list of dicts, one per trial, each with keys:
             'converged' (bool), 'pos_error_m' (float), 'rot_error_rad' (float)
    """
    converged = [r for r in results if r['converged']]
    failed = [r for r in results if not r['converged']]

    fig, ax = plt.subplots(figsize=(6, 5))

    if converged:
        ax.scatter([r['pos_error_m'] for r in converged], [r['rot_error_rad'] for r in converged],
                   marker='o', s=30, facecolors='none', edgecolors='#0ca30c', linewidths=1.2,
                   label=f'Converged ({len(converged)})')
    if failed:
        ax.scatter([r['pos_error_m'] for r in failed], [r['rot_error_rad'] for r in failed],
                   marker='x', s=40, color='#d03b3b', linewidths=1.5,
                   label=f'Failed ({len(failed)})')

    ax.axvline(pos_tol, color='0.6', linestyle='--', linewidth=1, zorder=0)
    ax.axhline(rot_tol, color='0.6', linestyle='--', linewidth=1, zorder=0)

    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xlabel("Final position error (m)")
    ax.set_ylabel("Final orientation error (rad)")
    ax.set_title("IK Final Error Distribution")
    ax.legend()

    plt.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close(fig)
    print(f"Saved plot to {save_path}")


def save_trial_results(results, save_path="results/ik_validation.csv"):
    """
    results: list of dicts, one per trial, each with keys:
             'converged' (bool), 'iter_conv' (int), 'pos_error_m' (float), 'rot_error_rad' (float)
    """
    with open(save_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["trial", "converged", "iter_conv",
                                                "pos_error_m", "rot_error_rad"])
        writer.writeheader()
        for i, r in enumerate(results):
            writer.writerow({"trial": i, **r})
    print(f"Saved {len(results)} trials to {save_path}")


def generate_reachable_target_poses(sim, ntrials, seed=None):

    if seed is not None:
        np.random.seed(seed)

    test_target_poses = []

    for i in range(ntrials):
        jnt_range = sim.model.jnt_range
        theta_true = np.random.uniform(jnt_range[:, 0], jnt_range[:, 1])
        T_target, _ = forward_kinematics(sim, theta_list=theta_true)
        target_ee_R, target_ee_pos = homog_to_R_p(T_target)
        target_ee_quat = Rotation.from_matrix(target_ee_R).as_quat(scalar_first=True)

        test_target_poses.append((target_ee_pos,target_ee_quat))

    return test_target_poses


def run_trials(sim, ntrials, pos_tol = 0.005, rot_tol=0.035, max_iter=300):

    

    test_target_poses = generate_reachable_target_poses(sim, ntrials, seed=42)
    results = []
    failed_trials = []
    n_failed_trials = 0



    for i, target_pose in enumerate(test_target_poses):
        theta, error_pos, error_rotvec, converged, iter_conv, pos_hist, rot_hist = run_ik_multistart(
            sim, target_ee_pos=target_pose[0], target_ee_quat=target_pose[1],
            pos_tol=pos_tol, rot_tol=rot_tol, max_iter=max_iter
        )
        results.append({
            'converged': converged,
            'iter_conv': iter_conv,
            'pos_error_m': np.linalg.norm(error_pos),
            'rot_error_rad': np.linalg.norm(error_rotvec),
        })

        if not converged:
            failed_trials.append({'trial_idx': i, 'pos_hist': pos_hist, 'rot_hist': rot_hist})
            print("Trial_IDX: ", i)
            n_failed_trials += 1

    convergence_rate = (1 - (n_failed_trials/len(test_target_poses))) * 100
            
    print("Convergence rate = ", convergence_rate, "%")

    return results, convergence_rate, failed_trials



### RUN ###

if __name__ == '__main__':

    sim = SimInterface(config['MODEL_PATH'])

    # hyperparameters
    pos_tol=0.005 
    rot_tol=0.035 
    max_iter=300

    results, convergence_rate, failed_trials = run_trials(sim, 100, pos_tol, rot_tol, max_iter)
    save_trial_results(results)
    plot_convergence_summary(results, max_iter=max_iter)
    plot_failed_trials(failed_trials, pos_tol, rot_tol)
    plot_error_distribution(results, pos_tol, rot_tol)