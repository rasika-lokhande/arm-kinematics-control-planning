import os

import pandas as pd
import matplotlib.pyplot as plt

from scripts.pid_tuning import generate_target
from src.controller import controller
from src.sim_interface import SimInterface
from utils.config import config


DEFAULT_BANDS = [(0.2, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 0.6), (0.6, 0.7)]
DEFAULT_SEED = 42
RESULTS_PATH = config['CONTROLLER_VALIDATION_CSV_PATH']


def generate_validation_targets(sim, bands=DEFAULT_BANDS, ntargets=20, seed=DEFAULT_SEED):
    """Generate labeled targets for each distance band."""
    targets = []
    for i, (frac_low, frac_high) in enumerate(bands):
        poses, _ = generate_target(sim, ntargets=ntargets, frac_low=frac_low, frac_high=frac_high, seed=seed + i)
        band_label = f"{frac_low}-{frac_high}"
        targets.extend((pos, quat, band_label) for pos, quat in poses)
    return targets


def run_validation(sim, targets, Kp_pos, Kp_rot):
    """Run the controller on each target and collect results as a DataFrame."""
    results = []
    for i, (target_ee_pos, target_ee_quat, band_label) in enumerate(targets):
        sim.set_joint_angles(sim.theta_home)
        result = controller(sim, target_ee_pos, target_ee_quat, Kp_pos=Kp_pos, Kp_rot=Kp_rot, max_iter = 10000)
        result["band"] = band_label
        result["target_ee_pos"] = target_ee_pos
        result["target_ee_quat"] = target_ee_quat
        result["final_pos_error"] = result["pos_err_history"][-1]
        result["final_rot_error"] = result["rot_err_history"][-1]
        results.append(result)
        print("Target: ", i, " Band: ", band_label)
        print("Converged at ", result["iter_conv"])
        print("-----")

    df = pd.DataFrame(results)
    return df.drop(columns=["pos_err_history", "rot_err_history"])


def save_results(df, path):
    df.to_csv(path)


def load_results(path):
    """Load previously saved validation results, skipping a re-run of the controller."""
    return pd.read_csv(path, index_col=0)


def plot_conv_iter(df, save_path):
    """Scatter plot of iterations-to-converge per target, grouped by band."""
    converged_df = df[df["is_converged"]]

    plt.figure(figsize=(8, 5))
    for band_label, band_df in converged_df.groupby("band"):
        plt.scatter(band_df.index, band_df["iter_conv"], label=band_label)
    plt.xlabel("target index")
    plt.ylabel("iterations to converge")
    plt.title("Convergence speed by target distance band")
    plt.legend(title="band (frac_low-frac_high)")
    plt.tight_layout()
    plt.savefig(save_path)
    plt.show()


def plot_is_converged(df, bands, save_path):
    """Stacked bar chart of converged vs. not-converged counts per band."""
    band_order = [f"{low}-{high}" for low, high in bands]
    counts = df.groupby("band")["is_converged"].value_counts().unstack(fill_value=0)
    counts = counts.reindex(band_order)
    converged = counts.get(True, 0)
    not_converged = counts.get(False, 0)

    plt.figure(figsize=(8, 5))
    plt.bar(band_order, converged, label="converged", color="tab:green")
    plt.bar(band_order, not_converged, bottom=converged, label="not converged", color="tab:red")
    plt.xlabel("target distance band (frac_low-frac_high)")
    plt.ylabel("number of targets")
    plt.title("Convergence outcome by target distance band")
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path)
    plt.show()


def plot_settling_histogram(df, save_path):
    converged_df = df[df["is_converged"]]
    bands = converged_df["band"].unique()

    fig, axes = plt.subplots(1, len(bands), figsize=(4*len(bands), 4), sharey=True)
    for ax, band in zip(axes, sorted(bands)):
        band_df = converged_df[converged_df["band"] == band]
        ax.hist(band_df["iter_conv"], bins=15, color="tab:blue")
        ax.set_title(band)
        ax.set_xlabel("iterations to converge")
    axes[0].set_ylabel("count")
    plt.tight_layout()
    plt.savefig(save_path)
    plt.show()


def plot_settling_seconds(df, dt, budget_s, save_path):
    df = df.copy()
    df["settle_s"] = df["iter_conv"] * dt
    converged_df = df[df["is_converged"]]

    plt.figure(figsize=(8,5))
    converged_df.boxplot(column="settle_s", by="band", ax=plt.gca())
    plt.axhline(budget_s, color="red", linestyle="--", label=f"budget ({budget_s}s)")
    plt.ylabel("settling time (s)")
    plt.title("Settling time by band vs. project budget")
    plt.suptitle("")
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path)
    plt.show()


if __name__ == "__main__":

        

    sim = SimInterface(config['MODEL_PATH'])
    targets = generate_validation_targets(sim, bands=DEFAULT_BANDS)
    df = run_validation(sim, targets, Kp_pos=380, Kp_rot=300)
    save_results(df, RESULTS_PATH)
    print(f"Convergence rate: {df['is_converged'].mean()}")


    df = load_results(RESULTS_PATH)
    plot_conv_iter(df, config['CONTROLLER_VALIDATION_PLOT_PATH'])
    plot_is_converged(df, DEFAULT_BANDS, config['CONTROLLER_CONVERGENCE_BY_BAND_PLOT_PATH'])
    plot_settling_histogram(df, config['CONTROLLER_SETTLING_HISTOGRAM_PLOT_PATH'])
    plot_settling_seconds(df, dt=0.002, budget_s=3.0, save_path=config['CONTROLLER_SETTLING_TIME_PLOT_PATH'])
