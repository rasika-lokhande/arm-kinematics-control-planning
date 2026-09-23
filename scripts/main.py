import argparse
import time
from pathlib import Path

import numpy as np
import mujoco
import yaml

from src.sim_interface import SimInterface
from src.orchestrator import move_to_target
from src.kinematics import forward_kinematics
from src.transforms import homog_to_R_p, R_to_quat
from scripts.validate_planner import generate_blocked_pair
from utils.config import config as static_config

DEFAULT_PARAMS_PATH = Path(__file__).resolve().parent / "config" / "params.yaml"


def load_params(path):
    with open(path) as f:
        return yaml.safe_load(f)


def resolve_target(sim, params):
    mode = params["target"]["mode"]

    if mode == "fixed":
        q_start = np.array(sim.get_current_joint_angles_list())
        return q_start, np.array(params["target"]["pos"]), np.array(params["target"]["quat"])

    if mode == "blocked_pair":
        q_start, q_goal = generate_blocked_pair(sim)
        sim.set_joint_angles(q_start)
        T_goal, _ = forward_kinematics(sim, q_goal)
        R_goal, pos_goal = homog_to_R_p(T_goal)
        return q_start, pos_goal, R_to_quat(R_goal)
    raise ValueError(f"Unknown target mode: {mode}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--params", default=str(DEFAULT_PARAMS_PATH))
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()

    params = load_params(args.params)
    np.random.seed(params["seed"])

    sim = SimInterface(static_config[params["scene"]])
    q_start, target_pos, target_quat = resolve_target(sim, params)

    use_viewer = params.get("viewer", True) and not args.no_viewer

    move_kwargs = dict(
        q_start=q_start,
        planner_kwargs=params["planner"],
        failure_retries=params["execution"]["failure_retries"],
        **params["controller"],
    )

    if use_viewer:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            move_to_target(sim, target_pos, target_quat, viewer=viewer, **move_kwargs)
            while viewer.is_running():
                viewer.sync()
                time.sleep(0.01)
    else:
        move_to_target(sim, target_pos, target_quat, **move_kwargs)


if __name__ == "__main__":
    main()
