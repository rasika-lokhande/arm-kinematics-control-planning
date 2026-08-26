import numpy as np
import mujoco.viewer

from src.sim_interface import SimInterface

EE_SITE = "attachment_site"  # replace with whatever inspect_model.py showed
MODEL_PATH = "models/franka_emika_panda/panda_nohand.xml"
READY_POSE = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])  # Franka's typical "ready" pose


def verify_stepping(sim, viewer, n_steps=10):
    for _ in range(n_steps):
        sim.step()
        viewer.sync()
    print("✓ Simulation steps run")


def verify_joint_angles(sim):
    angles = sim.get_joint_angles()
    print(f"✓ Joint angles read ({len(angles)} DOF): {angles}")
    return angles


def verify_pose_at(sim, viewer, angles, label):
    sim.set_joint_angles(angles)
    viewer.sync()
    pos, rot = sim.get_ee_pose()
    print(f"✓ End-effector pose at '{label}':")
    print(f"  Position: {pos}")
    print(f"  Rotation matrix:\n{rot}")
    return pos, rot


def main():
    sim = SimInterface(MODEL_PATH, EE_SITE)
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        verify_stepping(sim, viewer)
        verify_joint_angles(sim)
        verify_pose_at(sim, viewer, np.zeros(7), "home")
        input("Home pose shown. Press Enter to move to ready pose...")

        verify_pose_at(sim, viewer, READY_POSE, "ready")
        input("Ready pose shown. Press Enter to close viewer...")


if __name__ == "__main__":
    main()
