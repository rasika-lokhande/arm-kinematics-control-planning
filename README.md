# manipulator-kinematics-control

**STATUS**
- Kinematics (FK/IK) - Complete
- Control - Complete
- Motion Planning - Not started

Implemented and validated on a Franka Emika Panda (7-DOF) in MuJoCo.

**Problem Statement**: [docs/problem_statement.md](docs/problem_statement.md)
**Summary report**: [docs/report.md](docs/report.md)

- Forward kinematics via an explicit transform chain, checked against MuJoCo's own site pose 
- Damped least-squares (Levenberg-Marquardt) IK over the Jacobian, with multi-start restarts 
- A task-space PID controller (position + orientation error, damped-pseudoinverse Jacobian) driving the arm to target poses, with gains swept and validated across distance bands 

> - No built-in IK or control library calls are used for any of the three. 
> - The transform chain, the Jacobian, and the control law were implemented directly.

---

## Setup

```bash
pip install -e . # install dependencies
```

Run `python -m scripts.verify_setup` first to sanity-check the model loads and the viewer opens.

## Project structure

```
src/
  kinematics.py             # Forward Kinematics: Building transform chain from joint angles
  ik_solver.py              # Inverse Kinematics: Jacobian + damped least squares solver
  controller.py             # Task-space PID controller driving the arm to a target pose
  transforms.py             # Rotation/homogeneous-transform helper functions
  sim_interface.py          # Thin wrapper around the MuJoCo model/data for joint & pose access
  config.yaml               # Model path, joint limits, velocity limits, result paths

utils/
  config.py                 # Loads src/config.yaml into a shared `config` dict

scripts/
  validate_fk.py            # Compares FK output against MuJoCo's ground-truth pose
  validate_ik.py            # Runs IK over random reachable targets, checks convergence
  validate_controller.py    # Runs the controller over targets at increasing distance bands
  pid_tuning.py             # Sweeps Kp_pos/Kp_rot and plots convergence per target
  verify_setup.py           # Opens the viewer and steps through home/ready poses
  inspect_model.py          # Prints joint/body/site info for the loaded model

results/                    # CSVs and plots produced by the scripts above
```

## Usage

Run any script as a module from the repo root.

```bash
python -m scripts.validate_fk
python -m scripts.validate_ik
python -m scripts.validate_controller
python -m scripts.pid_tuning
```

Each writes its CSVs/plots into the matching `results/` subfolder.

## Results

- **FK**: max position/orientation error against MuJoCo's own reported pose, across 100 random joint configurations ([results/fk_validation/](results/fk_validation/)).
- **IK**: convergence rate, iterations-to-converge, and final error distribution over reachable random targets ([results/ik_validation/](results/ik_validation/)).
- **Control**: convergence rate, settling time, and overshoot as a function of target distance from home ([results/controller_validation/](results/controller_validation/)), plus the Kp sweep used to pick working gains ([results/pid_tuning/](results/pid_tuning/)).



