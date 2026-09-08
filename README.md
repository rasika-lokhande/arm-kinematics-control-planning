# manipulator-kinematics-control

STATUS:
Kinematics (FK/IK) - Complete
Control - Complete
Motion Planning - Not started

Implemented and validated on a Franka Emika Panda (7-DOF) in MuJoCo:
- Forward kinematics via an explicit transform chain, checked against MuJoCo's own site pose ([results/fk_validation/](results/fk_validation/))
- Damped least-squares (Levenberg-Marquardt) IK over the Jacobian, with multi-start restarts ([results/ik_validation/](results/ik_validation/))
- A task-space PID controller (position + orientation error, damped-pseudoinverse Jacobian) driving the arm to target poses, with gains swept and validated across distance bands ([results/pid_tuning/](results/pid_tuning/), [results/controller_validation/](results/controller_validation/))

## Problem Statement

This project answers three questions.

**1. Where is the end-effector right now?**

Given the arm's current joint angles, compute the end-effector's pose (position + orientation) directly from the arm's geometry — not by asking the simulator. This is forward kinematics (FK), and it's what makes the other two questions answerable at all: without it, there's no way to know the current pose to compare against a target, or to verify that IK/control are actually working.

> Given: a robot arm (known link lengths/geometry) and a set of joint angles.
> 
> Find: the end-effector's pose, verified against the simulator's own reported pose.

**2. What joint angles put the end-effector at a target pose?**

Given a desired end-effector pose, find the joint angles that achieve it. Unlike FK, this has no closed-form solution in general — it's solved numerically, using the Jacobian to iteratively reduce pose error.

> Given: a robot arm and a desired end-effector pose (target position + orientation).
> 
> Find: joint angles that achieve that pose, within a defined tolerance.

**3. How do I actually move the arm there, smoothly?**

Knowing the target joint angles (or target pose) isn't enough — the arm has to be driven there over time without overshooting or oscillating. This is the control problem: a closed loop that continuously measures pose error and corrects for it.

> Given: the arm's current state and a target pose (or sequence of target poses).
> 
> Find: a sequence of commands over time that drives the end-effector from wherever it currently is to the target — accurately, and without instability (overshoot, oscillation, excessive settling time).

---

No built-in IK or control library calls are used for any of the three — the point is to implement the transform chain, the Jacobian, and the control law directly.

## Setup



```bash
pip install -e .
```


The Panda model ([models/franka_emika_panda/](models/franka_emika_panda/)) and its config path are wired up in [src/config.yaml](src/config.yaml). Run `python -m scripts.verify_setup` first to sanity-check the model loads and the viewer opens.

## Project structure

```
src/
  kinematics.py     FK - builds the transform chain from joint angles
  ik_solver.py       IK - Jacobian + damped least-squares solver, multi-start wrapper
  controller.py       Task-space PID controller driving the arm to a target pose
  transforms.py       Rotation/homogeneous-transform helper functions
  sim_interface.py   Thin wrapper around the MuJoCo model/data for joint & pose access
  config.yaml           Model path, joint limits, velocity limits, result paths

scripts/
  validate_fk.py            Compares FK output against MuJoCo's ground-truth pose
  validate_ik.py            Runs IK over random reachable targets, checks convergence
  validate_controller.py    Runs the controller over targets at increasing distance bands
  pid_tuning.py                Sweeps Kp_pos/Kp_rot and plots convergence per target
  verify_setup.py               Opens the viewer and steps through home/ready poses
  inspect_model.py              Prints joint/body/site info for the loaded model

results/    CSVs and plots produced by the scripts above, one subfolder per validation
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

## Known gaps

