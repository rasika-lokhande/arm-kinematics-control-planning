# Problem Statement

This project answers three questions.

**1. Where is the end-effector right now?**

Given the arm's current joint angles, compute the end-effector's pose (position + orientation) directly from the arm's geometry, not by asking the simulator. This is forward kinematics (FK), and it's what makes the other two questions answerable at all: without it, there's no way to know the current pose to compare against a target, or to verify that IK/control are actually working.

> Given: a robot arm (known link lengths/geometry) and a set of joint angles.
> 
> Find: the end-effector's pose, verified against the simulator's own reported pose.

**2. What joint angles put the end-effector at a target pose?**

Given a desired end-effector pose, find the joint angles that achieve it. Unlike FK, this has no closed-form solution in general. it's solved numerically, using the Jacobian to iteratively reduce pose error.

> Given: a robot arm and a desired end-effector pose (target position + orientation).
> Find: joint angles that achieve that pose, within a defined tolerance.

**3. How do I actually move the arm there, smoothly?**
Knowing the target joint angles (or target pose) isn't enough. The arm has to be driven there over time without overshooting or oscillating. This is the control problem: a closed loop that continuously measures pose error and corrects for it.

> Given: the arm's current state and a target pose (or sequence of target poses).
> Find: a sequence of commands over time that drives the end-effector from wherever it currently is to the target, accurately, and without instability (overshoot, oscillation, excessive settling time).

