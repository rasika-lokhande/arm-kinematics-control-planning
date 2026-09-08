## Report: Manipulator Kinematics and Task-Space Control

Arm: Franka Emika Panda (7-DOF) 
Simulator: MuJoCo

## 1. Forward Kinematics

FK is built by chaining transforms straight from the MuJoCo model itself (joint positions, joint axes, body offsets).

Each body's frame is defined relative to its parent, and each joint's rotation is applied around the joint's own position, not the origin. So the transform has to shift to the joint, rotate, then shift back. 

**Result:** FK matches MuJoCo's own reported pose to about 10⁻¹⁵ m and 10⁻¹⁴ degrees, across 100 test configurations. Essentially exact.

## 2. Inverse Kinematics

IK uses damped least-squares so it doesn't blow up near singularities.

Two issues came up during development:
- **Step size was too small at first.** Raising it to 1.0 fixed slow/non-convergence.
- **Some targets got stuck in local minima** (visible as a sharp early drop in error, then a plateau). Fixed by trying 3 different random starting joint angles per target and keeping whichever one converges.

**Result** (100 random targets):
- **94% converged** (target was ≥90%)
- Median 8 iterations, average 15, out of a 300 budget
- The 6 failures had leftover errors of 1–16 cm — not close calls, so likely genuinely stuck or unreachable cases

## 3. Controller and Final Gains

The controller reads the pose error, runs it through PID, converts that into a joint motion using the same damped-Jacobian trick as IK, and moves the arm one small step. It repeats this every step rather than solving IK once and chasing that single answer.

**Final gains:**

| Term | Position | Orientation |
|---|---|---|
| P | 380 | 300 |
| I | 0 | 0 |
| D | 0 | 0 |

### Why separate position and rotation gains

Early on, one shared gain was applied to the combined 6D error (3 position + 3 rotation) at once. This didn't work well: rotation error is naturally in radians (up to ~π) and position error is in meters (typically under 0.3), so the same gain affects them very differently. The rotation part dominated, and most of the control effort went into fixing orientation while position barely moved. Splitting into separate `Kp_pos` and `Kp_rot` gains fixed this.

Also realized during the implementation: position and rotation motion aren't fully independent even after splitting, because both ultimately move through the same joints. Asking for "pure rotation, no translation" can still shift the end-effector's position slightly. So a small amount of position error moving even when only `Kp_rot` is active is expected.

### Two bugs found during tuning

1. **Gravity sag.** At low gains, error appeared to slowly diverge even when the arm was already near the target. This turned out to be the simulated actuators not pushing back hard enough against gravity when commanded corrections were tiny, so the arm sagged a little every step. Fixed by turning on MuJoCo's built-in gravity compensation (`gravcomp="1"`) on the arm bodies, rather than trying to patch it with an integral term (which would have to constantly change with arm pose).
2. **Velocity clamping** was removed after causing artificial slowness. It was capping how fast the arm could correct itself even when a faster correction was safe and stable.

### Why no I or D term

Once the two bugs above were fixed, plain P converged cleanly with no oscillation or overshoot  even at high gains, the limit was the velocity/torque ceiling, not instability. So:
- **D** wasn't needed since there was no oscillation to damp, and adding it would only add sensitivity to noisy error signals.
- **I** wasn't needed since the steady-state error it would normally fix (the gravity sag) was already solved directly at the model level.

### How gains were picked

A sweep from Kp=10 up to 480 showed: 10 is too weak to converge reliably, results keep improving up to about 300, then only marginally improve after that, with an estimated stability ceiling around Kp≈500 (from `Kp · dt < 1`, using `dt=0.002s`). **380/300** sits past the diminishing-returns point with clear margin below that ceiling, and had zero overshoot across the entire sweep.

## 4. Controller Test Results

100 targets, spread across 5 "how far from home" bands:

| Distance band | Success rate |
|---|---|
| 0.2–0.3 | 100% |
| 0.3–0.4 | 85% |
| 0.4–0.5 | 85% |
| 0.5–0.6 | 60% |
| 0.6–0.7 | 90% |

**Overall: 84/100 succeeded.**

For those 84:
- Settling time: usually under 1 second (well inside the 2–3s budget), though the worst case per band grew steadily with distance (367 → 415 → 720 → 1159 → 3618 iterations)
- Final position error: under 2.5mm (limit was 5mm)
- Final orientation error: under 2° (limit was 2°)
- Zero overshoot, every time

**On the 0.5–0.6 dip:** success rate doesn't drop smoothly with distance (100→85→85→60→90), but typical settling speed stays roughly flat across all bands (~320–400 iterations). It's only the worst-case time that grows with distance. Together this points to specific unlucky directions passing near poorly-conditioned (near-singular) configurations, rather than distance itself being the problem. Separately, targets beyond about 85% of the way to a joint's limit were found to pass through singularities directly.

**This is the reason a motion planner is needed rather than pushing the controller harder.**


## 5. Known gaps

- Every test in the controller starts from home pose. This tells us how the controller does from home, not for arbitrary start-to-target moves, which will matter once it's fed non-home starting points.
- PID gains were tuned on single, large point-to-target jumps; a longer, planner-supplied path (many closer-spaced waypoints) hasn't been tested and may need separate tuning.
- Failures aren't sorted by specific cause (unreachable vs. singularity vs. ran out of time). The singularity/direction hypothesis above is reasoned from indirect evidence, not confirmed per-failure.
- Gains haven't been validated on non-home start points or multi-waypoint paths yet.

