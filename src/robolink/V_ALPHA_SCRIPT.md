# V-Alpha Script

A domain-specific language for industrial robot arm control.
Readable by non-roboticists. Maps cleanly onto real motion primitives.

Designed by Ijaz Ahmed — derived from hands-on work with CR5 and UR5
arms for fiber composite layup at NUST Pakistan (2023-2024).

---

## Design philosophy

**Intent is separate from execution.**
A script expresses what the robot should do, not how.
The runtime proves the move is safe before sending it to hardware.

**Every motion must be proven before it runs.**
```
Intent → Admission → Kinematic Proof → Execution → Supervision
```

- **Intent** — user writes MOVJ / MOVL
- **Admission** — check servo state, joint limits, motion ownership
- **Kinematic Proof** — resolve frames, compute IK, validate path, check singularity
- **Execution** — send proven trajectory to robot
- **Supervision** — watchdog monitors progress, timeout, collision

v0.1.0 implements Intent + Execution.
v0.2.0 adds Admission + Kinematic Proof.

**Errors must be specific and actionable.**
A script error tells you exactly what is wrong and how to fix it.

---

## Syntax
```
# Joint move
MOVJ(0, -0.8, 0.5, 0, 1.5, 0)   # explicit 6-tuple, radians
MOVJ(0)                           # broadcast — all joints to 0
MOVJ(HOME)                        # named preset
MOVJ(LAYUP_READY)                 # named preset

# Cartesian linear move — v0.2.0
MOVL(0.3, 0.1, 0.4, 180, 0, 0)  # x, y, z metres, rx, ry, rz degrees

# Flow
WAIT(1.5)       # pause seconds
SPEED(0.7)      # speed multiplier 0.0–1.0
REPEAT(3)       # repeat block
    MOVJ(...)
END

# Comments
# anything after # is ignored
```

---

## Why MOVJ and MOVL

These are the two fundamental motion types in industrial robotics:

**MOVJ** — joint interpolation. Each joint moves independently to
its target. Fast, no guaranteed TCP path. Use for repositioning.

**MOVL** — linear interpolation in Cartesian space. TCP moves in a
straight line. Requires IK at each step. Use for process moves —
welding, layup, dispensing — where the path matters.

The same distinction exists in every major industrial system:

| Vendor | Joint move | Linear move |
|--------|-----------|-------------|
| FANUC  | J         | L           |
| ABB    | MoveJ     | MoveL       |
| KUKA   | PTP       | LIN         |
| UR     | moveJ     | moveL       |
| V-Alpha| MOVJ      | MOVL        |

---

## Backend contract (v0.2.0)

To implement MOVL, a backend must provide:
```python
def solve(pose, current_joints, tool) -> IKResult
    # Return all valid IK candidates with singularity metric

def fk(joints, tool) -> Pose
    # Forward kinematics — joints to TCP pose

def jacobian_normalised_sigma_min(joints, characteristic_length_mm) -> float
    # Smallest singular value of the normalised Jacobian
    # This is the singularity metric — physics-aware, not a heuristic
    J = compute_jacobian(joints)          # shape (6, 6)
    J_norm = J.copy()
    J_norm[0:3, :] /= characteristic_length_mm
    return np.linalg.svd(J_norm, compute_uv=False)[-1]
```

---

## Roadmap

| Feature | Version |
|---------|---------|
| MOVJ — joint move | ✅ v0.1.0 |
| MOVL — parsed, NotImplementedError | ✅ v0.1.0 |
| WAIT, SPEED, REPEAT | ✅ v0.1.0 |
| Named presets (configurable per robot) | ✅ v0.1.0 |
| Admission checks | 🔄 v0.2.0 |
| MOVL execution via MoveIt2 | 🔄 v0.2.0 |
| Singularity detection (Jacobian sigma_min) | 🔄 v0.2.0 |
| TOOL() and FRAME() commands | 🔄 v0.2.0 |
| Fault recovery and supervision | 🔄 v0.2.0 |
