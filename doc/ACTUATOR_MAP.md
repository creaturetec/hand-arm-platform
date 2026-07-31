# Actuator & Joint Index Map

> **Critical**: `ctrl[]` and `qpos[]` have **different orderings**. See [Index Ordering](#ctrl-vs-qpos-ordering) below.

All 54 actuators are **position servos** (`<position>` in MJCF). Set `data.ctrl[i]` to the desired joint angle in radians.

---

## ctrl[] Index Table

### Arm Actuators (indices 0–9) — with force limits

| ctrl | Actuator Name | Joint | Motor | Range (rad) | kp | kv | Force (N·m) |
|:----:|---------------|-------|-------|-------------|---:|---:|:-----------:|
| 0 | `left_joint1_ctrl` | `openarm_left_joint1` | DM8009 | [−3.491, 1.396] | 230 | 14 | ±40 |
| 1 | `left_joint2_ctrl` | `openarm_left_joint2` | DM8009 | [−3.316, 0.175] | 230 | 12 | ±40 |
| 2 | `left_joint3_ctrl` | `openarm_left_joint3` | DM4340 | [−1.571, 1.571] | 190 | 14 | ±27 |
| 3 | `left_joint4_ctrl` | `openarm_left_joint4` | DM4340 | [0, 2.444] | 190 | 14 | ±27 |
| 4 | `left_joint5_ctrl` | `openarm_left_joint5` | DM4310 | [−1.571, 1.571] | 30 | 1.5 | ±7 |
| 5 | `right_joint1_ctrl` | `openarm_right_joint1` | DM8009 | [−1.396, 3.491] | 230 | 14 | ±40 |
| 6 | `right_joint2_ctrl` | `openarm_right_joint2` | DM8009 | [−0.175, 3.316] | 230 | 12 | ±40 |
| 7 | `right_joint3_ctrl` | `openarm_right_joint3` | DM4340 | [−1.571, 1.571] | 190 | 14 | ±27 |
| 8 | `right_joint4_ctrl` | `openarm_right_joint4` | DM4340 | [0, 2.444] | 190 | 14 | ±27 |
| 9 | `right_joint5_ctrl` | `openarm_right_joint5` | DM4310 | [−1.571, 1.571] | 30 | 1.5 | ±7 |

### Left Hand Actuators (indices 10–31) — NO force limits

| ctrl | Actuator | Joint | Range (rad) | kp | kv |
|:----:|----------|-------|-------------|---:|---:|
| 10 | `WRIST-PALM-L_ctrl` | WRIST-PALM-L | [−1.222, 1.222] | 13.2 | 0.404 |
| 11 | `PALM-L_ctrl` | PALM-L | [−0.785, 0.785] | 6.95 | 0.285 |
| 12–15 | `F1-L-{MCP2,MCP1,PIP,DIP}_ctrl` | Thumb L | see XML | 6.62/4.76/0.9/0.9 | — |
| 16–19 | `F2-L-{MCP2,MCP1,PIP,DIP}_ctrl` | Index L | see XML | 6.62/4.76/0.9/0.9 | — |
| 20–23 | `F3-L-{MCP2,MCP1,PIP,DIP}_ctrl` | Middle L | see XML | 6.62/4.76/0.9/0.9 | — |
| 24–27 | `F4-L-{MCP2,MCP1,PIP,DIP}_ctrl` | Ring L | see XML | 6.62/4.76/0.9/0.9 | — |
| 28–31 | `F5-L-{MCP2,MCP1,PIP,DIP}_ctrl` | Pinky L | see XML | 6.62/4.76/0.9/0.9 | — |

### Right Hand Actuators (indices 32–53) — NO force limits

Symmetric to left hand. `ctrl[32]` = `WRIST-PALM-R_ctrl`, ..., `ctrl[53]` = `F5-R-DIP_ctrl`.

> **Thumb = F1, Index = F2, Middle = F3, Ring = F4, Pinky = F5**
> Per-finger chain: MCP2 (ab/adduction) → MCP1 (flexion) → PIP → DIP

### Hand Joint Ranges (relative to current `qpos = 0`)

All hand limits are measured from the current default pose. The same numeric limits apply to the left and right hands; mirrored joint axes produce mirrored physical motion.

| Joint group | Joint | Range (deg) | Range (rad) |
|-------------|-------|-------------|-------------|
| Wrist WR1 | `WRIST-PALM-{L\|R}` | [−70°, +70°] | [−1.2217305, 1.2217305] |
| Wrist WR2 | `PALM-{L\|R}` | [−45°, +45°] | [−0.7853982, 0.7853982] |
| Thumb F1 | MCP2 | [−25°, +45°] | [−0.4363323, 0.7853982] |
| Thumb F1 | MCP1 | [−35°, +50°] | [−0.6108652, 0.8726646] |
| Thumb F1 | PIP | [−30°, +75°] | [−0.5235988, 1.3089969] |
| Thumb F1 | DIP | [−25°, +80°] | [−0.4363323, 1.3962634] |
| Fingers F2–F5 | MCP1 | [−35°, +95°] | [−0.6108652, 1.6580628] |
| Fingers F2–F5 | PIP | [−5°, +105°] | [−0.0872665, 1.8325957] |
| Fingers F2–F5 | DIP | [−5°, +75°] | [−0.0872665, 1.3089969] |

MCP2 ab/adduction limits vary by finger:

| Finger | Range (deg) | Range (rad) |
|--------|-------------|-------------|
| F2 (index) | [−40°, +40°] | [−0.6981317, 0.6981317] |
| F3 (middle) | [−40°, +35°] | [−0.6981317, 0.6108652] |
| F4 (ring) | [−40°, +30°] | [−0.6981317, 0.5235988] |
| F5 (pinky) | [−40°, +40°] | [−0.6981317, 0.6981317] |

---

## ctrl[] vs qpos[] Ordering

**These are NOT the same order.** This is the most common source of indexing bugs.

| Vector | Layout |
|--------|--------|
| `data.ctrl[0:54]` | `[L-arm(5), R-arm(5), L-hand(22), R-hand(22)]` |
| `data.qpos[0:54]` | `[L-arm(5), L-hand(22), R-arm(5), R-hand(22)]` |

Concretely: `ctrl[5] = right_joint1` but `qpos[5] = WRIST-PALM-L`. **Do not assume they match.**

### qpos[] Index Table

| qpos | Joint Group |
|:----:|-------------|
| 0–4 | Left arm: `openarm_left_joint{1..5}` |
| 5–6 | `WRIST-PALM-L`, `PALM-L` |
| 7–26 | Left hand fingers: F1-L through F5-L (4 joints each) |
| 27–31 | Right arm: `openarm_right_joint{1..5}` |
| 32–33 | `WRIST-PALM-R`, `PALM-R` |
| 34–53 | Right hand fingers: F1-R through F5-R (4 joints each) |

### Safe Mapping via API

```python
import mujoco
model = mujoco.MjModel.from_xml_path("bimanualHand/openarm_hand_bimanual.xml")
data  = mujoco.MjData(model)

# Print ctrl[] → qpos[] mapping
for i in range(model.nu):
    act  = model.actuator(i)
    jnt  = model.joint(act.trnid[0])
    print(f"ctrl[{i:2d}] → qpos[{jnt.qposadr[0]:2d}]  {act.name}")

# Set a joint by name (always safe)
ctrl_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "F1-L-MCP1_ctrl")
data.ctrl[ctrl_id] = 0.5  # radians
```

---

## Home Keyframe and Default Hand Pose

The MJCF `home` keyframe is **not all-zeros**:

- `qpos[3]` (`openarm_left_joint4`) = **π/2** (1.5708 rad)
- `qpos[30]` (`openarm_right_joint4`) = **π/2** (1.5708 rad)
- All other joints, including all 44 hand joints, = 0.0

Both elbow joints fold forward at π/2. The hand limits above are relative to these zero-valued hand joint coordinates. Initial contact count is `ncon == 0`.

### Default finger-frame orientation

The nonzero neutral spread of F2, F4, and F5 is encoded in the URDF joint `origin/rpy` and the equivalent MJCF body quaternion, not in `qpos`. Therefore resetting every hand joint to zero preserves the current default finger pose.

| MCP2 joint | Right URDF `rpy` (rad) | Left URDF `rpy` (rad) | Physical neutral offset |
|------------|--------------------------|-------------------------|-------------------------|
| F2 | `0 0 −0.2618` | `3.1416 0 2.8798` | right −15°, left +15° |
| F3 | `0 0 0` | `3.1416 0 3.1416` | 0° |
| F4 | `0 0 0.2618` | `3.1416 0 3.4034` | right +15°, left −15° |
| F5 | `0 0 0.5934` | `3.1416 0 3.7350` | right +34°, left −34° |

The left-hand RPY values include the existing mirrored-link transform (`roll ≈ π`).

---

## Naming Convention

```
Arm joints:   openarm_{left|right}_joint{1..5}
Arm ctrls:    {left|right}_joint{1..5}_ctrl

Hand joints:  {WRIST-PALM|PALM|F{1..5}}-{L|R}[-{MCP2|MCP1|PIP|DIP}]
Hand ctrls:   {joint_name}_ctrl
```
