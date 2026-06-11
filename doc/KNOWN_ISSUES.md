# Known Issues & Tuning Guidance

> Read this before starting any training work. Each item below has caused real debugging time.

---

## Critical Issues

### ⚠️ ctrl[] and qpos[] have different orderings

`ctrl[]` = `[L-arm, R-arm, L-hand, R-hand]`, but `qpos[]` = `[L-arm, L-hand, R-arm, R-hand]`.

**`ctrl[5]` is NOT `qpos[5]`.** See [ACTUATOR_MAP.md](ACTUATOR_MAP.md#ctrl-vs-qpos-ordering) for the full mapping and safe API usage.

### ⚠️ Hand actuators have NO force limits

All 44 hand actuators (ctrl 10–53) have **no `forcerange`**. In contact-rich scenarios, they can output unlimited torque. If training involves grasping or collision, consider adding force limits:

```xml
<!-- Example: add to each hand actuator -->
<position name="F2-L-MCP1_ctrl" ... forcelimited="true" forcerange="-5 5" />
```

### ⚠️ URDF integration model is NOT validated

The integrated URDF (`bimanualHand/openarm_hand_bimanual.urdf`) has **not** been tested with `check_urdf`, PyBullet, Isaac Sim, or Gazebo. If you need URDF, validate it first. Standalone hand URDFs (`Hand/Hand_{left,right}/URDF_{L,R}.urdf`) have passed `check_urdf`.

---

## URDF vs MJCF Differences

If you need to use URDF instead of MJCF, be aware of these gaps:

| Feature | MJCF | URDF |
|---------|:----:|:----:|
| Actuators (kp/kv/ctrl) | ✅ 54 position actuators | ❌ None |
| Contact exclusions | ✅ 60+ exclude pairs | ❌ None |
| Home keyframe | ✅ Defined | ❌ No mechanism |
| Collision material (solref/solimp) | ✅ Tuned | ❌ None |
| Visual mesh grouping | ✅ Separated (group 2) | ❌ May participate in collision |

You must implement equivalent contact filtering and control on the simulator side if using URDF.

---

## Collision Mesh Gotchas

- All collision meshes are **single convex hulls** per link (not decomposed).
- In concave areas (e.g., finger inner surfaces), the collision hull is **larger than the visual mesh**, causing apparent "floating" contacts.
- Collision STL files for hands end with `_collision.STL`, stored in separate directories.
- The body pedestal (`openarm_body_link0`) has collision excluded with **all** hand links on both sides. If your task requires the arm base to interact with the hands, you need to selectively re-enable those pairs.

---

## Missing Features (by design)

These are intentionally absent. Add them if your training requires it:

- **Sensors**: No joint torque, contact force, or F/T sensors
- **Cameras**: No cameras defined
- **Tendons / couplings**: No tendon coupling between joints
- **Equality constraints**: None

---

## OpenArm Body Z-Stretch

The pedestal body (`openarm_body_link0`) has its Z-axis scaled by 1.5× to raise the arm mounting height from 0.698m to 1.047m. Mass is scaled proportionally (13.89 → 20.84 kg). This is intentional.

To revert: change mesh scale from `0.001 0.001 0.0015` back to `0.001 0.001 0.001` and adjust the arm mount Z position from 1.047 to 0.698.
