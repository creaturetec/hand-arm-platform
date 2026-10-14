# Known Issues & Tuning Guidance

> Physical assumptions, simulator limitations, and checks for the integrated model.

---

## Critical Issues

### ⚠️ ctrl[] and qpos[] have different orderings

`ctrl[]` = `[L-arm, R-arm, L-hand, R-hand]`, but `qpos[]` = `[L-arm, L-hand, R-arm, R-hand]`.

**`ctrl[5]` is NOT `qpos[5]`.** See [ACTUATOR_MAP.md](ACTUATOR_MAP.md#ctrl-vs-qpos-ordering) for the full mapping and safe API usage.

### Hand torque limits need hardware calibration

All 44 integrated hand actuators (ctrl 10–53) now have symmetric `forcerange` values copied from the integrated URDF joint `effort` limits. The wrist limits are ±12 and ±10 N·m; finger limits are ±1.5 to ±6.5 N·m. These are existing model values, not confirmed continuous motor ratings. The standalone hand MJCF reference files still have their original unlimited actuators.

URDF velocity limits are not enforced by MJCF position actuators. Real-hardware control needs its own trajectory velocity, acceleration, and motor-temperature limits.

### ⚠️ URDF integration model is NOT validated

The integrated URDF (`bimanualHand/openarm_hand_bimanual.urdf`) has **not** been tested with `check_urdf`, PyBullet, Isaac Sim, or Gazebo. If you need URDF, validate it first. Standalone hand URDFs (`Hand/Hand_{left,right}/URDF_{L,R}.urdf`) have passed `check_urdf`.

The integrated URDF arm and flange inertias were synchronized with MJCF in the 2026-10-14 revision. The joint5 output link carries the new flange inertia; fixed hand children carry the hand payload separately.

## Arm Inertia Baseline and Hardware Assumptions

Reviewed on 2026-10-14 against the local `openarm/MJCF_bimanual.xml`, [official OpenArm v2 MJCF](https://github.com/enactic/openarm_mujoco/blob/master/v2/openarm_bimanual.xml), and [official v2 URDF](https://github.com/enactic/openarm_description/blob/main/assets/robot/openarm_v2.0/urdf/example/v2.urdf). The current official URDF has the heavier masses too; the local integrated model had inherited a different, lighter URDF parameter set. No measurement record for these individual link masses was located.

| Per-side body | Previous integrated mass (kg) | Current baseline (kg) |
|---------------|------------------------------:|----------------------:|
| base_link | 0.487784888 | 1.14323 |
| link1 | 0.561428544 | 1.14167 |
| link2 | 0.295592749 | 0.277509 |
| link3 | 0.850916861 | 1.07386 |
| link4 | 0.590845933 | 0.634853 |
| link5 own mass | 0.580396803 | 0.05; new steel flange estimate |
| HOUSING (forearm hand housing) | 0.5561357 | 1.8; user-specified per side |

`openarm_body_link0` is the pedestal. `openarm_{left,right}_base_link` are fixed shoulder mounts. All three are welded to world, so their masses do not enter the moving-joint mass matrix or gravity loads in this fixed-base model. `link1` and the downstream links do.

For `base_link` and `link1`–`link4`, mass, center of mass, and principal inertia are copied together from the local arm MJCF. MJCF uses the source `quat + diaginertia`; URDF stores the equivalent tensor `I = R diag(Iprincipal) R^T` at the same center of mass. `fullinertia` and `quat + diaginertia` encode the same physical quantity; neither is inherently more accurate.

**Joint5 assembly assumption:** the user confirmed that the motor housing is fixed inside link4 and only its output/flange and the hand rotate. Inspection of the original visual geometry places the joint5 housing on the link4 side; original link5 is a separate downstream assembly with the next joint's structure. The baseline therefore retains the complete original link4 inertia and removes the obsolete link5 assembly inertia. No extra nominal motor mass is added to link4. The new flange is modeled on link5 as described below. The fixed hand housing separately contributes 1.8 kg to the joint5 output body, and the full moving hand remains in the dynamics. This assignment is a simulation baseline inferred from the assembly, not a measured decomposition of motor rotor/stator masses.

**Hand housing mass:** the user specified 1.8 kg for each `HOUSING-L/R`, replacing 0.5561357 kg in the integrated MJCF and URDF. The original center of mass and geometry are retained, and all six components of the COM inertia tensor are multiplied by `1.8 / 0.5561357 = 3.23662012706611`. This assumes the normalized mass distribution is unchanged; it is not a fresh CAD mass-property calculation. The standalone left/right `Hand/` URDF and MJCF models use the same updated mass and inertia.

**Steel flange estimate:** the user identified steel/stainless steel and a radius one third smaller than the joint5 motor radius. The official DM4310 outer diameter is 56 mm, so the modeled flange radius is `(0.056 / 2) * (2 / 3) = 0.0186666667 m`. Use a provisional 6 mm thickness and 0.05 kg per side, replacing the earlier 1.2 kg estimate. A solid disk of these dimensions at 7900 kg/m^3 weighs about 51.9 g; holes and fasteners are not separately modeled. The density reference is the [Outokumpu Core 304 datasheet](https://www.outokumpu.com/-/media/files/products/core/outokumpu-core-range-datasheet.pdf). At the same diameter, 5 mm and 10 mm thick solid steel disks would weigh about 43 g and 86 g respectively. These estimates concern mass, not structural strength.

The flange is represented by a uniform solid disk perpendicular to the joint5 axis (local Z), with its center of mass assumed at the joint5 output origin. Its principal moments are `Ixx = Iyy = m*(3*r^2+h^2)/12 = 0.00000450555555555556 kg*m^2` and `Izz = m*r^2/2 = 0.00000871111111111111 kg*m^2`. The two flanges add 0.10 kg total, separately from the hand mass. No new visual or collision geom is added.

**Still to measure:** actual flange thickness, mass, center offset, and the distribution of components inside the heavier hand housing. If the flange is offset along the mounting segment, update its COM instead of moving the hand. The 0.180 m hand mounting offset is retained. Existing joint `armature` values are also uncalibrated and were retained, as were joint damping and controller kp/kv.

The [official motor table](https://docs.openarm.dev/hardware/openarm-2.0/motor/) lists 0.896 kg for DM8009P, 0.362 kg for DM4340, and 0.300 kg for DM4310. These are complete motor weights, not rigid-link masses. A 1.14 kg shoulder assembly is plausible with a large motor and its mounts; this comparison does not prove which motor components are assigned to each link.

## Gravity and Reset Behavior

Gravity is now `0 0 -9.81`, with the 1 ms timestep and `implicitfast` integration. All position-controller gains and collision settings are retained. The `home` keyframe includes matching controls, using the actuator-to-joint mapping rather than copying the differently ordered `qpos` array.

There is no automatic gravity compensation. Position-only control will settle with a small tracking error under load. Use `python scripts/check_model.py` to check inertia consistency, home control mapping, torque saturation, and short free-space simulations. These checks do not validate real-hardware accuracy or grasp/contact performance.

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
