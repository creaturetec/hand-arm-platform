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
| link3 | 0.850916861 | 1.09286; J4 motor replacement correction |
| link4 | 0.590845933 | 0.715853; J5 motor replacement correction |
| link5 own mass | 0.580396803 | 0.0445; measured flange mass |
| HOUSING (forearm hand housing) | 0.5561357 | 1.8; user-specified per side |

`openarm_body_link0` is the pedestal. `openarm_{left,right}_base_link` are fixed shoulder mounts. All three are welded to world, so their masses do not enter the moving-joint mass matrix or gravity loads in this fixed-base model. `link1` and the downstream links do.

For `base_link`, `link1`, and `link2`, mass, center of mass, and principal inertia are copied together from the local arm MJCF. The original link3/link4 inertias are the starting point for the DM4340P corrections below. Integrated MJCF stores their corrected full tensors, and URDF stores the same tensors at the same centers of mass. `I = R diag(Iprincipal) R^T` converts the source principal inertias into link-frame tensors.

**DM4340P hardware alignment (2026-10-15):** all six J3–J5 joints and position actuators use `motor_DM4340P` / `position_DM4340P`. The target hardware note specifies 0.381 kg per motor, diameter 57 mm and height 56.5 mm. The [manufacturer product page](https://www.damiaokeji.com/index.php?c=show&id=101) confirms 9 N·m rated and 27 N·m peak torque; the [older V1 manual](https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/) lists approximately 0.375 kg. The model adopts the hardware note's 0.381 kg reference value by user decision; no additional weighing correction is applied for this catalogue/version difference. The original assembly already uses a P motor at J3, so no additional J3 motor is counted.

J4's original DM4340 body is assigned to link3 and is replaced by a DM4340P surrogate (0.362 → 0.381 kg). J5's original DM4310 body is assigned to link4 (0.300 → 0.381 kg). Per side, link3 becomes 1.09286 kg and link4 becomes 0.715853 kg. Both arms together gain 0.200 kg. The flange remains a separate measured 0.0445 kg body. J3's already-P contribution remains part of the adopted source baseline; no additional mass adjustment is made to it.

Motor inertias are approximated as uniform cylinders, subtracting the old cylinder and adding the replacement at a common link origin, then shifting the resulting tensor back to the new combined COM. Structure inertia is otherwise retained; this is a differential approximation, not recovered CAD mass properties. The replacement assumes the existing output mounting planes stay fixed. Centers below are in the carrying link's local frame and are inferred from the reference geometry and envelope heights:

| Motor / carrying body | Old mass, radius, height (SI) | Old COM (m) | New COM (m) / cylinder axis |
|-----------------------|------------------------------|-------------|--------------------------------|
| J4 / link3 | 0.362 kg, 0.0285 m, 0.0533 m | (0, -0.00485, -0.15375) | (0, -0.00325, -0.15375), Y |
| J5 / link4 | 0.300 kg, 0.028 m, 0.046 m | (0, 0, -0.0725) | (0, 0, -0.06725), Z |

The new cylinder is 0.381 kg, radius 0.0285 m, height 0.0565 m in both cases. With `P(c) = (c·c)E - c c^T`, the calculation is `M' = M - m_old + m_new`, `c' = (M c - m_old c_old + m_new c_new) / M'`, and `I_COM' = I_COM + M P(c) - I_old - m_old P(c_old) + I_new + m_new P(c_new) - M' P(c')`. `scripts/check_model.py` checks this baseline independently against the model files. The OpenArm mounting transforms and meshes are adopted without extra installation compensation. In particular, the J5 mounting position, axis, and fixed-side allocation remain from OpenArm, while its selected DM4340P mass, dimensions, and inertia contribution are retained. Installation baseline and motor properties are separate choices; no additional fixed-side J5 mass is moved into link5.

All J3–J5 joint and actuator force caps are ±27 N·m, a peak cap rather than a continuous torque rating. URDF velocity limits are 10.471975511966 rad/s (100 rpm, the 48 V no-load envelope); MJCF position servos do not enforce that speed limit. Per-joint kp/kv remain unchanged. The P class reuses the previous 4340-family estimates: damping 0.9, frictionloss 0.1, armature 0.16; J5 therefore changes from frictionloss 0.04 and armature 0.01. These adopted loss/rotor reference parameters are distinct from rigid-body inertia; no further identification or correction is part of the current model policy.

**Joint5 rigid-body allocation (geometry checked on 2026-10-15):** the [official OpenArm 2.0 STEP assembly](https://drive.google.com/file/d/1aU-V3lt_aPrZoRM8FFx6dMQu2f28Ws0I/view) identifies `DM4310_simplified`, `J4_C`, `J4_D`, `J4_E`, and `J4-J5_cover_v1.0` as parts corresponding to the original link4 geometry. In particular, the J5 motor housing is on the link4 side of joint5. The local reference `link4_00.obj`, `link4_01.obj`, and `link4_02.obj` are byte-identical to the [official MuJoCo meshes](https://github.com/enactic/openarm_mujoco/tree/master/v2/assets/visual/arm) checked on that date. A joint number does not identify which rigid body carries its motor housing.

**Mass-property source and limits:** the [official nominal inertial configuration](https://github.com/enactic/openarm_description/blob/main/assets/robot/openarm_v2.0/config/arm/inertials/nominal.yaml) specifies a complete link4 mass of 0.6348534566833373 kg; the [URDF link emitter](https://github.com/enactic/openarm_description/blob/main/assets/robot/openarm_v2.0/urdf/utils/openarm_v20_utils.xacro) copies the configured mass and inertia without a per-part mass calculation. The local arm MJCF baseline rounds this mass to 0.634853 kg. The checked STEP and BOM do not provide the per-part mass/density breakdown needed to independently reproduce that total or separate motor rotor/stator masses. Subtracting the nominal 0.300 kg DM4310 motor weight gives a remainder of approximately 334.853 g; this is an arithmetic difference, not an official measurement of the structural parts. The geometry correspondence supports the rigid-body allocation, not a measured internal motor mass decomposition.

The integrated model starts from the original link4 inertia and applies the DM4310-to-DM4340P correction above. It replaces the removed downstream link5 assembly with the measured 0.0445 kg flange described below. This replaces a motor already represented in link4 rather than counting another complete motor in link5. The fixed hand housing separately contributes 1.8 kg to the joint5 output body, and the full moving hand remains in the dynamics.

**Hand housing mass:** the user specified 1.8 kg for each `HOUSING-L/R`, replacing 0.5561357 kg in the integrated MJCF and URDF. The original center of mass and geometry are retained, and all six components of the COM inertia tensor are multiplied by `1.8 / 0.5561357 = 3.23662012706611`. This assumes the normalized mass distribution is unchanged; it is not a fresh CAD mass-property calculation. The standalone left/right `Hand/` URDF and MJCF models use the same updated mass and inertia.

**Measured flange parameters (2026-10-15):** the user measured a mass of 44.5 g, radius of 24.08 mm, and thickness of 12.48 mm. Both sides now use `m = 0.0445 kg`, `r = 0.02408 m`, and `h = 0.01248 m`, replacing the provisional 0.05 kg, 18.6667 mm radius, and 6 mm thickness. Mass comes directly from the measurement, not from a material-density calculation. A uniform disk with the measured mass and dimensions is used only as an equivalent inertia approximation; holes, cavities, and fasteners are not represented separately in its mass distribution.

The equivalent disk is perpendicular to the joint5 axis (local Z), with its center of mass still assumed at the joint5 output origin. Its principal moments are `Ixx = Iyy = m*(3*r^2+h^2)/12 = 0.0000070283656 kg*m^2` and `Izz = m*r^2/2 = 0.0000129015824 kg*m^2`. Products of inertia remain zero. The two flanges add 0.089 kg total, separately from the hand mass. No new visual or collision geom is added.

**Adopted distribution and installation policy (2026-10-15):** motor mass/properties use the selected motor reference, with OpenArm installation geometry retained and no additional fit or COM correction. The flange uses its measured mass and dimensions with a uniform disk distribution; its COM is fixed at the J5 output origin, with no offset correction. These are deliberate model assumptions rather than pending measurement tasks. The integrated model retains its existing hand payload, 0.180 m hand mounting offset, and controller kp/kv; the Notion construction note describes only the arm subsystem and excludes both hands.

The [official motor table](https://docs.openarm.dev/hardware/openarm-2.0/motor/) lists 0.896 kg for DM8009P, 0.362 kg for DM4340, and 0.300 kg for DM4310. These are complete motor weights, not rigid-link masses. A 1.14 kg shoulder assembly is plausible with a large motor and its mounts; this comparison does not prove which motor components are assigned to each link.

## Gravity and Reset Behavior

Gravity is now `0 0 -9.81`, with the 1 ms timestep and `implicitfast` integration. All position-controller gains and collision settings are retained. The `home` keyframe includes matching controls, using the actuator-to-joint mapping rather than copying the differently ordered `qpos` array.

There is no automatic gravity compensation. Position-only control will settle with a small tracking error under load. Use `python scripts/check_model.py` to check inertia consistency, home control mapping, torque saturation, and short free-space simulations. These checks do not validate real-hardware accuracy or grasp/contact performance.

The DM4340P revision passed this script with MuJoCo 3.16.0: zero and home poses each simulated for 5 s without solver warnings, with maximum arm tracking errors of 0.029° and 1.484° respectively. The new arm force limits were exercised to saturation. These are numerical stability and consistency checks; mass distribution and mounting reuse remain the adopted modeling assumptions.

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
