# OpenArm + Hand Bimanual Dexterous Hand

Bimanual robot model: two 5-DoF OpenArm arms + two 22-DoF dexterous hands. 54 total DoF, ready for MuJoCo simulation and RL training.

## Quick Start

```bash
# Visualize in MuJoCo
cd bimanualHand
simulate openarm_hand_bimanual.xml
```

```python
import mujoco

model = mujoco.MjModel.from_xml_path("bimanualHand/openarm_hand_bimanual.xml")
data  = mujoco.MjData(model)

# Verify model dimensions
assert model.nq == 54  # generalized coords
assert model.nv == 54  # generalized velocities
assert model.nu == 54  # actuators (= ctrl vector length)

# Send a control command (all position actuators)
data.ctrl[:] = 0.0
mujoco.mj_step(model, data)
```

## Model at a Glance

| Property | Value |
|----------|-------|
| Configuration | Bimanual: 2 × 5-DoF arm + 2 × 22-DoF hand |
| Total DoF | 54 (all hinge/revolute) |
| Bodies | 62 |
| Geoms | 133 (visual + collision) |
| Actuators | 54 position actuators |
| Recommended format | **MJCF** (`openarm_hand_bimanual.xml`) |
| URDF available | Yes, but limited — see [Known Issues](doc/KNOWN_ISSUES.md) |
| MuJoCo compatibility | ≥ 2.3.x |

## Repository Layout

| Directory | Role | When to Use |
|-----------|------|-------------|
| **`bimanualHand/`** | **Integrated model (primary deliverable)** | Training, simulation, deployment |
| `Hand/` | Standalone left/right hand reference | Debug hand-only issues |
| `openarm/` | Standalone dual-arm reference | Debug arm-only issues |

> Use `bimanualHand/` for all training work. The other directories are upstream references only.

## Documentation

| Document | Content |
|----------|---------|
| **[doc/ACTUATOR_MAP.md](doc/ACTUATOR_MAP.md)** | `ctrl[]` and `qpos[]` index tables, naming convention, code examples |
| **[doc/KNOWN_ISSUES.md](doc/KNOWN_ISSUES.md)** | Uncalibrated parameters, URDF limitations, collision gotchas, tuning guidance |
| **[doc/KINEMATIC_TREE.md](doc/KINEMATIC_TREE.md)** | Body hierarchy, joint naming rules, contact exclusion summary |

## Upstream Sources

OpenArm model is derived from:
- [enactic/openarm_description](https://github.com/enactic/openarm_description)
- [enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco)

See upstream repositories for original license information.
