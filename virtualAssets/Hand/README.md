# Hand Models

> **Reference only.** For training and simulation, use the integrated model in [`bimanualHand/`](../bimanualHand/) instead. These standalone hand models are provided for debugging and inspection.

This package contains left and right hand robot models for URDF-based tooling and MuJoCo training.

## Contents

```text
Hand_left/
  URDF_L.urdf
  MJCF_L.xml
  meshes/
    visual/
    collision/

Hand_right/
  URDF_R.urdf
  MJCF_R.xml
  meshes/
    visual/
    collision/
```

- `URDF_L.urdf`, `URDF_R.urdf`: URDF robot descriptions for general robotics tooling.
- `MJCF_L.xml`, `MJCF_R.xml`: MuJoCo models intended for simulation and training.
- `meshes/visual/`: high-detail STL meshes for rendering.
- `meshes/collision/`: simplified convex-hull STL meshes for collision.

## Recommended Use

For MuJoCo training, use the MJCF files:

```bash
simulate Hand_left/MJCF_L.xml
simulate Hand_right/MJCF_R.xml
```

For URDF-based tools, use:

```bash
check_urdf Hand_left/URDF_L.urdf
check_urdf Hand_right/URDF_R.urdf
```

## Model Notes

- Each hand has 24 bodies/links and 22 active revolute joints.
- No mimic joints, passive joints, transmissions, or tendon couplings are defined.
- Visual and collision meshes are intentionally separate.
- Collision meshes are single convex hulls per link.
- MJCF visual geoms use `group="1"` and do not participate in contact.
- MJCF collision geoms use `group="3"` and participate in contact.
- MJCF position actuators include `kp`, `kv`, and `inheritrange`; actuator `forcerange` is not set.

## Contact Filtering

The MJCF files exclude contacts between adjacent or initially overlapping internal structures:

- Wrist/palm near-neighbor contacts.
- Palm to finger-base near-neighbor contacts.
- Adjacent links within each finger.
- Initial static overlaps between adjacent MCP1 collision hulls.

Initial MJCF contact count is expected to be zero at the default pose.

## Physical Parameters

Joint effort and velocity limits are defined in the URDF. The MJCF includes MuJoCo-specific joint defaults and position actuator gains. These actuator gains and physical parameters are coarse reference values only, and should be calibrated against real hardware data or training performance before being treated as final.

## Validation

The package was validated with:

```bash
check_urdf Hand_left/URDF_L.urdf
check_urdf Hand_right/URDF_R.urdf

compile Hand_left/MJCF_L.xml /tmp/Hand_L_compiled.xml
compile Hand_right/MJCF_R.xml /tmp/Hand_R_compiled.xml
```

MuJoCo initial-state contact checks were also run for both MJCF models, with `data.ncon == 0`.

## Assumptions

- Mesh paths are relative to each hand directory.
- STL units are assumed to match the robot model scale.
- Convex hull collision meshes are simplified approximations and may be larger than the visual surface in concave areas.
- The MJCF files are the recommended source for MuJoCo simulation; the URDF files are kept for compatibility with URDF workflows.
