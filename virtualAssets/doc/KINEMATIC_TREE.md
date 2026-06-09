# Kinematic Tree & Contact Exclusions

## Body Hierarchy

The model has 62 bodies. The tree structure is:

```
worldbody
└── openarm_body_link0                    ← Pedestal (fixed, Z-stretched 1.5×)
    ├── openarm_left_base_link
    │   └── openarm_left_link1            ← joint1 (shoulder yaw)
    │       └── openarm_left_link2        ← joint2 (shoulder pitch)
    │           └── openarm_left_link3    ← joint3 (upper arm rotation)
    │               └── openarm_left_link4  ← joint4 (elbow)
    │                   └── openarm_left_link5  ← joint5 (forearm rotation)
    │                       └── Hand_L_base_link
    │                           └── HOUSING-L
    │                               └── WRIST-PALM-L      ← wrist rotation
    │                                   └── PALM-L         ← palm pitch
    │                                       ├── F1-L-MCP2 → F1-L-MCP1 → F1-L-PIP → F1-L-DIP  (thumb)
    │                                       ├── F2-L-MCP2 → F2-L-MCP1 → F2-L-PIP → F2-L-DIP  (index)
    │                                       ├── F3-L-MCP2 → F3-L-MCP1 → F3-L-PIP → F3-L-DIP  (middle)
    │                                       ├── F4-L-MCP2 → F4-L-MCP1 → F4-L-PIP → F4-L-DIP  (ring)
    │                                       └── F5-L-MCP2 → F5-L-MCP1 → F5-L-PIP → F5-L-DIP  (pinky)
    │
    └── openarm_right_base_link
        └── openarm_right_link1           ← joint1
            └── openarm_right_link2       ← joint2
                └── openarm_right_link3   ← joint3
                    └── openarm_right_link4 ← joint4
                        └── openarm_right_link5 ← joint5
                            └── Hand_R_base_link
                                └── HOUSING-R
                                    └── WRIST-PALM-R
                                        └── PALM-R
                                            ├── F1-R  (thumb)
                                            ├── F2-R  (index)
                                            ├── F3-R  (middle)
                                            ├── F4-R  (ring)
                                            └── F5-R  (pinky)
```

Key observations:
- Each arm is a **serial chain** of 5 links ending at the hand housing
- Each hand has **5 parallel finger chains** branching from `PALM`
- Each finger is a **serial chain** of 4 links (MCP2 → MCP1 → PIP → DIP)
- Thumb (F1) has a different joint axis configuration from F2–F5

---

## Joint Naming Convention

### Arm Joints

```
openarm_{left|right}_joint{N}    N = 1..5
```

| Joint | Role | Axis (left) |
|-------|------|-------------|
| joint1 | Shoulder yaw | Y |
| joint2 | Shoulder pitch | −X |
| joint3 | Upper arm rotation | −Z |
| joint4 | Elbow flex | −Y |
| joint5 | Forearm rotation | −Z |

Right arm axes are mirrored where appropriate.

### Hand Joints

```
{component}-{L|R}              for WRIST-PALM, PALM
F{digit}-{L|R}-{segment}      for fingers
```

| Segment | Role | Typical Axis |
|---------|------|--------------|
| WRIST-PALM | Wrist rotation | Z (L) / −Z (R) |
| PALM | Palm pitch | −X (L) / X (R) |
| MCP2 | Finger ab/adduction | varies |
| MCP1 | Finger flexion | ±X |
| PIP | Proximal interphalangeal | X |
| DIP | Distal interphalangeal | X |

---

## Contact Exclusions (MJCF only)

The MJCF defines **62 contact exclude pairs** in the `<contact>` section. These are **not present in the URDF**.

### Exclusion Categories

| Category | Count | Pairs |
|----------|:-----:|-------|
| Hand internal (per hand) | 25×2=50 | HOUSING↔WRIST-PALM, WRIST-PALM↔PALM, HOUSING↔PALM, PALM↔each MCP2, PALM↔each MCP1, within-finger adjacents, adjacent-finger MCP1s |
| Pedestal ↔ all hand links | 2×(1+24)=~50* | `openarm_body_link0` excluded with every hand body |

*Some pairs overlap with the above. Total unique `<exclude>` entries: **62 per hand side + body pairs**.

### Within Each Finger

```
PALM ↔ MCP2    (palm to finger base)
PALM ↔ MCP1    (palm to finger base)
MCP2 ↔ MCP1    (adjacent links)
MCP1 ↔ PIP     (adjacent links)
PIP  ↔ DIP     (adjacent links)
```

### Between Adjacent Fingers

```
F2-MCP1 ↔ F3-MCP1    (index-middle)
F3-MCP1 ↔ F4-MCP1    (middle-ring)
```

> **Note**: F4-MCP1 ↔ F5-MCP1 and F1 ↔ F2 are NOT excluded. These finger pairs can collide.

### Pedestal Exclusions

`openarm_body_link0` is excluded from colliding with **all** hand bodies on both sides. If your task requires base-hand interaction (unlikely), you'll need to remove some of these.

---

## Geom Groups

| Group | Type | Contact | Count |
|:-----:|------|:-------:|:-----:|
| 1 | Hand visual (STL) | No (`contype=0`) | — |
| 2 | Arm visual (OBJ) | No (`contype=0`) | — |
| 3 | All collision (STL) | Yes (`contype=1`) | — |

Toggle visibility in `simulate` using the group checkboxes in the Rendering section.
