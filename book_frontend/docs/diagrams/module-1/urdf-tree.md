# URDF Kinematic Tree Diagram Specification

**Purpose**: Illustrate the parent-child link hierarchy and joint connections for a robotic arm

**Diagram Type**: Tree diagram / Kinematic chain visualization

## Simple Arm Kinematic Tree

```
                    [World Frame]
                          │
                          │ (fixed)
                          ▼
                   ┌──────────────┐
                   │  base_link   │  ← Root link (fixed to world)
                   └──────┬───────┘
                          │
                  [shoulder_joint]  ← Revolute joint (yaw/Z-axis)
                    Type: revolute
                    Axis: [0, 0, 1] (Z)
                    Limits: -90° to +90°
                          │
                          ▼
                 ┌────────────────┐
                 │ upper_arm_link │  ← Child of shoulder_joint
                 └────────┬───────┘
                          │
                   [elbow_joint]   ← Revolute joint (pitch/Y-axis)
                    Type: revolute
                    Axis: [0, 1, 0] (Y)
                    Limits: 0° to 150°
                          │
                          ▼
                 ┌────────────────┐
                 │ forearm_link   │  ← Child of elbow_joint
                 └────────┬───────┘
                          │
                   [wrist_joint]   ← Revolute joint (roll/Z-axis)
                    Type: revolute
                    Axis: [0, 0, 1] (Z)
                    Limits: -90° to +90°
                          │
                          ▼
                 ┌────────────────┐
                 │  wrist_link    │  ← End effector mount
                 └────────────────┘
```

## Kinematic Chain Properties

### Links (Physical Segments)

| Link Name | Parent Joint | Length (m) | Mass (kg) | Purpose |
|-----------|--------------|------------|-----------|---------|
| **base_link** | (world) | 0.1 | 1.0 | Fixed base mounting |
| **upper_arm_link** | shoulder_joint | 0.3 | 0.5 | Upper arm segment |
| **forearm_link** | elbow_joint | 0.25 | 0.3 | Forearm segment |
| **wrist_link** | wrist_joint | 0.05 | 0.1 | End effector mount |

### Joints (Connections)

| Joint Name | Type | Parent Link | Child Link | Axis | Range (rad) | Range (deg) |
|------------|------|-------------|------------|------|-------------|-------------|
| **shoulder_joint** | revolute | base_link | upper_arm_link | Z (yaw) | -1.57 to +1.57 | -90° to +90° |
| **elbow_joint** | revolute | upper_arm_link | forearm_link | Y (pitch) | 0.0 to +2.618 | 0° to +150° |
| **wrist_joint** | revolute | forearm_link | wrist_link | Z (roll) | -1.57 to +1.57 | -90° to +90° |

## Coordinate Frame Transformations

### Transformation Chain (Base to Wrist)

```
T_world_to_wrist = T_world_to_base × T_base_to_upper_arm × T_upper_arm_to_forearm × T_forearm_to_wrist
```

**Breakdown**:

1. **T_world_to_base** (Fixed):
   - Translation: [0, 0, 0]
   - Rotation: Identity (no rotation)

2. **T_base_to_upper_arm** (Shoulder Joint):
   - Translation: [0, 0, 0.1] (joint origin)
   - Rotation: Rz(θ₁) where θ₁ = shoulder joint angle

3. **T_upper_arm_to_forearm** (Elbow Joint):
   - Translation: [0, 0, 0.3] (upper arm length)
   - Rotation: Ry(θ₂) where θ₂ = elbow joint angle

4. **T_forearm_to_wrist** (Wrist Joint):
   - Translation: [0, 0, 0.25] (forearm length)
   - Rotation: Rz(θ₃) where θ₃ = wrist joint angle

### Total Reach

**Maximum reach** (all joints straight):
- Base height: 0.1 m
- Upper arm: 0.3 m
- Forearm: 0.25 m
- Wrist offset: 0.05 m
- **Total**: 0.7 meters vertical reach

**Workspace**: Cylindrical volume (due to revolute joints)
- Radius: 0.55 m (horizontal reach when arm extended)
- Height: 0.7 m

## Parent-Child Relationships

### Link Hierarchy (Indented Tree)

```
base_link (root)
└── upper_arm_link
    └── forearm_link
        └── wrist_link
```

### Joint-Link Associations

```
[Joint] → Child Link (Parent Link)

shoulder_joint → upper_arm_link (base_link)
elbow_joint    → forearm_link (upper_arm_link)
wrist_joint    → wrist_link (forearm_link)
```

## Degrees of Freedom (DoF) Analysis

**Total DoF**: 3 (3 revolute joints)

| Joint | DoF Type | Motion Description |
|-------|----------|-------------------|
| Shoulder | 1 (revolute) | Yaw: Horizontal rotation (left/right) |
| Elbow | 1 (revolute) | Pitch: Vertical bending (up/down) |
| Wrist | 1 (revolute) | Roll: Twisting around forearm axis |

**Configuration Space**: 3-dimensional (θ₁, θ₂, θ₃)
- Each joint angle contributes 1 dimension
- Joint limits constrain the configuration space

**Task Space**: 3D position (x, y, z) of wrist_link
- Orientation is partially constrained (3 DoF → 6 DoF task space requires more joints)

## Forward Kinematics Example

**Given joint angles**: θ₁ = 0°, θ₂ = 90°, θ₃ = 0°

**Wrist position calculation**:
1. Start at base: (0, 0, 0.1)
2. Add upper arm (vertical): (0, 0, 0.1 + 0.3) = (0, 0, 0.4)
3. Add forearm (90° bent, now horizontal): (0.25, 0, 0.4)
4. Wrist offset: (0.25 + 0.05, 0, 0.4) = (0.3, 0, 0.4)

**Result**: Wrist at (0.3 m forward, 0 m side, 0.4 m up)

## TF Tree Representation

**ROS 2 TF (Transform) Tree**:

```
/world
  └─ /base_link
       └─ /upper_arm_link
            └─ /forearm_link
                 └─ /wrist_link
```

Each frame publishes its transform relative to its parent. ROS 2 `tf2` library automatically computes transforms between any two frames.

**Usage**:
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check transform from base to wrist
ros2 run tf2_ros tf2_echo base_link wrist_link
```

## Visual Layout (Simplified)

```
    🟦 base_link (gray cube, 0.1m)
     ↓
    ⚙️ shoulder_joint (yaw, Z-axis)
     ↓
    🟦 upper_arm_link (blue cylinder, 0.3m)
     ↓
    ⚙️ elbow_joint (pitch, Y-axis)
     ↓
    🟩 forearm_link (green cylinder, 0.25m)
     ↓
    ⚙️ wrist_joint (roll, Z-axis)
     ↓
    🔴 wrist_link (red sphere, end effector)
```

## Common Kinematic Patterns

### Serial Chain (This Arm)
- Links connected in sequence (no branching)
- Simple forward kinematics (multiply transforms sequentially)
- Inverse kinematics can be complex (multiple solutions)

### Parallel Mechanisms (Not This Arm)
- Multiple kinematic chains connecting same links
- Examples: Delta robot, Stewart platform
- More complex URDF with closed-loop constraints

### Branching Trees (Humanoid Robots)
- Torso as root, multiple arms and legs branch out
- Example tree: base → torso → [left_arm, right_arm, left_leg, right_leg]

## Usage in Book

- **Referenced in**: Chapter 5 (Core Concept 2: Links and Joints, Core Concept 3: Coordinate Frames)
- **Purpose**: Help learners trace parent-child relationships in URDF and understand kinematic chain structure
- **Learning Goal**: Learner can draw kinematic tree from URDF file and compute forward kinematics by hand

## Key Takeaways

1. **Links** are physical segments (mass, inertia, geometry)
2. **Joints** connect links and define motion (type, axis, limits)
3. **Parent-child hierarchy** forms a tree (root = base, leaves = end effectors)
4. **Coordinate frames** at each link origin, transforms propagate from root to leaves
5. **TF tree** in ROS 2 mirrors URDF structure for runtime transforms
