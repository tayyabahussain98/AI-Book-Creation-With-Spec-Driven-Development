---
sidebar_position: 5
title: Chapter 5 - URDF and Robot Description
---

# Chapter 5: URDF and Robot Description

## Introduction

In Chapters 1-4, you learned the software infrastructure for robotics: ROS 2 middleware, nodes, topics, packages, and AI integration. But how do you describe the **physical structure** of a robot—its links, joints, dimensions, and coordinate frames—in a way that simulation engines (Gazebo, Isaac Sim) and motion planning libraries (MoveIt2) can understand?

The answer is **URDF (Unified Robot Description Format)**, an XML-based standard for representing robot kinematics and dynamics. URDF files define:
- **Links**: Rigid body segments (arm segments, legs, torso)
- **Joints**: Connections between links with motion constraints (revolute, prismatic)
- **Coordinate frames**: Transformations between link origins
- **Visual and collision geometry**: Shapes for rendering and physics
- **Inertial properties**: Mass and inertia tensors for dynamics simulation

This chapter teaches you how to read, write, and understand URDF files. You'll learn to trace kinematic chains, identify joint types, and prepare robot descriptions for simulation. By the end, you'll be able to describe a humanoid robot's physical structure and visualize it in ROS 2 tools.

**Prerequisites**:
- Chapter 4 (ROS 2 packages, launch files)
- Basic understanding of 3D coordinate systems
- Familiarity with XML syntax (helpful but not required)

---

## Core Concept 1: URDF Purpose and Structure

### What is URDF?

**URDF (Unified Robot Description Format)** is an XML specification for robot models. It serves as the **single source of truth** for:
- **Kinematics**: Link lengths, joint types, motion ranges
- **Dynamics**: Mass, inertia, friction, damping
- **Visualization**: 3D shapes and colors for rendering
- **Collision**: Simplified geometry for physics simulation

URDF files are consumed by:
- **Simulators**: Gazebo, Isaac Sim, PyBullet
- **Visualization tools**: RViz2, URDF visualizer
- **Motion planners**: MoveIt2 (kinematic solvers, collision checking)
- **Controllers**: Joint trajectory controllers (need joint limits)

### XML Structure

URDF files follow a hierarchical XML format:

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Link definitions -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>

  <!-- Joint definitions -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="10" velocity="1" lower="-1.57" upper="1.57"/>
  </joint>

  <link name="link1">...</link>

  <!-- More joints and links -->
</robot>
```

**Key Elements**:
- **`<robot>`**: Root element (contains all links and joints)
- **`<link>`**: Physical segment with geometry and inertial properties
- **`<joint>`**: Connection between two links (parent → child)
- **`<origin>`**: Position (xyz) and orientation (rpy = roll, pitch, yaw) of joint relative to parent
- **`<axis>`**: Direction of joint motion (rotation axis or translation direction)

### URDF File Workflow

```
1. Design robot structure (sketch kinematic chain)
2. Write URDF file (links, joints, geometry)
3. Validate syntax: check_urdf my_robot.urdf
4. Visualize: ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
5. Use in simulation (Gazebo) or motion planning (MoveIt2)
```

---

## Core Concept 2: Links and Joints

### Links: Physical Segments

A **link** represents a rigid body segment of the robot. Each link has:

#### 1. Visual Geometry (Rendering)

Defines how the link appears in visualization tools (RViz2):

```xml
<visual>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="0.03" length="0.3"/>
  </geometry>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
</visual>
```

**Geometry Primitives**:
- **Box**: `<box size="x y z"/>`
- **Cylinder**: `<cylinder radius="r" length="l"/>`
- **Sphere**: `<sphere radius="r"/>`
- **Mesh**: `<mesh filename="package://pkg/meshes/arm.stl"/>`

#### 2. Collision Geometry (Physics)

Simplified geometry for collision detection (faster than visual):

```xml
<collision>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
  <geometry>
    <cylinder radius="0.03" length="0.3"/>
  </geometry>
</collision>
```

**Best Practice**: Use primitive shapes (box, cylinder, sphere) for collision geometry even if visual uses complex meshes.

#### 3. Inertial Properties (Dynamics)

Mass and inertia tensor for physics simulation:

```xml
<inertial>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>
  <mass value="0.5"/>
  <inertia ixx="0.00375" ixy="0.0" ixz="0.0"
           iyy="0.00375" iyz="0.0"
           izz="0.000225"/>
</inertial>
```

**Inertia Tensor**: 3×3 symmetric matrix describing resistance to rotational acceleration:
```
I = [ixx  ixy  ixz]
    [ixy  iyy  iyz]
    [ixz  iyz  izz]
```

**Formulas for Primitives**:
- **Solid cylinder** (radius r, length l, mass m):
  - I_xx = I_yy = (1/12) m (3r² + l²)
  - I_zz = (1/2) m r²

### Joints: Connections and Motion

A **joint** connects two links and defines their relative motion. Joints specify:

#### Joint Attributes

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_link"/>
  <child link="forearm_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="20.0" velocity="1.57" lower="0.0" upper="2.618"/>
  <dynamics damping="0.1" friction="0.01"/>
</joint>
```

**Fields**:
- **`name`**: Unique identifier
- **`type`**: Joint type (see Core Concept 3)
- **`parent`**: Fixed link (reference frame)
- **`child`**: Moving link (changes pose based on joint state)
- **`origin`**: Position (xyz in meters) and orientation (rpy in radians) of joint relative to parent frame
- **`axis`**: Direction vector for joint motion (e.g., [0, 0, 1] = Z-axis rotation)
- **`limit`**: Motion constraints:
  - `effort`: Max torque (Nm) or force (N)
  - `velocity`: Max speed (rad/s or m/s)
  - `lower`, `upper`: Joint range (radians or meters)
- **`dynamics`**: Simulation parameters (damping, friction)

#### Parent-Child Relationship

```
  parent_link (fixed)
        │
        │ [joint origin: xyz, rpy]
        ▼
    ┌───────┐
    │ Joint │ ← Motion constraint (revolute, prismatic, etc.)
    └───┬───┘
        │ [joint state: θ or d]
        ▼
  child_link (moves with joint)
```

**Key Insight**: The child link's pose is always defined relative to the parent link's frame. Changing the joint value (angle or displacement) updates the child's transform.

---

## Core Concept 3: Joint Types and Motion Constraints

See the [Joint Types Diagram](/docs/diagrams/module-1/joint-types.md) for complete visual reference.

### 1. Revolute Joint (Bounded Rotation)

**Motion**: Rotation around axis within angle limits

**Example**: Elbow joint (bends 0° to 150°)

```xml
<joint name="elbow_joint" type="revolute">
  <axis xyz="0 1 0"/>
  <limit effort="20.0" velocity="1.57" lower="0.0" upper="2.618"/>
</joint>
```

**DoF**: 1 (angle θ)

### 2. Continuous Joint (Unbounded Rotation)

**Motion**: Rotation around axis with no limits (can spin forever)

**Example**: Wheel axle

```xml
<joint name="wheel_joint" type="continuous">
  <axis xyz="0 1 0"/>
</joint>
```

**DoF**: 1 (angle θ, unbounded)

### 3. Prismatic Joint (Linear Translation)

**Motion**: Sliding along axis within distance limits

**Example**: Gripper finger (opens/closes linearly)

```xml
<joint name="gripper_joint" type="prismatic">
  <axis xyz="0 1 0"/>
  <limit effort="10.0" velocity="0.5" lower="0.0" upper="0.1"/>
</joint>
```

**DoF**: 1 (displacement d)

### 4. Fixed Joint (No Motion)

**Motion**: None (rigidly connects links)

**Example**: Camera mount

```xml
<joint name="camera_mount" type="fixed">
  <origin xyz="0.1 0 0.2" rpy="0 0.2 0"/>
</joint>
```

**DoF**: 0

### 5. Planar Joint (2D Motion)

**Motion**: Translation in X and Y, rotation around Z (3 DoF in a plane)

**Example**: Air hockey puck on table

**DoF**: 3 (x, y, θ)

### 6. Floating Joint (6 DoF)

**Motion**: Full 3D position and orientation (unconstrained)

**Example**: Drone in flight, free-floating humanoid

**DoF**: 6 (x, y, z, roll, pitch, yaw)

### Joint Type Selection Guide

| Use Case | Joint Type |
|----------|------------|
| Arm/leg joints (limited range) | **Revolute** |
| Wheels, propellers | **Continuous** |
| Gripper fingers, linear actuators | **Prismatic** |
| Sensor mounts, structural connections | **Fixed** |
| Mobile robot base (simplified) | **Planar** |
| Flying robots, underwater vehicles | **Floating** |

---

## Core Concept 4: Coordinate Frames and Transformations

### ROS 2 TF (Transform) Tree

Every link in URDF has an associated **coordinate frame** at its origin. ROS 2's `tf2` library maintains a **transform tree** mirroring the URDF structure:

```
/world
  └─ /base_link
       └─ /upper_arm_link
            └─ /forearm_link
                 └─ /wrist_link
```

Each frame publishes its transform relative to its parent. The `tf2` library can compute transforms between **any two frames** automatically.

**Example Query**: "What is the position of the wrist relative to the base?"
```bash
ros2 run tf2_ros tf2_echo base_link wrist_link
```

Output:
```
Translation: [0.55, 0.0, 0.4]
Rotation: in Quaternion [0.0, 0.0, 0.0, 1.0]
```

### Forward Kinematics: Computing End Effector Pose

**Forward kinematics** computes the end effector (e.g., wrist) pose given joint angles.

**Transformation Chain**:
```
T_base_to_wrist = T_base_to_upper_arm × T_upper_arm_to_forearm × T_forearm_to_wrist
```

Each transformation includes:
1. **Translation**: Joint origin (xyz from URDF)
2. **Rotation**: Joint angle (revolute) or identity (fixed)

**Example** (simple_arm.urdf with θ_shoulder = 0°, θ_elbow = 90°, θ_wrist = 0°):

1. **Base to upper arm** (shoulder joint at [0, 0, 0.1]):
   - Translation: [0, 0, 0.1]
   - Rotation: R_z(0°) = Identity

2. **Upper arm to forearm** (elbow joint at [0, 0, 0.3], upper arm length):
   - Translation: [0, 0, 0.3]
   - Rotation: R_y(90°) = Pitch 90° (arm now horizontal)

3. **Forearm to wrist** (wrist joint at [0, 0, 0.25], forearm length):
   - Translation: [0, 0, 0.25] (now in horizontal direction due to 90° pitch)
   - Rotation: R_z(0°) = Identity

**Result**: Wrist at approximately [0.25 m forward, 0 m side, 0.4 m up] relative to base.

### Homogeneous Transformation Matrices

Transformations are represented as 4×4 matrices:

```
T = [R | t]   = [r11 r12 r13 tx]
    [0 | 1]     [r21 r22 r23 ty]
                [r31 r32 r33 tz]
                [0   0   0   1 ]
```

Where:
- **R**: 3×3 rotation matrix
- **t**: 3×1 translation vector

**Composition**: T_total = T_1 × T_2 × T_3 (multiply matrices left to right)

### Visualizing TF Tree

```bash
# Generate TF tree graph
ros2 run tf2_tools view_frames

# Output: frames.pdf (visual graph of all coordinate frames)
```

See the [URDF Tree Diagram](/docs/diagrams/module-1/urdf-tree.md) for kinematic chain visualization.

---

## Core Concept 5: Visual vs Collision Geometry

URDF distinguishes between **visual** and **collision** geometry for performance:

### Visual Geometry (High Detail)

**Purpose**: Rendering in visualization tools (RViz2, Gazebo GUI)

**Characteristics**:
- Can be complex (high-poly meshes, textures)
- Used for appearance only (not physics)
- Loaded from `.stl`, `.dae`, `.obj` files

**Example** (humanoid head with detailed face mesh):
```xml
<visual>
  <geometry>
    <mesh filename="package://humanoid_description/meshes/head.stl" scale="1.0 1.0 1.0"/>
  </geometry>
  <material name="skin">
    <color rgba="0.9 0.7 0.6 1.0"/>
  </material>
</visual>
```

### Collision Geometry (Simplified)

**Purpose**: Physics simulation (collision detection, contact forces)

**Characteristics**:
- Should be **simple** (primitives or low-poly meshes)
- Faster collision checks (critical for real-time simulation)
- Often different from visual geometry

**Example** (same head approximated as sphere for collision):
```xml
<collision>
  <geometry>
    <sphere radius="0.12"/>
  </geometry>
</collision>
```

### Why Separate Geometries?

**Performance**: Collision detection runs at high frequency (100-1000 Hz in physics engines). Complex meshes slow down simulation.

**Accuracy Trade-off**:
- **Visual**: High detail for realism
- **Collision**: Simplified for speed

**Best Practice**:
- **Visual**: Use detailed meshes for visual appeal
- **Collision**: Use primitives (box, cylinder, sphere) or convex hulls
- **For simple robots**: Visual and collision can be identical (e.g., cylinder arm segments)

### Convex Hulls

For complex shapes, use **convex decomposition**:

```xml
<collision>
  <geometry>
    <mesh filename="package://pkg/meshes/hand_collision.stl"/>
  </geometry>
</collision>
```

**Convex hull**: Smallest convex shape enclosing the mesh (faster collision than concave meshes).

---

## Hands-On Example: URDF File

Let's examine a complete 3-joint robotic arm URDF.

**File**: `simple_arm.urdf` (3 DoF arm: shoulder yaw, elbow pitch, wrist roll)

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- ========================================
       Link 1: Base Link (Fixed to World)
       ======================================== -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001"/>
    </inertial>
  </link>


  <!-- ========================================
       Joint 1: Shoulder Joint (Yaw/Rotation)
       ======================================== -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="30.0" velocity="1.57" lower="-1.57" upper="1.57"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>


  <!-- ========================================
       Link 2: Upper Arm Link (Shoulder to Elbow)
       ======================================== -->
  <link name="upper_arm_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.3"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="0.5"/>
      <inertia ixx="0.00375" ixy="0.0" ixz="0.0"
               iyy="0.00375" iyz="0.0"
               izz="0.000225"/>
    </inertial>
  </link>


  <!-- ========================================
       Joint 2: Elbow Joint (Pitch/Bending)
       ======================================== -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="20.0" velocity="1.57" lower="0.0" upper="2.618"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>


  <!-- ========================================
       Link 3: Forearm Link (Elbow to Wrist)
       ======================================== -->
  <link name="forearm_link">
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <material name="green">
        <color rgba="0.0 0.8 0.0 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <mass value="0.3"/>
      <inertia ixx="0.0015625" ixy="0.0" ixz="0.0"
               iyy="0.0015625" iyz="0.0"
               izz="0.00009375"/>
    </inertial>
  </link>


  <!-- ========================================
       Joint 3: Wrist Joint (Roll/Rotation)
       ======================================== -->
  <joint name="wrist_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="10.0" velocity="1.57" lower="-1.57" upper="1.57"/>
    <dynamics damping="0.05" friction="0.01"/>
  </joint>


  <!-- ========================================
       Link 4: Wrist Link (End Effector Mount)
       ======================================== -->
  <link name="wrist_link">
    <visual>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.0 0.0 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.000036" ixy="0.0" ixz="0.0"
               iyy="0.000036" iyz="0.0"
               izz="0.000036"/>
    </inertial>
  </link>

</robot>
```

### Kinematic Structure

**Chain**: base_link → shoulder_joint → upper_arm_link → elbow_joint → forearm_link → wrist_joint → wrist_link

**DoF**: 3 (shoulder yaw, elbow pitch, wrist roll)

**Total Reach**: 0.7 meters (0.1 base + 0.3 upper arm + 0.25 forearm + 0.05 wrist)

### Key URDF Elements Explained

1. **Link Geometry**:
   - Base: Gray box (0.1 × 0.1 × 0.1 m)
   - Upper arm: Blue cylinder (radius 0.03 m, length 0.3 m)
   - Forearm: Green cylinder (radius 0.025 m, length 0.25 m)
   - Wrist: Red sphere (radius 0.03 m)

2. **Joint Configuration**:
   - Shoulder: Z-axis rotation (yaw), range -90° to +90°
   - Elbow: Y-axis rotation (pitch), range 0° to 150°
   - Wrist: Z-axis rotation (roll), range -90° to +90°

3. **Inertial Properties**:
   - Mass decreases from base (1.0 kg) to wrist (0.1 kg)
   - Inertia tensors computed for cylinders and sphere

### Visualizing the URDF

```bash
# Validate URDF syntax
check_urdf simple_arm.urdf

# Visualize in RViz2 (requires urdf_tutorial package)
ros2 launch urdf_tutorial display.launch.py model:=simple_arm.urdf

# Generate TF tree graph
ros2 run tf2_tools view_frames
```

**Expected Output**:
- RViz2 window with interactive joint sliders (move joints to see arm motion)
- TF tree: base_link → upper_arm_link → forearm_link → wrist_link

---

## Diagrams

### URDF Kinematic Tree

See the [URDF Tree Diagram](/docs/diagrams/module-1/urdf-tree.md) for:
- Complete parent-child link hierarchy
- Joint types and motion ranges
- Coordinate frame transformations
- Forward kinematics example

**Key Takeaway**: URDF defines a tree structure. Each joint has exactly one parent and one child. Coordinate frames propagate from root (base) to leaves (end effectors).

### Joint Types Reference

See the [Joint Types Diagram](/docs/diagrams/module-1/joint-types.md) for:
- Visual representations of all 6 joint types
- Motion characteristics and DoF
- URDF syntax for each type
- Use case recommendations

**Key Takeaway**: Choose joint type based on motion requirements. Revolute (bounded rotation) and prismatic (linear translation) are most common in robot arms. Fixed joints connect rigid structures (sensor mounts).

---

## Summary

This chapter introduced URDF for robot description and simulation:

1. **URDF Purpose**: XML-based standard for robot kinematics, dynamics, and geometry. Single source of truth for simulators (Gazebo, Isaac Sim), visualization (RViz2), and motion planning (MoveIt2).

2. **Links and Joints**: Links are rigid body segments with visual/collision/inertial properties. Joints connect links and define motion constraints (type, axis, limits). Parent-child relationships form a kinematic tree.

3. **Joint Types**: Six types available (revolute, continuous, prismatic, fixed, planar, floating). Revolute (bounded rotation) and prismatic (linear translation) are most common. Fixed joints create rigid connections.

4. **Coordinate Frames**: Each link has a coordinate frame at its origin. ROS 2 `tf2` library maintains transform tree mirroring URDF structure. Forward kinematics computes end effector pose by chaining transformations.

5. **Visual vs Collision**: Visual geometry (high detail) for rendering, collision geometry (simplified) for physics. Use primitives (box, cylinder, sphere) for collision to optimize performance.

6. **Practical Workflow**: Write URDF → Validate (`check_urdf`) → Visualize (RViz2) → Use in simulation or motion planning. URDF enables accurate robot models in virtual environments before physical testing.

---

## Self-Assessment Checklist

After completing this chapter, you should be able to:

- [ ] **I can explain** the purpose of URDF and why robots need standardized description formats for simulation and planning.
- [ ] **I can identify** links and joints in a URDF file and trace the parent-child hierarchy to construct a kinematic tree.
- [ ] **I can choose** appropriate joint types (revolute, prismatic, fixed, continuous) based on motion requirements for a given robot design.
- [ ] **I can read** URDF joint definitions and extract key information: axis direction, joint limits (lower/upper), max effort, and velocity.
- [ ] **I can trace** coordinate frame transformations from base to end effector using joint origins (xyz, rpy) and understand how joint angles affect transforms.
- [ ] **I can distinguish** between visual and collision geometry in URDF and explain why simplified collision shapes improve simulation performance.

**Next Steps**: With robot description skills in place, you're ready to simulate humanoid robots in Gazebo or Isaac Sim (Module 2) and implement motion controllers (Module 3). URDF is the foundation for all physical robot modeling in ROS 2.