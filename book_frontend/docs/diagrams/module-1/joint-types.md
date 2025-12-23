# Joint Types Diagram Specification

**Purpose**: Illustrate the different joint types available in URDF and their motion characteristics

**Diagram Type**: Component diagram / Motion illustration

## ROS 2 URDF Joint Types

URDF supports 6 joint types, each with different motion capabilities:

### 1. Revolute Joint (Rotational with Limits)

```
    Parent Link
         │
         │ Axis of rotation
    ┌────┴────┐
    │  Joint  │  ← Revolute joint
    └────┬────┘
         │ θ (joint angle)
         │ Range: [lower, upper]
         ▼
    Child Link (rotates)
```

**Properties**:
- **Motion**: Rotation around a single axis
- **Limits**: Required (lower and upper angle in radians)
- **Axis**: Defined by 3D vector (e.g., [0, 0, 1] for Z-axis)
- **Example**: Elbow joint (bends 0° to 150°)

**URDF Syntax**:
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_link"/>
  <child link="forearm_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="20.0" velocity="1.57" lower="0.0" upper="2.618"/>
</joint>
```

**Use Cases**:
- Robot arm joints (shoulder, elbow, wrist)
- Humanoid leg joints (hip, knee, ankle)
- Gripper fingers
- Wheels with limited rotation (steering)

**Visual Representation**:
```
    ╭─────╮
    │  ⟲  │  ← Rotation within limits
    ╰─────╯
  [θ_min, θ_max]
```

---

### 2. Continuous Joint (Unlimited Rotation)

```
    Parent Link
         │
         │ Axis of rotation
    ┌────┴────┐
    │  Joint  │  ← Continuous joint
    └────┬────┘
         │ θ (no limits)
         │ Range: (-∞, +∞)
         ▼
    Child Link (rotates infinitely)
```

**Properties**:
- **Motion**: Rotation around a single axis (no limits)
- **Limits**: Not required (can rotate indefinitely)
- **Axis**: Defined by 3D vector
- **Example**: Wheel axle (can spin continuously)

**URDF Syntax**:
```xml
<joint name="wheel_joint" type="continuous">
  <parent link="chassis"/>
  <child link="wheel"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

**Use Cases**:
- Drive wheels (mobile robots)
- Propellers and rotors
- Turrets (continuous 360° rotation)

**Visual Representation**:
```
    ╭─────╮
    │  ⟳  │  ← Unlimited rotation
    ╰─────╯
     360°+
```

---

### 3. Prismatic Joint (Linear Translation with Limits)

```
    Parent Link
         │
         │ Axis of translation
    ┌────┴────┐
    │  Joint  │  ← Prismatic joint
    └────┬────┘
         │ d (displacement)
         │ Range: [lower, upper]
         ▼
    Child Link (slides)
```

**Properties**:
- **Motion**: Linear translation along a single axis
- **Limits**: Required (lower and upper distance in meters)
- **Axis**: Direction of linear motion
- **Example**: Gripper linear actuator (opens/closes)

**URDF Syntax**:
```xml
<joint name="gripper_joint" type="prismatic">
  <parent link="gripper_base"/>
  <child link="gripper_finger"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit effort="10.0" velocity="0.5" lower="0.0" upper="0.1"/>
</joint>
```

**Use Cases**:
- Gripper fingers (open/close)
- Linear actuators (telescoping arms)
- Elevators (vertical translation)
- Drawer mechanisms

**Visual Representation**:
```
    ╭─────╮
    │  ⟷  │  ← Linear motion
    ╰─────╯
  [d_min, d_max]
```

---

### 4. Fixed Joint (No Motion)

```
    Parent Link
         │
         │ Rigid connection
    ┌────┴────┐
    │  Joint  │  ← Fixed joint (no motion)
    └────┬────┘
         │
         ▼
    Child Link (rigidly attached)
```

**Properties**:
- **Motion**: None (0 DoF)
- **Limits**: Not applicable
- **Axis**: Not applicable
- **Example**: Camera mount (fixed to robot head)

**URDF Syntax**:
```xml
<joint name="camera_mount" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0.2 0"/>
</joint>
```

**Use Cases**:
- Sensor mounts (cameras, LiDAR)
- Structural connections (connecting rigid parts)
- Cosmetic attachments (decorative elements)

**Visual Representation**:
```
    ╭─────╮
    │  ⊗  │  ← No motion (locked)
    ╰─────╯
```

---

### 5. Planar Joint (2D Translation + 1D Rotation)

```
    Parent Link
         │
         │ Planar surface
    ┌────┴────┐
    │  Joint  │  ← Planar joint
    └────┬────┘
         │ x, y, θ
         ▼
    Child Link (moves in 2D plane)
```

**Properties**:
- **Motion**: 2D translation (x, y) + rotation (θ) in a plane
- **Limits**: Optional (bounding box and angle limits)
- **Axis**: Normal to the plane
- **Example**: Air hockey puck on table

**URDF Syntax**:
```xml
<joint name="planar_joint" type="planar">
  <parent link="table"/>
  <child link="puck"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

**Use Cases**:
- Mobile robots on flat ground (before adding wheel constraints)
- Objects sliding on surfaces
- Simplified 2D robot models

**Visual Representation**:
```
    ╭─────╮
    │ ⟲⟷⟷ │  ← 2D translation + rotation
    ╰─────╯
   (x, y, θ)
```

---

### 6. Floating Joint (6 DoF: Unconstrained)

```
    Parent Link
         │
         │ No constraints
    ┌────┴────┐
    │  Joint  │  ← Floating joint
    └────┬────┘
         │ x, y, z, roll, pitch, yaw
         ▼
    Child Link (free in 3D space)
```

**Properties**:
- **Motion**: Full 6 DoF (3D position + 3D orientation)
- **Limits**: Not applicable
- **Axis**: Not applicable
- **Example**: Drone in free flight, floating object

**URDF Syntax**:
```xml
<joint name="floating_joint" type="floating">
  <parent link="world"/>
  <child link="drone_base"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

**Use Cases**:
- Flying robots (drones, quadcopters)
- Underwater robots (submarines)
- Root joint for free-floating humanoid (before contact with ground)

**Visual Representation**:
```
    ╭─────╮
    │ ⟲⟲⟲ │  ← Full 6 DoF
    ╰─────╯
  (x,y,z,r,p,y)
```

---

## Joint Type Comparison Table

| Joint Type | DoF | Motion | Limits Required | Common Use Cases |
|------------|-----|--------|-----------------|------------------|
| **Revolute** | 1 | Rotation (bounded) | Yes | Arm/leg joints |
| **Continuous** | 1 | Rotation (unbounded) | No | Wheels, propellers |
| **Prismatic** | 1 | Linear translation | Yes | Grippers, actuators |
| **Fixed** | 0 | None | No | Sensor mounts |
| **Planar** | 3 | 2D translation + rotation | Optional | Mobile robots (2D) |
| **Floating** | 6 | Full 3D pose | No | Drones, free-floating |

---

## Joint Axis Conventions

### Axis Definition

The `<axis>` tag defines the direction of motion using a 3D unit vector:

```xml
<axis xyz="x y z"/>
```

**Standard Axes**:
- **X-axis**: `[1, 0, 0]` - Forward/backward
- **Y-axis**: `[0, 1, 0]` - Left/right
- **Z-axis**: `[0, 0, 1]` - Up/down

**Custom Axes**:
- Diagonal: `[1, 1, 0]` (45° in XY plane)
- Must be normalized: sqrt(x² + y² + z²) = 1

### Rotation Axes for Revolute/Continuous Joints

```
     Z-axis (Yaw)
        ↑
        │
        │
        └──→ Y-axis (Pitch)
       ╱
      ╱
     ↙
  X-axis (Roll)
```

**Rotation Examples**:
- **Shoulder yaw**: Axis = [0, 0, 1] (Z) → Horizontal rotation
- **Elbow pitch**: Axis = [0, 1, 0] (Y) → Vertical bending
- **Wrist roll**: Axis = [1, 0, 0] (X) → Twisting

### Translation Axes for Prismatic Joints

```
  Gripper closing:

  ← [0, -1, 0] ───┐
                  │ Finger 1
                ╔═╧═╗
                ║   ║ Gripper base
                ╚═╤═╝
                  │ Finger 2
  → [0, 1, 0] ────┘
```

---

## Joint Limits and Dynamics

### Limit Parameters

```xml
<limit effort="30.0" velocity="1.57" lower="-1.57" upper="1.57"/>
```

| Parameter | Units | Description |
|-----------|-------|-------------|
| **effort** | Nm (revolute) or N (prismatic) | Max torque/force |
| **velocity** | rad/s (revolute) or m/s (prismatic) | Max speed |
| **lower** | rad or m | Minimum joint value |
| **upper** | rad or m | Maximum joint value |

### Dynamics Parameters

```xml
<dynamics damping="0.1" friction="0.01"/>
```

| Parameter | Description |
|-----------|-------------|
| **damping** | Viscous damping coefficient (resistance proportional to velocity) |
| **friction** | Coulomb friction (constant resistance to motion) |

**Usage**: Simulation engines (Gazebo, Isaac Sim) use these for realistic physics.

---

## Visual Representation of DoF

### 1 DoF Joints (Revolute, Continuous, Prismatic)

```
  Revolute:        Continuous:      Prismatic:

     ↻  ←→            ⟳  ←→            ←  →
    ╱│╲               ╱│╲              ││││
   ╱ │ ╲             ╱ │ ╲             ││││
  θ₁ │  θ₂         (no limit)        d₁  d₂
```

### Multi-DoF Joints (Planar, Floating)

```
  Planar (3 DoF):              Floating (6 DoF):

    ⟲  ↕                         ⟲⟲⟲
    ←→←→                         ↕↕↕
   (x,y,θ)                      (x,y,z,r,p,y)
```

---

## Usage in Book

- **Referenced in**: Chapter 5 (Core Concept 2: Links and Joints)
- **Purpose**: Help learners understand motion constraints and choose appropriate joint types
- **Learning Goal**: Learner can identify joint type from motion requirements and write correct URDF syntax

## Key Takeaways

1. **Revolute** = bounded rotation (most common in arms/legs)
2. **Continuous** = unbounded rotation (wheels, turrets)
3. **Prismatic** = linear translation (grippers, actuators)
4. **Fixed** = no motion (structural connections)
5. **Planar** = 2D motion (simplified mobile robots)
6. **Floating** = 6 DoF (drones, free-floating objects)
7. **Axis direction** defines motion direction (rotation or translation)
8. **Limits** constrain motion range (required for revolute/prismatic)
