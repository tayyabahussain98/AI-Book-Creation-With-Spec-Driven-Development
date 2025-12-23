---
sidebar_position: 3
title: Chapter 3 - Robot Models in Simulation
---

# Chapter 3: Robot Models in Simulation

## Introduction

You've mastered world creation and physics configuration. Now it's time to bring your robots into the simulation. This chapter bridges the gap between the URDF format you learned in Module 1 and the SDF format used by Gazebo. You'll learn to convert robot descriptions, spawn models, and tune parameters for stable simulation.

### Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapter 2 (Gazebo Physics & Environment Setup)
- Module 1, Chapter 5 (URDF and Robot Description)
- A working URDF file (e.g., `simple_arm.urdf` from Module 1)
- Gazebo Harmonic installed with `gz` command available

### Learning Objectives

By the end of this chapter, you will be able to:
1. Explain the differences between URDF and SDF formats
2. Convert URDF files to SDF using Gazebo tools
3. Spawn robot models in Gazebo worlds
4. Tune joint damping, friction, and collision geometry
5. Verify robot stability under gravity

---

## Core Concept 1: URDF vs SDF

### Format Overview

In Module 1, you learned **URDF** (Unified Robot Description Format) for describing robot kinematics. Gazebo uses **SDF** (Simulation Description Format), which is more powerful but follows similar principles.

| Aspect | URDF | SDF |
|--------|------|-----|
| **Primary Use** | ROS robot description | Gazebo simulation |
| **Scope** | Robot only | Entire worlds |
| **Sensors** | Not natively supported | Full sensor simulation |
| **Physics Config** | Not supported | Engine, timestep, gravity |
| **Plugins** | Via `<gazebo>` extension | Native support |
| **Tree Structure** | Strict single-parent | DAG (multiple parents) |

### Key Structural Differences

**URDF structure:**
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>...</inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

**Equivalent SDF structure:**
```xml
<model name="my_robot">
  <link name="base_link">
    <visual name="base_visual">...</visual>
    <collision name="base_collision">...</collision>
    <inertial>...</inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent>base_link</parent>
    <child>arm_link</child>
    <axis>
      <xyz>0 0 1</xyz>
      <limit>
        <lower>-1.57</lower>
        <upper>1.57</upper>
        <effort>100</effort>
        <velocity>1</velocity>
      </limit>
    </axis>
  </joint>
</model>
```

### When to Use Each Format

**Use URDF when:**
- Working exclusively with ROS 2 tools (MoveIt, Navigation2)
- Robot has simple tree structure (no closed loops)
- Using xacro for parameterized robot descriptions
- Need compatibility with existing ROS packages

**Use SDF when:**
- Simulating in Gazebo (sensors, physics, environments)
- Need closed kinematic loops (parallel robots)
- Configuring physics engine parameters per-robot
- Defining complete simulation worlds

**Best Practice:** Maintain your robot in URDF for ROS compatibility, convert to SDF for simulation.

See the [URDF vs SDF Comparison Diagram](../diagrams/module-2/urdf-vs-sdf-comparison.md) for detailed feature comparison.

---

## Core Concept 2: Spawning Robots in Gazebo

### Conversion Workflow

The standard workflow is: URDF → SDF → Gazebo.

#### Step 1: Validate URDF

```bash
# Check URDF syntax and tree structure
check_urdf robot.urdf

# Expected output:
# robot name is: my_robot
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 3 child(ren)
#     child(1):  upper_arm
#     ...
```

#### Step 2: Convert to SDF

```bash
# Basic conversion
gz sdf -p robot.urdf > robot.sdf

# With specific SDF version
gz sdf -p robot.urdf --sdf-version 1.9 > robot.sdf

# Validate the result
gz sdf -k robot.sdf
```

#### Step 3: Include in World File

**Method 1: Direct inclusion**
```xml
<world name="robot_arena">
  <!-- World config... -->

  <!-- Include robot model -->
  <include>
    <uri>model://my_robot</uri>
    <name>robot_instance</name>
    <pose>0 0 0.5 0 0 0</pose>  <!-- X Y Z Roll Pitch Yaw -->
  </include>
</world>
```

**Method 2: Inline model**
```xml
<world name="robot_arena">
  <!-- World config... -->

  <!-- Inline SDF model -->
  <model name="my_robot">
    <pose>0 0 0.5 0 0 0</pose>
    <!-- Full model definition here -->
  </model>
</world>
```

#### Step 4: Launch and Verify

```bash
# Launch world with robot
gz sim world_with_robot.sdf

# Or spawn at runtime via service
gz service -s /world/robot_arena/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "robot.sdf", pose: {position: {z: 0.5}}'
```

### Common Conversion Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| Missing inertia | "Treating as static" warning | Add `<inertial>` to all dynamic links |
| Package paths | "Unable to find file" | Replace `package://` with `model://` or absolute paths |
| Gazebo plugins | Plugins don't work | Migrate from Classic plugin names to Harmonic names |
| Self-collision | Robot explodes on spawn | Reduce collision size or disable self-collision |

See the [SDF Conversion Workflow Diagram](../diagrams/module-2/sdf-conversion-workflow.md) for step-by-step guidance.

---

## Core Concept 3: Joint and Collision Tuning

### Joint Dynamics

Gazebo joints need tuning beyond URDF limits. The key parameters are **damping** and **friction**.

```xml
<joint name="shoulder_joint" type="revolute">
  <parent>base_link</parent>
  <child>upper_arm</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>    <!-- Max torque (N·m) -->
      <velocity>2.0</velocity> <!-- Max angular velocity (rad/s) -->
    </limit>
    <dynamics>
      <damping>5.0</damping>   <!-- Viscous damping (N·m·s/rad) -->
      <friction>0.5</friction> <!-- Coulomb friction (N·m) -->
    </dynamics>
  </axis>
</joint>
```

**Damping** opposes motion proportionally to velocity. Higher damping:
- Reduces oscillation and overshoot
- Makes joint feel "heavier"
- Typical values: 0.1–10 N·m·s/rad

**Friction** provides constant resistance. It:
- Prevents drift at low velocities
- Models static friction in joint
- Typical values: 0.1–1.0 N·m

### Collision Geometry

Physics engines compute collisions at high frequency (1kHz). Complex meshes slow simulation dramatically.

**Best Practice:** Use simplified primitives for collision, detailed meshes for visuals.

```xml
<link name="torso">
  <!-- Visual: detailed mesh for appearance -->
  <visual name="torso_visual">
    <geometry>
      <mesh>
        <uri>model://robot/meshes/torso_detailed.dae</uri>
      </mesh>
    </geometry>
  </visual>

  <!-- Collision: simple box for physics -->
  <collision name="torso_collision">
    <geometry>
      <box>
        <size>0.3 0.2 0.5</size>
      </box>
    </geometry>
  </collision>
</link>
```

### Surface Properties

Tune how links interact with the environment:

```xml
<collision name="foot_collision">
  <geometry>
    <box><size>0.2 0.1 0.03</size></box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.9</mu>      <!-- High friction for grip -->
        <mu2>0.9</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e6</kp>      <!-- Contact stiffness -->
        <kd>100</kd>      <!-- Contact damping -->
        <min_depth>0.001</min_depth>  <!-- Penetration tolerance -->
      </ode>
    </contact>
  </surface>
</collision>
```

### Debugging Collisions

1. **Enable collision visualization:** View → Collisions in Gazebo GUI
2. **Check for overlap:** Adjacent link collisions should have 1-2mm gap
3. **Reduce collision size:** Make collision geometry slightly smaller than visual
4. **Disable self-collision:** As last resort, add `<self_collide>false</self_collide>`

See the [Collision Tuning Diagram](../diagrams/module-2/collision-tuning.md) for visual debugging guide.

---

## Core Concept 4: Stability Testing

### Gravity Test

The first test for any robot model: does it stay upright under gravity?

```bash
# Launch with robot spawned slightly above ground
gz sim world_with_robot.sdf

# Watch for 10 seconds
# ✓ Robot settles onto ground
# ✓ No jittering or vibration
# ✓ Joints stay within limits
# ✗ Robot falls over = CoM/support polygon issue
# ✗ Robot bounces = Contact parameters too stiff
# ✗ Robot sinks = Initial pose below ground
```

### Center of Mass Verification

For bipedal robots, CoM must be above the support polygon.

**In Gazebo:**
1. Right-click model → Inspect
2. Check "Center of Mass" to visualize
3. CoM marker (yellow sphere) should be:
   - Horizontally: between feet
   - Vertically: low (hip height or below for stability)

**Common CoM Issues:**
- **CoM too high:** Robot topples easily
- **CoM too far forward/back:** Falls when standing
- **Asymmetric CoM:** Leans to one side

### Joint Limit Testing

Verify joints respect their limits:

```bash
# Publish joint command beyond limit
ros2 topic pub /joint_commands std_msgs/Float64MultiArray \
  "{data: [3.14]}"  # Request π rad (180°)

# If limit is ±90° (1.57 rad):
# ✓ Joint stops at 1.57 rad
# ✓ No oscillation at limit
# ✗ Joint exceeds limit = limit not enforced
# ✗ Joint vibrates = needs more damping
```

### Stability Checklist

Before using a robot model for development:

- [ ] Robot spawns without warnings
- [ ] Robot stands under gravity for 30+ seconds
- [ ] No collision warnings in console
- [ ] Joints move smoothly within limits
- [ ] Contact with ground is stable (no bouncing)
- [ ] CoM visualization shows reasonable position

---

## Hands-On Example: Stable Humanoid Model

Let's examine a stable humanoid model designed for Gazebo simulation.

### Model Overview

The `humanoid_stable.sdf` model demonstrates:
- Proper mass distribution (CoM in torso)
- Wide feet for stable support polygon
- Joint damping for smooth motion
- High friction foot contacts

### Key Design Decisions

**1. Torso (Root Link)**
```xml
<link name="torso">
  <inertial>
    <mass>20.0</mass>  <!-- Heaviest link = low CoM -->
    <pose>0 0 0 0 0 0</pose>  <!-- CoM at geometric center -->
  </inertial>
</link>
```

**2. Foot Contact Surface**
```xml
<collision name="left_foot_collision">
  <pose>0.02 0 -0.015 0 0 0</pose>  <!-- Offset forward for balance -->
  <geometry>
    <box><size>0.2 0.1 0.03</size></box>  <!-- Large footprint -->
  </geometry>
  <surface>
    <friction>
      <ode><mu>0.9</mu></ode>  <!-- High grip -->
    </friction>
  </surface>
</collision>
```

**3. Hip Joint with Damping**
```xml
<joint name="left_hip_pitch" type="revolute">
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>-1.05</lower>  <!-- -60° -->
      <upper>0.52</upper>   <!-- +30° -->
      <effort>100</effort>
      <velocity>5.0</velocity>
    </limit>
    <dynamics>
      <damping>5.0</damping>   <!-- Prevents oscillation -->
      <friction>0.5</friction>
    </dynamics>
  </axis>
</joint>
```

### Spawning the Humanoid

```bash
# Create world file that includes humanoid
cat > test_humanoid.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="test_world">
    <physics name="default" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>

    <!-- Ground -->
    <model name="ground">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
      </link>
    </model>

    <!-- Include humanoid -->
    <include>
      <uri>model://humanoid_stable</uri>
      <pose>0 0 0.98 0 0 0</pose>  <!-- Start just above ground -->
    </include>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  </world>
</sdf>
EOF

# Launch
gz sim test_humanoid.sdf
```

See the full model: [humanoid_stable.sdf](../code-examples/module-2/humanoid_stable.sdf)

---

## Diagrams Referenced

This chapter references the following diagram specifications:

1. **[URDF vs SDF Comparison](../diagrams/module-2/urdf-vs-sdf-comparison.md)**: Format capabilities and when to use each
2. **[SDF Conversion Workflow](../diagrams/module-2/sdf-conversion-workflow.md)**: Step-by-step URDF to SDF conversion
3. **[Collision Tuning](../diagrams/module-2/collision-tuning.md)**: Visual vs collision geometry debugging

---

## Summary

In this chapter, you learned to bring robots into Gazebo simulation:

- **URDF vs SDF**: URDF for ROS compatibility, SDF for simulation features; convert with `gz sdf -p`
- **Spawning workflow**: Validate URDF → Convert to SDF → Include in world → Launch
- **Joint tuning**: Add damping (5-10 N·m·s/rad) and friction (0.1-1.0 N·m) to prevent oscillation
- **Collision geometry**: Use simple primitives for physics, detailed meshes for visuals
- **Surface properties**: Set μ=0.9 for foot-ground friction, kp=1e6 for contact stiffness
- **Stability testing**: Verify robot stands under gravity with no jitter or warnings

---

## Self-Assessment Checklist

Before proceeding to Chapter 4, verify you can:

- [ ] **Distinguish formats**: Explain when to use URDF vs SDF and what each supports
- [ ] **Convert files**: Use `gz sdf -p` to convert URDF to SDF and validate with `gz sdf -k`
- [ ] **Spawn robots**: Include a robot model in a world file and launch in Gazebo
- [ ] **Tune joints**: Add damping and friction to prevent joint oscillation
- [ ] **Fix collisions**: Identify and resolve collision geometry issues
- [ ] **Test stability**: Verify a robot stands stably under gravity for 30+ seconds

If you can check all boxes, you're ready for **Chapter 4: Sensor Simulation**!
