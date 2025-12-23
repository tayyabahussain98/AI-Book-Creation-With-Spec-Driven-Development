---
sidebar_position: 2
title: Chapter 2 - Gazebo Physics & Environment Setup
---

# Chapter 2: Gazebo Physics & Environment Setup

## Introduction

In Chapter 1, you learned *what* digital twins are and *why* we need them. Now it's time to build one. This chapter focuses on **Gazebo Harmonic**—the physics-first simulator that serves as the standard testing ground for ROS 2 robots. You'll learn to configure physics engines, design simulation environments, and understand the stability metrics essential for humanoid robots.

### Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapter 1 (Digital Twin Foundations)
- Gazebo Harmonic installed (`gz sim --version` returns 8.x)
- Basic understanding of XML syntax
- Familiarity with coordinate frames (from Module 1)

### Learning Objectives

By the end of this chapter, you will be able to:
1. Create SDF world files with custom physics configurations
2. Select and configure physics engines (DART, Bullet, ODE)
3. Explain humanoid stability using ZMP and CoM concepts
4. Design test environments with friction, obstacles, and terrain
5. Launch and verify Gazebo simulations from the command line

---

## Core Concept 1: Gazebo World File Structure

### What is an SDF World File?

An **SDF (Simulation Description Format) world file** defines everything in your simulation: physics engine settings, lighting, terrain, obstacles, and robots. Unlike URDF (which describes only robots), SDF describes *entire worlds*.

```text
SDF World File Hierarchy:
─────────────────────────
<sdf version="1.9">
  └── <world name="...">
        ├── <physics>        ← Engine type, timestep, RTF
        ├── <gravity>        ← Gravity vector
        ├── <scene>          ← Ambient light, shadows
        ├── <light>          ← Sun, lamps (0 or more)
        ├── <model>          ← Ground, obstacles, robots (0 or more)
        └── <plugin>         ← Physics, sensors, user commands (0 or more)
```

### Minimal World File

The absolute minimum to run physics (though not very useful):

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="minimal">
    <physics name="default" type="dart">
      <max_step_size>0.001</max_step_size>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  </world>
</sdf>
```

**What's missing?** No ground plane (objects fall forever), no lighting (black screen), no way to spawn robots at runtime. Let's fix that.

### Complete World File Anatomy

A production-ready world file includes these essential sections:

#### 1. Physics Configuration

```xml
<physics name="1ms_dart" type="dart">
  <!-- Timestep: 0.001s = 1ms = 1kHz physics rate -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor: 1.0 = real-time, 0.0 = fast as possible -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Max updates per second (0 = unlimited) -->
  <real_time_update_rate>1000</real_time_update_rate>

  <!-- DART-specific solver settings -->
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

#### 2. Environment Forces

```xml
<!-- Gravity: Z-down at 9.81 m/s² -->
<gravity>0 0 -9.81</gravity>

<!-- Magnetic field for IMU/compass sensors (microtesla) -->
<magnetic_field>5.5645e-6 22.8758e-6 -42.3884e-6</magnetic_field>
```

#### 3. Visual Setup

```xml
<scene>
  <ambient>0.4 0.4 0.4 1</ambient>    <!-- Fill light in shadows -->
  <background>0.7 0.85 1.0 1</background>  <!-- Sky color -->
  <shadows>true</shadows>
</scene>

<light name="sun" type="directional">
  <direction>-0.5 0.5 -0.9</direction>
  <cast_shadows>true</cast_shadows>
  <diffuse>0.8 0.8 0.8 1</diffuse>
</light>
```

#### 4. Required Plugins

```xml
<!-- Physics engine (required) -->
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>

<!-- Sensor simulation -->
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>

<!-- Runtime model spawning -->
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

<!-- State publishing for GUI -->
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
```

See the [SDF World Structure Diagram](../diagrams/module-2/sdf-world-structure.md) for a visual breakdown.

---

## Core Concept 2: Physics Engine Configuration

### Choosing a Physics Engine

Gazebo Harmonic supports three physics engines. Your choice significantly impacts simulation accuracy and speed.

| Engine | Strength | Best For | Default Timestep |
|--------|----------|----------|------------------|
| **DART** | Joint accuracy | Humanoids, manipulators | 1ms (1kHz) |
| **Bullet** | Collision speed | Large scenes, soft bodies | 2ms (500Hz) |
| **ODE** | Joint stability | Legacy, simple robots | 5ms (200Hz) |

### DART (Default Choice)

**DART** (Dynamic Animation and Robotics Toolkit) is the default and recommended engine for articulated robots. It uses the Featherstone algorithm for efficient multi-body dynamics.

```xml
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <dart>
    <solver>
      <!-- dantzig: Fast, general-purpose (default) -->
      <!-- pgs: Better for high-friction contacts -->
      <solver_type>dantzig</solver_type>
    </solver>
    <!-- Use Bullet's collision detection (faster than DART's) -->
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

**When to use DART:**
- Humanoid robots (20+ degrees of freedom)
- Industrial manipulators requiring precise joint control
- Contact-rich manipulation tasks
- When joint limits must be strictly enforced

### Bullet Physics

**Bullet** excels at rigid body dynamics and offers GPU acceleration for collision detection.

```xml
<physics name="bullet_physics" type="bullet">
  <max_step_size>0.002</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <bullet>
    <solver>
      <iters>50</iters>  <!-- More iterations = more accurate -->
    </solver>
  </bullet>
</physics>
```

**When to use Bullet:**
- Scenes with many objects (100+ colliding bodies)
- Soft body simulation (cloth, deformable objects)
- Mobile robot navigation (simple dynamics)
- Fast prototyping where precision isn't critical

### ODE (Open Dynamics Engine)

**ODE** is the legacy engine from Gazebo Classic, useful for backward compatibility.

```xml
<physics name="ode_physics" type="ode">
  <max_step_size>0.005</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <ode>
    <solver>
      <type>world</type>
      <iters>50</iters>
    </solver>
    <constraints>
      <cfm>0.0001</cfm>  <!-- Constraint Force Mixing -->
      <erp>0.2</erp>     <!-- Error Reduction Parameter -->
    </constraints>
  </ode>
</physics>
```

**When to use ODE:**
- Migrating from Gazebo Classic
- When DART exhibits instability (rare)
- Lower control frequencies (100-200 Hz)

See the [Physics Engine Comparison Diagram](../diagrams/module-2/physics-engine-comparison.md) for detailed trade-offs.

### Timestep Selection

The physics timestep (`max_step_size`) critically affects accuracy and performance:

| Timestep | Frequency | Accuracy | Speed | Use Case |
|----------|-----------|----------|-------|----------|
| 0.001s | 1kHz | High | Slow | Humanoid control, manipulation |
| 0.002s | 500Hz | Medium | Medium | Mobile robots, general use |
| 0.005s | 200Hz | Low | Fast | Fast training, simple dynamics |

**Rule of thumb:** Match your control loop frequency. If your robot controller runs at 1kHz, use a 1ms timestep.

---

## Core Concept 3: Humanoid Stability Fundamentals

### Why Humanoids Fall

Unlike wheeled robots with inherent stability, humanoid robots are **statically unstable**—they must actively balance like humans. Understanding stability metrics is essential for simulation and control.

### Center of Mass (CoM)

The **Center of Mass** is the point where the robot's total mass is effectively concentrated. For a humanoid standing upright, CoM is typically in the torso, about 1 meter above the ground.

```text
         ┌─────┐
         │Head │
         └──┬──┘
            │
     ┌──────┼──────┐
     │   ●←─┼──────│──── CoM (Center of Mass)
     │  Torso      │     Height ≈ 1.0m
     └──────┬──────┘
            │
       ┌────┴────┐
       │ Pelvis  │
       └────┬────┘
      ┌─────┴─────┐
      │           │
     ┌┴┐         ┌┴┐
     │L│         │R│
     │e│         │e│
     │g│         │g│
     └┬┘         └┬┘
    ══╧══       ══╧══
    Feet on ground
```

### Zero-Moment Point (ZMP)

The **Zero-Moment Point** is where the sum of all moments (from gravity and inertia) equals zero. In simple terms, it's where the ground reaction force effectively acts.

**Stability Criterion:**
- **ZMP inside support polygon** → Robot is stable
- **ZMP outside support polygon** → Robot is falling

```text
TOP VIEW - Support Polygon:

    ┌───────────────────────────────────────┐
    │                                       │
    │     ┌─────────┐     ┌─────────┐      │
    │     │  Left   │     │  Right  │      │
    │     │  Foot   │     │  Foot   │      │
    │     └────┬────┘     └────┬────┘      │
    │          │               │            │
    │          └───────●───────┘            │
    │                 ZMP                   │
    │                                       │
    │    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░    │
    │    ░░░░░ Support Polygon ░░░░░░░    │
    │    ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░    │
    │                                       │
    └───────────────────────────────────────┘

    ZMP inside = STABLE ✓
```

### Support Polygon

The **support polygon** is the convex hull of all contact points between the robot and ground:

- **Double support** (both feet): Large polygon (~0.15 m²)
- **Single support** (one foot): Small polygon (~0.02 m²)—much less stable!

### Static vs Dynamic Stability

| Type | CoM-ZMP Relationship | Example |
|------|---------------------|---------|
| **Static** | ZMP = CoM projection | Standing still |
| **Dynamic** | ZMP ≠ CoM projection | Walking, running |

During walking, the ZMP shifts ahead of or behind the CoM projection due to acceleration. Humanoid controllers must plan ZMP trajectories that stay within the support polygon throughout the gait cycle.

See the [Humanoid Stability ZMP Diagram](../diagrams/module-2/humanoid-stability-zmp.md) for detailed visualizations.

### Stability in Gazebo

To monitor stability in simulation:

1. **Add force/torque sensors** at ankles to measure ground reaction forces
2. **Add contact sensors** on feet to detect ground contact
3. **Calculate ZMP** from force/torque readings
4. **Visualize** CoM and ZMP in RViz or Gazebo GUI

```xml
<!-- Force/Torque Sensor at Ankle -->
<sensor name="left_ankle_ft" type="force_torque">
  <always_on>true</always_on>
  <update_rate>1000</update_rate>
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
</sensor>
```

---

## Core Concept 4: Environmental Design

### Ground Plane with Friction

The ground plane is the most important model in your world. Its friction properties determine how robots grip the surface.

```xml
<model name="ground_plane">
  <static>true</static>
  <link name="ground_link">
    <collision name="ground_collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <!-- Friction coefficient: 0.8 = rubber on concrete -->
            <mu>0.8</mu>
            <mu2>0.8</mu2>
          </ode>
        </friction>
        <contact>
          <ode>
            <kp>1e6</kp>   <!-- Contact stiffness -->
            <kd>100</kd>   <!-- Contact damping -->
          </ode>
        </contact>
      </surface>
    </collision>
    <visual name="ground_visual">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <ambient>0.3 0.3 0.3 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Friction Coefficient Guidelines

| Surface | μ (mu) | Example |
|---------|--------|---------|
| Ice | 0.05 | Slippery, robot slides |
| Polished concrete | 0.3 | Indoor warehouse |
| Concrete | 0.5 | Standard floor |
| Rubber on concrete | 0.8 | Robot foot (typical) |
| High-grip | 1.0+ | Sandpaper-like surface |

### Static vs Dynamic Obstacles

**Static obstacles** don't move and have no physics overhead:

```xml
<model name="wall">
  <static>true</static>  <!-- No physics computation -->
  <pose>5 0 1 0 0 0</pose>
  <link name="wall_link">
    <collision name="wall_collision">
      <geometry>
        <box><size>0.1 4 2</size></box>
      </geometry>
    </collision>
    <visual name="wall_visual">
      <geometry>
        <box><size>0.1 4 2</size></box>
      </geometry>
    </visual>
  </link>
</model>
```

**Dynamic obstacles** can be pushed and require inertial properties:

```xml
<model name="pushable_box">
  <!-- No <static> tag = dynamic by default -->
  <pose>2 0 0.15 0 0 0</pose>
  <link name="box_link">
    <inertial>
      <mass>5.0</mass>
      <inertia>
        <ixx>0.0375</ixx>
        <iyy>0.0375</iyy>
        <izz>0.0375</izz>
      </inertia>
    </inertial>
    <collision>...</collision>
    <visual>...</visual>
  </link>
</model>
```

### Terrain Features for Testing

Test your robot's capabilities with varied terrain:

**Ramp (15° incline):**
```xml
<model name="ramp">
  <static>true</static>
  <pose>5 0 0.13 0 -0.26 0</pose>  <!-- Pitch = 15° -->
  <link name="ramp_link">
    <collision name="ramp_collision">
      <geometry>
        <box><size>2.0 1.0 0.05</size></box>
      </geometry>
      <surface>
        <friction><ode><mu>0.9</mu></ode></friction>
      </surface>
    </collision>
  </link>
</model>
```

**Stairs (for humanoid testing):**
```xml
<!-- Each step as a separate box -->
<model name="step_1">
  <static>true</static>
  <pose>3 0 0.1 0 0 0</pose>
  <link><collision><geometry><box><size>0.3 1.0 0.2</size></box></geometry></collision></link>
</model>
<model name="step_2">
  <static>true</static>
  <pose>3.3 0 0.3 0 0 0</pose>
  <link><collision><geometry><box><size>0.3 1.0 0.2</size></box></geometry></collision></link>
</model>
```

---

## Hands-On Example: Creating a Gazebo World

Let's walk through the complete world file from our code examples. This file demonstrates all concepts from this chapter.

### File: `simple_world.sdf`

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="robot_arena">
    <!-- Physics: DART at 1kHz -->
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart>
        <solver><solver_type>dantzig</solver_type></solver>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <!-- Lighting -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <direction>-0.5 0.5 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground with friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface>
            <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><diffuse>0.7 0.7 0.7 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Static obstacle -->
    <model name="obstacle_box">
      <static>true</static>
      <pose>2 0 0.25 0 0 0</pose>
      <link name="box_link">
        <collision><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>
        <visual><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual>
      </link>
    </model>

    <!-- Required plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
  </world>
</sdf>
```

See the full annotated version: [simple_world.sdf](../code-examples/module-2/simple_world.sdf)

### Launching the World

```bash
# Launch with GUI
gz sim simple_world.sdf

# Launch headless (server only, for training)
gz sim -s simple_world.sdf

# Launch with verbose logging
gz sim -v 4 simple_world.sdf
```

### Verifying the Simulation

1. **Check physics rate:** Look at "Real Time Factor" in GUI status bar (should be ~1.0)
2. **Test friction:** Spawn a box, apply force—it should slide realistically
3. **Verify gravity:** Drop an object—it should accelerate at 9.81 m/s²
4. **Check lighting:** Shadows should appear from directional light

---

## Diagrams Referenced

This chapter references the following diagram specifications:

1. **[SDF World Structure](../diagrams/module-2/sdf-world-structure.md)**: Hierarchical view of SDF world file elements
2. **[Physics Engine Comparison](../diagrams/module-2/physics-engine-comparison.md)**: DART vs Bullet vs ODE decision matrix
3. **[Humanoid Stability ZMP](../diagrams/module-2/humanoid-stability-zmp.md)**: CoM, ZMP, and support polygon visualization

---

## Summary

In this chapter, you learned to create and configure Gazebo simulation environments:

- **SDF world files** organize physics, lighting, models, and plugins in a hierarchical XML structure
- **DART is the default engine** for articulated robots; use Bullet for large scenes, ODE for legacy compatibility
- **Timestep selection** should match your control loop frequency (1ms for 1kHz control)
- **Humanoid stability** depends on keeping ZMP inside the support polygon; single support is 10× less stable than double support
- **Friction coefficients** (μ) control robot-ground interaction; 0.8 is typical for rubber feet on concrete
- **Static models** skip physics computation; dynamic models require inertial properties

---

## Self-Assessment Checklist

Before proceeding to Chapter 3, verify you can:

- [ ] **Create a world file**: Write an SDF file with physics config, ground plane, lighting, and plugins
- [ ] **Select physics engine**: Choose DART, Bullet, or ODE based on robot type and justify your choice
- [ ] **Configure timestep**: Set appropriate `max_step_size` for your control frequency
- [ ] **Explain ZMP**: Describe what ZMP is and why it must stay inside the support polygon
- [ ] **Set friction**: Adjust friction coefficients to simulate different floor surfaces
- [ ] **Launch Gazebo**: Start simulation with `gz sim` and verify real-time factor

If you can check all boxes, you're ready for **Chapter 3: Robot Models in Simulation**!
