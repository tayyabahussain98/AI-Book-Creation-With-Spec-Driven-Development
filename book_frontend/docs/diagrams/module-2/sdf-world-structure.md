# SDF World Structure Specification

**Purpose**: Illustrate the anatomy of an SDF world file showing how physics, models, lighting, and plugins are organized

**Diagram Type**: architecture

## SDF World File Hierarchy

```text
┌─────────────────────────────────────────────────────────────────┐
│                        SDF FILE (root)                          │
│                      <sdf version="1.9">                        │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│                         <world>                                 │
│                    name="robot_arena"                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐ │
│  │  <physics>  │  │  <gravity>  │  │  <magnetic_field>       │ │
│  │ • type=dart │  │  0 0 -9.81  │  │  Earth field vector     │ │
│  │ • timestep  │  └─────────────┘  └─────────────────────────┘ │
│  │ • rtf       │                                               │
│  └─────────────┘                                               │
│                                                                 │
│  ┌─────────────┐  ┌─────────────────────────────────────────┐  │
│  │   <scene>   │  │            <light> (0..N)               │  │
│  │ • ambient   │  │  ┌─────────┐  ┌─────────┐               │  │
│  │ • background│  │  │   sun   │  │  lamp   │               │  │
│  │ • shadows   │  │  │direction│  │  point  │               │  │
│  └─────────────┘  │  └─────────┘  └─────────┘               │  │
│                   └─────────────────────────────────────────┘  │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   <model> (0..N)                         │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │   │
│  │  │ ground_plane │  │ obstacle_box │  │    robot     │  │   │
│  │  │   (static)   │  │   (static)   │  │  (dynamic)   │  │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘  │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   <plugin> (0..N)                        │   │
│  │  ┌─────────┐  ┌─────────────┐  ┌───────────────────┐   │   │
│  │  │ Physics │  │ UserCommands│  │ SceneBroadcaster  │   │   │
│  │  └─────────┘  └─────────────┘  └───────────────────┘   │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Model Internal Structure

```text
<model name="robot_arm">
├── <static>false</static>          ← Dynamic (has physics)
├── <pose>0 0 0 0 0 0</pose>        ← Initial position/orientation
│
├── <link name="base_link">         ← Physical body
│   ├── <inertial>                  ← Mass properties
│   │   ├── <mass>10.0</mass>
│   │   └── <inertia>               ← 3×3 inertia tensor
│   │       ├── <ixx>, <iyy>, <izz>
│   │       └── <ixy>, <ixz>, <iyz>
│   │
│   ├── <collision>                 ← Physics interaction
│   │   ├── <geometry>              ← Shape (box/sphere/mesh)
│   │   └── <surface>               ← Friction, contact params
│   │
│   ├── <visual>                    ← Rendering appearance
│   │   ├── <geometry>              ← Visual shape (can differ)
│   │   └── <material>              ← Color, texture, PBR
│   │
│   └── <sensor> (optional)         ← Attached sensors
│       └── <camera>, <lidar>, <imu>
│
├── <joint name="shoulder_joint">   ← Connection between links
│   ├── <type>revolute</type>       ← Joint type
│   ├── <parent>base_link</parent>
│   ├── <child>upper_arm</child>
│   ├── <axis>                      ← Rotation/translation axis
│   │   ├── <xyz>0 0 1</xyz>        ← Z-axis rotation
│   │   └── <limit>                 ← Position, velocity, effort
│   └── <dynamics>                  ← Damping, friction
│
└── <plugin>                        ← Model-level plugins
    └── <filename>diff_drive.so</filename>
```

## Physics Configuration Detail

```text
<physics name="robot_physics" type="dart">
│
├── <max_step_size>0.001</max_step_size>
│   └── Timestep: 1ms = 1kHz physics update rate
│
├── <real_time_factor>1.0</real_time_factor>
│   └── RTF: 1.0 = real-time, 0.0 = fast as possible
│
├── <real_time_update_rate>1000</real_time_update_rate>
│   └── Max physics Hz (0 = unlimited)
│
├── <dart>                          ← Engine-specific config
│   ├── <solver>
│   │   └── <solver_type>dantzig</solver_type>
│   │       Options: dantzig (fast), pgs (stable)
│   │
│   └── <collision_detector>bullet</collision_detector>
│       Options: bullet (fast), fcl (accurate), ode (legacy)
│
└── Engine Type Options:
    • type="dart"   → Default, best for articulated robots
    • type="bullet" → Fast collision, soft bodies
    • type="ode"    → Legacy, stable joints
```

## Surface Properties Detail

```text
<surface>
├── <friction>
│   ├── <ode>                       ← ODE-compatible friction model
│   │   ├── <mu>0.8</mu>            ← Primary friction coefficient
│   │   │   └── 0.0=ice, 0.5=concrete, 0.8=rubber, 1.0+=grip
│   │   ├── <mu2>0.8</mu2>          ← Secondary direction friction
│   │   └── <fdir1>0 0 0</fdir1>    ← Friction direction vector
│   │
│   └── <torsional>                 ← Rotational friction
│       ├── <coefficient>0.5</coefficient>
│       └── <surface_radius>0.05</surface_radius>
│
└── <contact>
    └── <ode>
        ├── <kp>1e6</kp>             ← Contact stiffness (N/m)
        │   └── Higher = harder surface
        ├── <kd>100</kd>             ← Contact damping (N·s/m)
        │   └── Prevents bouncing
        ├── <max_vel>100</max_vel>   ← Max penetration correction velocity
        └── <min_depth>0.001</min_depth>  ← Penetration threshold (m)
```

## Element Relationships

```text
                        SDF ELEMENT DEPENDENCIES
                        ═══════════════════════

World-Level Elements (order-independent):
─────────────────────────────────────────
<physics>  ────────────────────────────────┐
<gravity>  ─────────────────────────────── │ Applied to
<magnetic_field> ───────────────────────── │ all models
                                           ↓
<model> ←────────────────────────────────────
   │
   ├─ <static>true ───► No physics computation (efficient)
   │
   ├─ <static>false ──► Requires <inertial> in each <link>
   │
   └─ <link> ─────────┬─ <collision> ──► Needed for physics
                      ├─ <visual> ─────► Needed for rendering
                      └─ <sensor> ─────► Needs plugin system

<plugin> (world-level):
─────────────────────
Physics      ──────► Required: runs physics engine
Sensors      ──────► Required if any <sensor> elements exist
UserCommands ──────► Optional: enables runtime spawning
SceneBroadcaster ──► Optional: publishes state for GUI

Inheritance:
───────────
<world gravity="..."> ────► All models inherit unless overridden
<model pose="..."> ───────► All links relative to model pose
<link pose="..."> ────────► Collision/visual relative to link
```

## Minimal vs Complete World Files

```text
MINIMAL WORLD (bare minimum for physics):
─────────────────────────────────────────
<sdf version="1.9">
  <world name="minimal">
    <physics name="default" type="dart">
      <max_step_size>0.001</max_step_size>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  </world>
</sdf>

Missing: No ground (objects fall forever), no lighting (black screen)


COMPLETE WORLD (production-ready):
──────────────────────────────────
<sdf version="1.9">
  <world name="complete">
    <!-- Physics engine with tuned parameters -->
    <physics name="1ms_dart" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart><solver><solver_type>dantzig</solver_type></solver></dart>
    </physics>

    <!-- Environmental forces -->
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>5.5645e-6 22.8758e-6 -42.3884e-6</magnetic_field>
    <atmosphere type="adiabatic"/>

    <!-- Visual setup -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">...</light>

    <!-- Ground with friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="ground">
        <collision>...</collision>
        <visual>...</visual>
      </link>
    </model>

    <!-- Required plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  </world>
</sdf>
```

## Usage in Book

- **Referenced in**: Chapter 2 (Core Concept 1: Gazebo World File Structure)
- **Purpose**: Provide visual map of SDF organization before diving into code
- **Learning Goal**: Understand hierarchical structure so learners can navigate and modify world files confidently

## Key Takeaways

1. **Hierarchical structure**: SDF → World → Physics/Models/Lights/Plugins
2. **Physics configuration**: Engine type, timestep, RTF, solver settings all at world level
3. **Model composition**: Links (with inertial/collision/visual) connected by joints
4. **Surface properties**: Friction coefficients and contact parameters control robot-ground interaction
5. **Plugins required**: Physics plugin mandatory; Sensors plugin for any sensor-equipped models
6. **Static vs dynamic**: Static models skip physics (efficient); dynamic models need inertial properties
