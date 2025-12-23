# Gazebo Architecture Specification

**Purpose**: Illustrate the layered architecture of Gazebo Harmonic and how components interact to produce physics-based simulation

**Diagram Type**: architecture

## Gazebo Harmonic Layered Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                    USER INTERFACE LAYER                         │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐  │
│  │  gz sim GUI    │  │  RViz2 (ROS 2) │  │  Custom UIs     │  │
│  │  (Qt-based)    │  │  (Visualization)│  │  (Python/C++)   │  │
│  └────────┬───────┘  └────────┬───────┘  └────────┬────────┘  │
└───────────┼──────────────────┼───────────────────┼────────────┘
            │                  │                   │
            ↓                  ↓                   ↓
┌─────────────────────────────────────────────────────────────────┐
│                    TRANSPORT LAYER                              │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Gazebo Transport (gz-transport)                        │   │
│  │  • Topic-based pub/sub (e.g., /model/robot/pose)       │   │
│  │  • Services (e.g., /world/default/create)              │   │
│  │  │  Protocol: Google Protocol Buffers over TCP/UDP     │   │
│  └─────────────────────────────────────────────────────────┘   │
│            ↕                                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  ROS 2 Bridge (ros_gz_bridge)                           │   │
│  │  • Maps gz-transport ↔ ROS 2 DDS topics                │   │
│  │  • Example: /scan (gz) → /scan (ROS 2 LaserScan)       │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
            │
            ↓
┌─────────────────────────────────────────────────────────────────┐
│                    SIMULATION CORE LAYER                        │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐  │
│  │  World Manager │  │  Entity Manager│  │  System Manager │  │
│  │  • Time control│  │  • ECS pattern │  │  • Plugin loader│  │
│  │  • Pause/play  │  │  • Components  │  │  • Update loop  │  │
│  │  • Step mode   │  │  • Entities    │  │  • Parallelism  │  │
│  └────────┬───────┘  └────────┬───────┘  └────────┬────────┘  │
└───────────┼──────────────────┼───────────────────┼────────────┘
            │                  │                   │
            ↓                  ↓                   ↓
┌─────────────────────────────────────────────────────────────────┐
│                    PHYSICS ENGINE LAYER                         │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Physics Abstraction (gz-physics)                       │   │
│  │  • Unified API for multiple engines                     │   │
│  └─────────────────────────────────────────────────────────┘   │
│       │                      │                      │           │
│       ↓                      ↓                      ↓           │
│  ┌────────┐            ┌────────┐            ┌────────┐        │
│  │  DART  │            │ Bullet │            │  ODE   │        │
│  │(default)│            │        │            │        │        │
│  └────────┘            └────────┘            └────────┘        │
│  • Contact forces      • Fast collisions     • Stable joints   │
│  • Constraint solving  • Soft bodies         • Legacy support  │
│  • 1kHz timestep       • GPU acceleration    • 200Hz timestep  │
└─────────────────────────────────────────────────────────────────┘
            │
            ↓
┌─────────────────────────────────────────────────────────────────┐
│                    RENDERING ENGINE LAYER                       │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Rendering Abstraction (gz-rendering)                   │   │
│  │  • Scene graph management                               │   │
│  │  • Camera frustum culling                               │   │
│  └─────────────────────────────────────────────────────────┘   │
│       │                      │                                  │
│       ↓                      ↓                                  │
│  ┌────────┐            ┌────────┐                              │
│  │ OGRE 2 │            │ OGRE 1 │                              │
│  │(default)│            │(legacy)│                              │
│  └────────┘            └────────┘                              │
│  • PBR materials       • Compatibility                         │
│  • 60Hz refresh        • Lower overhead                        │
└─────────────────────────────────────────────────────────────────┘
            │
            ↓
┌─────────────────────────────────────────────────────────────────┐
│                    PLUGIN ECOSYSTEM LAYER                       │
│  ┌────────────────┐  ┌────────────────┐  ┌─────────────────┐  │
│  │ Sensor Plugins │  │ Actuator Plugins│  │  World Plugins  │  │
│  │ • GPU LiDAR    │  │ • Diff drive    │  │  • Wind model   │  │
│  │ • Depth camera │  │ • Joint control │  │  • Magnetometer │  │
│  │ • IMU          │  │ • Gripper force │  │  • Buoyancy     │  │
│  └────────────────┘  └────────────────┘  └─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Component Details

### Layer 1: User Interface Layer
**Purpose**: Human interaction with the simulation

**Components**:
- **gz sim GUI**: Built-in Qt-based 3D viewer with play/pause controls, entity inspector, plot widgets
- **RViz2**: ROS 2 visualization tool, displays topics like `/tf`, `/scan`, `/camera/image_raw`
- **Custom UIs**: Python scripts or web dashboards using Gazebo Transport API for remote control

**Data Flow**: User clicks "Play" → GUI sends service request `/world/default/control` → World Manager processes

### Layer 2: Transport Layer
**Purpose**: Inter-process communication between Gazebo components and external tools

**Gazebo Transport (gz-transport)**:
- Middleware library for topic-based pub/sub and services
- Uses Google Protocol Buffers (protobuf) for serialization
- Supports discovery (automatic node detection on same network)
- Example topic: `/model/robot_arm/pose` (publishes `gz.msgs.Pose` at 1kHz)

**ROS 2 Bridge (ros_gz_bridge)**:
- Bidirectional adapter between Gazebo and ROS 2 ecosystems
- Maps message types: `gz.msgs.LaserScan` ↔ `sensor_msgs/msg/LaserScan`
- Configuration via YAML file or launch parameters
- Latency: ~1-2ms typical on same machine

### Layer 3: Simulation Core Layer
**Purpose**: Orchestrate simulation state and plugin execution

**World Manager**:
- Manages simulation time (real-time factor, paused/running/step-by-step)
- Loads world SDF files (environment, lighting, physics config)
- Handles entity creation/deletion via services

**Entity Manager (ECS Pattern)**:
- Uses Entity-Component-System architecture for performance
- **Entities**: Unique IDs for robots, sensors, lights (e.g., entity ID 42)
- **Components**: Data containers (Pose, Velocity, Name, Model)
- **Systems**: Logic that operates on components (PhysicsSystem, RenderingSystem)

**System Manager**:
- Loads plugins as "systems" (shared libraries `.so` on Linux, `.dll` on Windows)
- Update loop: PreUpdate → Update → PostUpdate at configurable rate (default 1kHz)
- Parallelizes systems with no dependencies for multi-core utilization

### Layer 4: Physics Engine Layer
**Purpose**: Compute forces, torques, collisions, constraint solving

**Physics Abstraction (gz-physics)**:
- Unified API so switching engines requires only SDF config change:
  ```xml
  <physics name="fast_physics" type="bullet">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  ```

**Engine Comparison**:

| Feature | DART | Bullet | ODE |
|---------|------|--------|-----|
| **Default** | ✓ Yes | No | No |
| **Timestep** | 0.001s (1kHz) | 0.002s (500Hz) | 0.005s (200Hz) |
| **Joint Stability** | High | Medium | High |
| **Collision Speed** | Medium | Fast | Slow |
| **Soft Body** | Limited | ✓ Yes | No |
| **GPU Acceleration** | No | Partial | No |
| **Best For** | Articulated robots | Rigid body dynamics | Legacy compatibility |

**Physics Computation**:
1. Collision detection (broad phase → narrow phase)
2. Constraint setup (joint limits, contact normals)
3. LCP solving (Lemke, Dantzig, PGS algorithms)
4. Integration (Euler, Runge-Kutta, semi-implicit)
5. State update (positions, velocities, accelerations)

### Layer 5: Rendering Engine Layer
**Purpose**: Produce visual output for cameras and GUI

**Rendering Abstraction (gz-rendering)**:
- Scene graph with nodes for models, lights, cameras
- Camera types: perspective, orthographic, depth, segmentation
- Frustum culling: only render objects in camera view
- GPU profiling: ~5-15ms per frame at 60 FPS

**OGRE 2 (default)**:
- Modern rendering engine with Physically-Based Rendering (PBR)
- Features: real-time shadows, HDR lighting, GPU instancing
- Supports depth cameras for point cloud generation

**OGRE 1 (legacy)**:
- For older GPUs or lower-spec hardware
- Reduced visual fidelity but higher compatibility

### Layer 6: Plugin Ecosystem Layer
**Purpose**: Extend simulation with custom sensors, actuators, world physics

**Sensor Plugins** (implemented as systems):
- `GpuLidarSystem`: Ray-casting for 3D point clouds, publishes `/lidar` topic
- `DepthCameraSystem`: RGBD images with camera matrix, publishes `/camera/depth/image_raw`
- `ImuSystem`: Accelerometer + gyroscope + magnetometer with noise, publishes `/imu`

**Actuator Plugins**:
- `DiffDriveSystem`: Subscribes to `/cmd_vel`, computes wheel velocities for differential drive
- `JointPositionController`: PID loop for joint tracking, subscribes to `/joint_cmd`
- `GripperPlugin`: Force-based gripper control with tactile feedback

**World Plugins**:
- `WindPlugin`: Applies force field to models based on surface area
- `BuoyancyPlugin`: Simulates underwater physics for marine robots
- `MagnetometerPlugin`: Simulates Earth's magnetic field for compass readings

## Data Flow Example: LiDAR Sensor

```text
1. User spawns robot with LiDAR sensor in SDF:
   <sensor name="lidar" type="gpu_lidar">
     <plugin filename="libgz-sim-gpulidar-system.so" name="gz::sim::GpuLidar">
       <ros><topic>/scan</topic></ros>
     </plugin>
   </sensor>

2. System Manager loads GpuLidarSystem plugin

3. Each physics step (1kHz):
   a. PhysicsSystem updates robot pose
   b. RenderingSystem moves camera to sensor location

4. Each sensor update (10Hz):
   a. GpuLidarSystem triggers ray-casting via OGRE 2
   b. GPU computes 640×16 rays (10,240 points) in ~2ms
   c. Converts depth buffer to LaserScan message

5. GpuLidarSystem publishes to /lidar (gz-transport)

6. ros_gz_bridge converts to sensor_msgs/LaserScan

7. ROS 2 node (e.g., SLAM) subscribes and processes
```

## Usage in Book

- **Referenced in**: Chapter 2 (Gazebo Basics), Chapter 4 (Sensor Simulation)
- **Purpose**: Help learners understand Gazebo as a modular system rather than a monolithic black box
- **Learning Goal**: Recognize which layer to modify when customizing simulation (e.g., sensor plugins for new sensors, physics config for dynamics tuning)

## Key Takeaways

1. **Six-layer architecture**: UI → Transport → Core → Physics → Rendering → Plugins
2. **Pluggable physics engines**: DART (default), Bullet, ODE selectable via SDF
3. **ECS pattern**: Entity-Component-System for performance and modularity
4. **Transport abstraction**: gz-transport + ROS 2 bridge enables hybrid ecosystems
5. **Plugin extensibility**: Add sensors, actuators, world physics without modifying core
6. **Separation of concerns**: Physics at 1kHz, rendering at 60Hz, sensor rates configurable
