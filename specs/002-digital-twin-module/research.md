# Research Notes: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-module
**Date**: 2025-12-22
**Purpose**: Research Gazebo simulation, SDF format, sensor plugins, and Unity ROS 2 integration for educational content creation

## Research Areas

### 1. Gazebo Simulation Platform

**Decision**: Use Gazebo Harmonic (latest LTS as of 2024-2025) for all examples

**Rationale**:
- Gazebo Harmonic is the newest LTS release with modern architecture (Ignition rebranding complete)
- Native ROS 2 integration via gz_ros2_control and sensor plugins
- Improved physics engine (DART, Bullet, or ODE selectable)
- Better performance and stability than Gazebo Classic
- Active development and community support

**Alternatives Considered**:
- **Gazebo Garden**: Previous LTS, but Harmonic has better ROS 2 Humble/Iron compatibility
- **Gazebo Classic (Gazebo 11)**: Deprecated, ROS 1 focused, poor ROS 2 support
- **Isaac Sim**: Excellent but requires NVIDIA GPU and proprietary license (not free-tier compatible)

**Key Capabilities for Module 2**:
- SDF (Simulation Description Format) world files
- Physics engines: DART (default), Bullet, ODE
- Sensor plugins: GPU LiDAR, depth camera, IMU, contact sensors
- ROS 2 integration via gz-sim plugins
- Visual and collision geometry rendering
- Real-time factor control and step-based simulation

### 2. SDF (Simulation Description Format) vs URDF

**Decision**: Teach both formats with clear distinction - URDF for robot structure, SDF for Gazebo-specific features

**Rationale**:
- URDF: Universal robot description (ROS standard), focuses on kinematics
- SDF: Extends URDF with Gazebo-specific elements (sensors, plugins, physics properties)
- Conversion workflow: URDF → SDF for Gazebo compatibility
- SDF supports features URDF cannot: multiple models, world environments, sensor plugins

**Key Differences to Teach**:
1. **Scope**: URDF = robot only, SDF = robot + world + sensors
2. **Plugins**: SDF supports Gazebo plugins (sensors, controllers), URDF does not
3. **World Files**: SDF can describe entire environments, URDF only describes robots
4. **Physics**: SDF has advanced physics parameters (surface friction models, contact properties)
5. **Sensors**: SDF natively supports sensor definitions with noise models

**Teaching Approach**:
- Chapter 3 explains conversion: `gz sdf -p robot.urdf > robot.sdf`
- Show side-by-side comparison of URDF vs SDF for same robot
- Highlight when to use each format

### 3. Gazebo Sensor Plugins for ROS 2

**Decision**: Focus on 3 essential sensors for AI perception - LiDAR, Depth Camera, IMU

**Rationale**:
- LiDAR: 3D point clouds for navigation and SLAM (Simultaneous Localization and Mapping)
- Depth Camera: RGB-D images for object detection and manipulation
- IMU: Orientation and acceleration for state estimation and control
- These 3 cover primary perception modalities for humanoid robotics

**Plugin Details**:

**LiDAR (GPU Ray Sensor)**:
- Plugin: `libgz-sim-sensors-system.so` with `<sensor type="gpu_lidar">`
- ROS 2 integration: `ros_gz_bridge` publishes PointCloud2 messages
- Parameters: range, resolution, horizontal/vertical FoV, update rate
- Noise models: Gaussian noise on range measurements

**Depth Camera**:
- Plugin: `libgz-sim-sensors-system.so` with `<sensor type="depth_camera">`
- ROS 2 topics: /camera/image_raw, /camera/depth/image_raw, /camera/camera_info
- Parameters: resolution, FoV, near/far clip planes, update rate
- Noise: Gaussian noise on depth values, image distortion models

**IMU (Inertial Measurement Unit)**:
- Plugin: `libgz-sim-sensors-system.so` with `<sensor type="imu">`
- ROS 2 topic: /imu/data (sensor_msgs/Imu)
- Outputs: Linear acceleration, angular velocity, orientation (quaternion)
- Noise: Gaussian noise on accel/gyro, bias drift models

**Alternatives Considered**:
- Camera (RGB only): Simpler but less useful for robotics (no depth)
- Force-torque sensors: Advanced, deferred to manipulation modules
- GPS: Outdoor navigation, not relevant for indoor humanoids

### 4. Unity for Human-Robot Interaction

**Decision**: Use Unity 2022 LTS with Unity Robotics Hub for Chapter 5

**Rationale**:
- Unity Robotics Hub: Official Unity package for ROS/ROS 2 integration
- Unity 2022 LTS: Long-term support, stable, free for personal/educational use
- Photorealistic rendering for human-robot interaction scenarios
- Asset store for human models, environments (homes, offices)
- Cross-platform (Windows, Mac, Linux)

**Unity Robotics Hub Components**:
- **ROS-TCP-Connector**: TCP/IP bridge between Unity and ROS 2
- **URDF Importer**: Import robot models from URDF files into Unity
- **ArticulationBody**: Unity's physics component for robot joints (maps to ROS joint states)

**Integration Workflow**:
1. Install Unity Robotics Hub via Package Manager
2. Import URDF using URDF Importer tool
3. Configure ROS-TCP-Connector with ROS 2 endpoint
4. Subscribe to /joint_states topic to animate robot in Unity
5. Publish sensor data or interaction events back to ROS 2

**Alternatives Considered**:
- **Unreal Engine**: High fidelity but more complex, steeper learning curve
- **Isaac Sim**: Best physics but requires NVIDIA GPU, not free-tier
- **Web-based (Three.js)**: Lightweight but limited physics and asset ecosystem

### 5. Educational Content Structure (Following Module 1 Pattern)

**Decision**: Mirror Module 1 structure for consistency

**Chapter Structure** (each ~600-800 lines):
1. Introduction (prerequisites, learning objectives)
2. Core Concept 1 (fundamental theory)
3. Core Concept 2 (key mechanism)
4. Core Concept 3 (advanced topic)
5. Core Concept 4-5 (additional concepts as needed)
6. Hands-On Example (embedded code with annotations)
7. Diagrams section (references to diagram/ files)
8. Summary (6-bullet recap)
9. Self-Assessment Checklist (6 items)

**Diagram Specifications** (following Module 1 pattern):
- ASCII art for workflows and architecture
- Markdown tables for comparisons
- Text-based specifications (no images or screenshots)
- Each diagram in separate .md file in diagrams/module-2/

**Code Examples**:
- SDF files for robot models and world environments
- Sensor configuration snippets
- Unity C# script excerpts (minimal, concept-focused)
- All examples tested and runnable

**Teaching Progression**:
- Week 1: Conceptual foundation (digital twins, tool selection)
- Week 2: Hands-on Gazebo (physics, environments)
- Week 3: Robot modeling (URDF/SDF, tuning)
- Week 4: Perception (sensors, noise, ROS 2 integration)
- Week 5: Visualization (Unity, human-robot interaction)

### 6. Gazebo World File Best Practices

**Decision**: Provide reusable world file templates with annotated sections

**Essential Elements**:
```xml
<sdf version="1.9">
  <world name="humanoid_test_env">
    <!-- Physics engine configuration -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting (ambient + directional sun) -->
    <light type="directional" name="sun">...</light>

    <!-- Ground plane with friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
          <surface><friction><ode><mu>0.8</mu></ode></friction></surface>
        </collision>
      </link>
    </model>

    <!-- Obstacles, stairs, etc. -->
  </world>
</sdf>
```

**Key Parameters**:
- **max_step_size**: 0.001s (1kHz physics update for stability)
- **real_time_factor**: 1.0 (real-time), can slow down for debugging
- **Gravity**: Default [0, 0, -9.81] m/s² (Earth), can modify for lunar/Martian scenarios
- **Surface friction**: Typically 0.8-1.0 for indoor floors, 0.3-0.5 for slippery surfaces

### 7. Humanoid Stability Metrics

**Decision**: Teach Zero-Moment Point (ZMP) and Center of Mass (CoM) concepts for stability analysis

**Rationale**:
- ZMP: Industry-standard metric for humanoid balance (point where ground reaction force acts)
- CoM: Critical for understanding toppling and dynamic stability
- Essential for debugging why simulated humanoids fall or cannot walk

**Key Concepts to Cover**:
1. **Static Stability**: CoM must project inside support polygon (convex hull of contact points)
2. **Dynamic Stability**: ZMP must remain inside support polygon during motion
3. **Support Polygon**: Changes as feet lift (single support vs double support)
4. **Margin of Stability**: Distance from ZMP to polygon edge

**Teaching Approach**:
- Chapter 2: Conceptual explanation with diagrams
- Show ZMP visualization in Gazebo (if plugin available)
- Explain why narrow feet or high CoM leads to instability

### 8. Unity-ROS 2 Integration Architecture

**Decision**: Use Unity Robotics Hub ROS-TCP-Connector with ROS 2

**Architecture**:
```
Unity (C#)                    ROS 2 (Python/C++)
┌─────────────┐              ┌──────────────┐
│ Robot       │◄─TCP/IP───►  │ ros_tcp_     │
│ Visualizer  │   (50051)    │ endpoint     │
│             │              │              │
│ Articulation│◄─/joint_    ─┤ Joint State  │
│ Body        │   states      │ Publisher    │
│             │              │              │
│ Sensor      ├─/camera/    ─►│ Perception   │
│ Publisher   │   image       │ Pipeline     │
└─────────────┘              └──────────────┘
```

**Components**:
1. **Unity Side**:
   - ROS-TCP-Connector asset (Unity package)
   - ROSConnection MonoBehaviour (manages TCP connection)
   - MessagePublisher/Subscriber scripts (send/receive ROS messages)

2. **ROS 2 Side**:
   - ros_tcp_endpoint package (ROS 2 node that bridges TCP to ROS topics)
   - Standard ROS 2 nodes (publishers, subscribers)

**Data Flow**:
- Unity → ROS 2: Sensor data, interaction events, camera frames
- ROS 2 → Unity: Joint commands, visualization markers, robot state

**Latency Considerations**:
- TCP adds ~10-50ms latency (acceptable for visualization, not for control)
- Unity update rate: 30-60 FPS (sufficient for visualization)
- ROS 2 control rate: 100-1000 Hz (runs separately in Gazebo or on hardware)

## Implementation Decisions

### File Naming Conventions

**Chapters**: `chapter-X-<topic-slug>.md`
- chapter-1-foundations.md
- chapter-2-gazebo-physics.md
- chapter-3-robot-models.md
- chapter-4-sensor-simulation.md
- chapter-5-unity-hri.md

**Diagrams**: `<concept-description>.md` (descriptive names)
- digital-twin-lifecycle.md
- gazebo-architecture.md
- urdf-vs-sdf-comparison.md
- sensor-plugin-dataflow.md
- unity-ros-bridge-architecture.md

**Code Examples**: `<purpose>_<detail>.sdf` or `.xml`
- simple_world.sdf (basic environment)
- humanoid_stable.sdf (balanced biped model)
- lidar_sensor.sdf (LiDAR configuration)
- depth_camera.sdf (RGB-D sensor)
- imu_config.sdf (IMU with noise)

### Sidebar Configuration Update

**Location**: `book_frontend/sidebars.ts`

**Addition** (insert after Module 1):
```typescript
{
  type: 'category',
  label: 'Module 2: The Digital Twin (Gazebo & Unity)',
  items: [
    'module-2-digital-twin/chapter-1-foundations',
    'module-2-digital-twin/chapter-2-gazebo-physics',
    'module-2-digital-twin/chapter-3-robot-models',
    'module-2-digital-twin/chapter-4-sensor-simulation',
    'module-2-digital-twin/chapter-5-unity-hri',
  ],
  collapsed: false,
},
```

### Chapter Content Guidelines

**Each chapter includes**:
1. Front matter (sidebar_position, title)
2. Introduction (prerequisites, objectives)
3. 3-5 Core Concepts (conceptual explanations)
4. Hands-On Example (embedded SDF/config with annotations)
5. Diagram references (ASCII art, tables, workflows)
6. Summary (6 bullets)
7. Self-Assessment Checklist (6 items)

**Tone**: Educational, beginner-friendly, concept-first before technical details

**Code Style**: Heavily annotated XML/SDF with inline comments explaining each element

### Diagram Specifications Required

Based on functional requirements and user stories:

1. **digital-twin-lifecycle.md**: Full lifecycle (design → simulate → test → deploy → sync)
2. **gazebo-architecture.md**: Gazebo components (physics engine, rendering, plugins, ROS 2 bridge)
3. **urdf-vs-sdf-comparison.md**: Side-by-side comparison table of formats
4. **sdf-world-structure.md**: Anatomy of a Gazebo world file (physics, models, lighting)
5. **sensor-plugin-flow.md**: Data flow from Gazebo sensor plugin → ROS 2 topic → perception node
6. **lidar-raycasting.md**: How LiDAR simulation works (ray casting, point cloud generation)
7. **depth-camera-rendering.md**: Depth image generation and encoding
8. **imu-noise-model.md**: Accelerometer/gyro noise characteristics (Gaussian, bias drift)
9. **unity-ros-bridge.md**: Unity-ROS 2 architecture (TCP connector, message serialization)
10. **hri-interaction-patterns.md**: Common human-robot interaction scenarios (object handoff, gesture recognition)

### Example Files to Create

**Gazebo Examples** (code-examples/module-2/):
1. **simple_world.sdf**: Minimal world with ground plane, lighting, gravity
2. **humanoid_stable.sdf**: Balanced bipedal robot with proper CoM and contact properties
3. **obstacles_world.sdf**: Environment with stairs, ramps, obstacles for navigation testing
4. **lidar_robot.sdf**: Robot model with LiDAR sensor plugin and ROS 2 bridge
5. **depth_camera_robot.sdf**: Robot with RGB-D camera sensor
6. **imu_config.sdf**: IMU sensor configuration with realistic noise parameters

**Unity Examples**:
- unity_bridge_config.xml: ROS-TCP-Connector configuration (IP, port, topics)
- Sample C# script snippet for joint control (inline in Chapter 5, not full file)

## Technical Constraints

### Gazebo System Requirements

**Minimum**:
- Ubuntu 22.04 or 24.04
- 4GB RAM
- Integrated graphics (Intel/AMD)
- 10GB disk space

**Recommended**:
- Ubuntu 24.04
- 8GB+ RAM
- NVIDIA GPU (for sensor rendering performance)
- SSD for faster scene loading

### Unity System Requirements

**Minimum**:
- Windows 10, macOS 10.15, or Ubuntu 20.04+
- 8GB RAM
- DirectX 11/Metal/Vulkan graphics
- 20GB disk space (Unity Editor + assets)

**Recommended**:
- 16GB+ RAM
- Dedicated GPU (NVIDIA/AMD)
- SSD

### ROS 2 Integration

**Required Packages**:
- **ros-humble-ros-gz** or **ros-iron-ros-gz**: Gazebo-ROS 2 bridge
- **ros-humble-ros-gz-sim**: Gazebo simulation integration
- **ros-humble-ros-gz-bridge**: Topic/service/action bridging
- **Unity Robotics Hub**: Install via Unity Package Manager (com.unity.robotics.ros-tcp-connector)

**Installation Commands** (to document in chapters):
```bash
# Gazebo Harmonic (Ubuntu 24.04)
sudo apt install gz-harmonic

# ROS 2 Gazebo integration
sudo apt install ros-humble-ros-gz

# Unity Robotics Hub
# (installed via Unity Package Manager GUI)
```

## Teaching Strategy

### Progression Logic

**Chapter 1 → 2**: Conceptual understanding → Hands-on environment creation
**Chapter 2 → 3**: Environment setup → Robot model integration
**Chapter 3 → 4**: Robot structure → Sensor data generation
**Chapter 4 → 5**: Gazebo physics → Unity visualization
**Chapter 5 → Module 3**: Simulation → AI control algorithms (future)

### Conceptual Anchors

**Digital Twin Analogy**: "Digital twin is to a robot what a flight simulator is to an aircraft—a virtual environment for testing before real-world deployment"

**Reality Gap**: Emphasize limitations (friction models approximate, motor delays absent, sensor noise simplified) so learners understand simulation ≠ reality

**Tool Selection Heuristic**:
- **Need accurate physics?** → Gazebo
- **Need beautiful visuals?** → Unity
- **Need both?** → Use both (Gazebo for physics, Unity for visualization)

## Research Sources

**Gazebo Documentation**:
- https://gazebosim.org/docs/harmonic/tutorials
- SDF Specification: http://sdformat.org/spec
- Sensor plugins: https://gazebosim.org/api/sensors/

**Unity Robotics Hub**:
- https://github.com/Unity-Technologies/Unity-Robotics-Hub
- ROS-TCP-Connector docs: https://github.com/Unity-Technologies/ROS-TCP-Connector

**ROS 2 Gazebo Integration**:
- ros_gz packages: https://github.com/gazebosim/ros_gz
- Bridge configuration: https://gazebosim.org/docs/harmonic/ros2_integration

**Humanoid Stability**:
- ZMP theory: Standard robotics textbooks (e.g., Kajita et al., "Introduction to Humanoid Robotics")
- Balance control: Open-source humanoid projects (e.g., Atlas, Valkyrie documentation)

## Open Questions (Resolved)

**Q: Which Gazebo version?**
- **Resolved**: Gazebo Harmonic (latest LTS, best ROS 2 support)

**Q: URDF or SDF for examples?**
- **Resolved**: Both - teach URDF→SDF conversion workflow, use SDF for Gazebo-specific examples

**Q: How many sensors to cover?**
- **Resolved**: 3 essential sensors (LiDAR, depth camera, IMU) - sufficient for AI perception pipelines

**Q: Unity integration depth?**
- **Resolved**: Conceptual overview + basic setup (Chapter 5). Full Unity development deferred to advanced modules.

**Q: Code examples vs concept-only?**
- **Resolved**: Minimal examples (SDF files, world files) with heavy annotation. Concept-first per user constraint.

## Next Steps

- **Phase 1**: Create data-model.md (chapter/diagram entity structure), contracts/ (templates), quickstart.md (development workflow)
- **Post-Planning**: Run `/sp.tasks` to generate task breakdown for implementation
- **Implementation**: Follow Module 1 approach (phases: setup, diagrams, chapters, polish)
