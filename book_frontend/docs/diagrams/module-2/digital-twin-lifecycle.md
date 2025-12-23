# Digital Twin Lifecycle Specification

**Purpose**: Illustrate the complete workflow from physical robot design to synchronized digital twin simulation

**Diagram Type**: workflow

## Digital Twin Development Workflow

```text
┌────────────────────────────────────────────────────────────────────┐
│                    DIGITAL TWIN LIFECYCLE                          │
└────────────────────────────────────────────────────────────────────┘

Phase 1: DESIGN
┌──────────────┐
│  CAD Model   │  → Create 3D geometry (Fusion 360, SolidWorks, Blender)
│  (.stl, .dae)│  → Define link shapes, mass properties, joint axes
└──────┬───────┘
       │
       ↓
Phase 2: DESCRIBE
┌──────────────┐
│ URDF/SDF File│  → Convert CAD to robot description format
│  (.urdf, .sdf)│  → Add visual/collision geometry, inertia tensors
└──────┬───────┘  → Specify joint types, limits, dynamics
       │
       ↓
Phase 3: SIMULATE
┌──────────────┐
│    Gazebo    │  → Load robot model into physics engine
│  Simulation  │  → Configure world environment (gravity, friction)
└──────┬───────┘  → Attach sensor plugins (LiDAR, camera, IMU)
       │
       ↓
Phase 4: VALIDATE
┌──────────────┐
│   Testing    │  → Run test scenarios (navigation, manipulation)
│   & Tuning   │  → Compare sim behavior to physical expectations
└──────┬───────┘  → Tune PID gains, friction coefficients, noise models
       │
       ↓
Phase 5: DEPLOY
┌──────────────┐
│  ROS 2 Bridge│  → Launch ros_gz_bridge for topic/service mapping
│   & Control  │  → Connect controllers (joint_state_controller, etc.)
└──────┬───────┘  → Start teleoperation or autonomous navigation stack
       │
       ↓
Phase 6: SYNCHRONIZE
┌──────────────┐
│ Physical ←→  │  → Real robot publishes sensor data to ROS 2 topics
│   Digital    │  → Digital twin subscribes and mirrors joint states
└──────────────┘  → Bidirectional: predict outcomes before physical execution
       │
       ↓
       ⟲ Iterate: Refine model based on real-world discrepancies
```

## Detailed Phase Breakdown

### Phase 1: DESIGN (CAD Modeling)
**Tools**: Fusion 360, SolidWorks, Blender, FreeCAD

Create 3D meshes for each robot link (base, arms, grippers, wheels, etc.). Export to formats that preserve scale and orientation:
- **STL** (Standard Tessellation Language): Lightweight, widely supported
- **DAE** (COLLADA): Preserves materials, colors, coordinate systems

**Key Considerations**:
- Simplify meshes for collision geometry (reduce polygon count)
- Model center of mass (CoM) location accurately for balance calculations
- Define coordinate frames: X-forward, Y-left, Z-up (ROS convention)

### Phase 2: DESCRIBE (URDF/SDF Authoring)
**Output**: `.urdf` or `.sdf` file

Transform CAD geometry into robot description format:
- **URDF** (Unified Robot Description Format): ROS standard, defines kinematic tree
- **SDF** (Simulation Description Format): Superset of URDF, adds Gazebo-specific features (sensors, plugins, worlds)

**URDF Structure**:
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>      <!-- What you see -->
    <collision>   <!-- What collides -->
    <inertial>    <!-- Mass & inertia tensor -->
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <axis xyz="0 1 0"/>  <!-- Y-axis rotation -->
    <limit effort="10" velocity="5"/>
  </joint>
</robot>
```

**SDF Extensions**:
- Sensor plugins (GPU LiDAR, depth camera)
- Actuator controllers (position, velocity, effort)
- World physics config (timestep, solver iterations)

### Phase 3: SIMULATE (Gazebo Execution)
**Command**: `gz sim <world_file>.sdf`

Launch physics simulation:
1. Load robot model from URDF/SDF
2. Spawn in world environment (empty.sdf, house.sdf, warehouse.sdf)
3. Physics engine (DART, Bullet, ODE) computes forces, torques, collisions at 1kHz
4. Rendering engine displays 3D visualization at 60Hz

**Plugin Activation**:
- `libgazebo_ros_joint_state_publisher.so` → publishes `/joint_states`
- `libgazebo_ros_gpu_lidar.so` → publishes `/scan` (sensor_msgs/LaserScan)
- `libgazebo_ros_diff_drive.so` → subscribes to `/cmd_vel` (geometry_msgs/Twist)

### Phase 4: VALIDATE (Testing & Tuning)
**Goal**: Close the reality gap

Compare simulated behavior to physical robot expectations:
- **Kinematics**: Joint angles match commanded positions within ±2°
- **Dynamics**: Acceleration profiles realistic (no instant velocity changes)
- **Sensors**: Noise models reflect real hardware (e.g., LiDAR ±3cm accuracy)

**Common Adjustments**:
- Increase joint friction (`<friction>` in `<joint>`)
- Tune PID controller gains (`<pid>` in `ros2_control` config)
- Add Gaussian noise to sensors (`<noise>` in `<sensor>`)
- Adjust contact surface properties (`<mu1>`, `<mu2>` for friction)

### Phase 5: DEPLOY (ROS 2 Integration)
**Tools**: `ros_gz_bridge`, `ros2_control`

Bridge Gazebo topics to ROS 2 ecosystem:
```yaml
# ros_gz_bridge config
- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
```

Launch controllers:
```bash
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller diff_drive_controller
```

### Phase 6: SYNCHRONIZE (Physical-Digital Loop)
**Architecture**: Physical Robot ↔ ROS 2 ↔ Digital Twin

**Unidirectional Mirroring** (most common):
- Physical robot publishes sensor data (`/odom`, `/joint_states`, `/scan`)
- Digital twin subscribes and updates visualization
- Use case: Remote monitoring, debugging, training visualization

**Bidirectional Sync** (advanced):
- Digital twin runs ahead in "prediction mode"
- Simulates outcome of planned action (e.g., grasp trajectory)
- If safe, physical robot executes; if collision detected, replan
- Use case: Motion planning validation, safety checking

**Sync Rate**: 10-50 Hz typical (balance between latency and bandwidth)

## Iteration and Refinement

The lifecycle is **iterative**, not linear:
- Discover sim-to-real gap in Phase 6 → return to Phase 4 for tuning
- Add new sensor in Phase 2 → revalidate in Phases 3-5
- Physical robot modified → update CAD in Phase 1 → propagate changes

## Usage in Book

- **Referenced in**: Chapter 1 (Introduction), Chapter 2 (Gazebo Basics)
- **Purpose**: Provide learners with a mental model of the end-to-end workflow before diving into technical details
- **Learning Goal**: Understand that digital twins are not one-time creations but continuously refined artifacts synchronized with physical systems

## Key Takeaways

1. **Six distinct phases**: Design → Describe → Simulate → Validate → Deploy → Synchronize
2. **Iterative process**: Refinement based on sim-to-real discrepancies is essential
3. **Format bridge**: CAD (.stl) → URDF/SDF → Gazebo → ROS 2 topics
4. **Reality gap closure**: Tuning friction, noise, and dynamics parameters reduces simulation error
5. **Bidirectional potential**: Digital twins can predict outcomes before physical execution
