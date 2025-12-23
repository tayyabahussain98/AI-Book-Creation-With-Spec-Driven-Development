# URDF vs SDF Comparison Specification

**Purpose**: Compare URDF and SDF formats to help learners choose the right format for their robot description needs

**Diagram Type**: comparison

## Format Comparison: URDF vs SDF

| Aspect | URDF (Unified Robot Description Format) | SDF (Simulation Description Format) |
|--------|------------------------------------------|-------------------------------------|
| **Primary Use Case** | Robot kinematics and structure for ROS | Complete simulation worlds with physics, sensors, environments |
| **Ecosystem** | ROS 1 & ROS 2 standard | Gazebo standard (also used in Isaac Sim, Unity with adapters) |
| **File Extension** | `.urdf` (XML) or `.xacro` (templated URDF) | `.sdf` (XML) or `.world` (SDF world file) |
| **Robot Tree Structure** | Strict tree (single parent per link) | DAG support (multiple parents via `<frame>`) |
| **World Description** | Not supported (robot-only) | Full worlds: terrain, lighting, physics, multiple models |
| **Sensors** | Basic definitions only (no plugins) | Full sensor specs with plugins (GPU LiDAR, depth camera, IMU) |
| **Physics Config** | Not supported | Per-world physics: engine type, timestep, gravity, solver iterations |
| **Closed Kinematic Loops** | Not natively supported | Supported via fixed joints between models |
| **Plugins** | ROS-specific (Gazebo plugins added via `<gazebo>` tags) | Native plugin system for sensors, actuators, world physics |
| **Material/Visual** | Basic colors (`<color rgba="..."/>`) | Advanced: PBR materials, textures, normal maps, emissive surfaces |
| **Versioning** | Informal (ROS distribution-dependent) | Strict versioning (SDF 1.6, 1.7, 1.8, 1.9, 1.10) |
| **Conversion** | Can be converted to SDF via `gz sdf -p <file.urdf>` | SDF → URDF lossy (loses sensors, plugins, world data) |

## Detailed Feature Breakdown

### Robot Tree Structure

**URDF**: Enforces strict tree topology
```xml
<robot name="simple_arm">
  <link name="base_link"/>
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm_link"/>  <!-- Only ONE parent allowed -->
  </joint>
</robot>
```
- **Limitation**: Cannot model closed loops (e.g., parallel mechanisms, Stewart platforms)
- **Workaround**: Break loop with "fake" joint, add constraint in controller

**SDF**: Supports Directed Acyclic Graphs (DAG) via `<frame>` elements
```xml
<model name="parallel_robot">
  <link name="base"/>
  <link name="platform"/>
  <joint name="leg1_joint"><parent>base</parent><child>platform</child></joint>
  <joint name="leg2_joint"><parent>base</parent><child>platform</child></joint>
  <!-- Platform has TWO parents: legal in SDF -->
</model>
```
- **Advantage**: Natively model parallel mechanisms
- **Use Case**: Delta robots, cable-driven systems

### World Description

**URDF**: No concept of "world" or environment
- Robot description exists in isolation
- Must spawn into ROS parameter server, then Gazebo loads it

**SDF**: Complete simulation worlds
```xml
<sdf version="1.9">
  <world name="robot_arena">
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <light name="sun" type="directional">...</light>
    <model name="ground_plane">...</model>
    <model name="my_robot">
      <include><uri>model://robot_arm</uri></include>
      <pose>0 0 1.5 0 0 0</pose>  <!-- Spawn at specific location -->
    </model>
  </world>
</sdf>
```
- Includes terrain, lighting, multiple robots, physics configuration
- Self-contained: `gz sim arena.sdf` launches complete scene

### Sensors

**URDF**: Defines sensor existence, no functionality
```xml
<link name="lidar_link">
  <sensor name="lidar" type="ray">
    <!-- No plugin, no behavior defined -->
  </sensor>
</link>
```
- Gazebo plugins must be added separately via `<gazebo>` extension tags
- No standardized sensor parameters

**SDF**: Full sensor specification with plugins
```xml
<sensor name="lidar" type="gpu_lidar">
  <lidar>
    <scan>
      <horizontal><samples>640</samples><resolution>1</resolution></horizontal>
      <vertical><samples>16</samples><resolution>1</resolution></vertical>
    </scan>
    <range><min>0.05</min><max>100.0</max></range>
  </lidar>
  <update_rate>10</update_rate>
  <plugin filename="libgz-sim-gpulidar-system.so" name="gz::sim::GpuLidar">
    <ros><topic>/scan</topic></ros>
  </plugin>
</sensor>
```
- Sensor behavior fully defined in SDF
- Portable across simulations

### Physics Configuration

**URDF**: No physics engine control
- Physics parameters set globally in Gazebo launch files
- Cannot specify per-robot dynamics preferences

**SDF**: Granular physics control
```xml
<physics name="fast_sim" type="bullet">
  <max_step_size>0.002</max_step_size>
  <real_time_factor>2.0</real_time_factor>  <!-- 2× faster than real-time -->
  <dart>
    <solver><solver_type>dantzig</solver_type></solver>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```
- Choose engine: DART, Bullet, ODE
- Configure timestep, real-time factor, solver algorithms

### Closed Kinematic Loops

**URDF**: Not natively supported
- Must break loop and enforce constraint in software
- Example: Four-bar linkage requires dummy link or external constraint

**SDF**: Supported via fixed joints between models
```xml
<model name="four_bar">
  <link name="ground"/>
  <link name="bar1"/>
  <link name="bar2"/>
  <link name="coupler"/>
  <joint name="j1" type="revolute"><parent>ground</parent><child>bar1</child></joint>
  <joint name="j2" type="revolute"><parent>ground</parent><child>bar2</child></joint>
  <joint name="j3" type="revolute"><parent>bar1</parent><child>coupler</child></joint>
  <joint name="j4" type="revolute"><parent>bar2</parent><child>coupler</child></joint>
  <!-- j4 closes the loop: supported in SDF -->
</model>
```

### Plugins

**URDF**: Requires `<gazebo>` extension tags
```xml
<robot name="diff_drive_robot">
  <link name="base_link">...</link>
  <!-- Standard URDF content -->

  <gazebo>  <!-- Extension for Gazebo-specific plugins -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
    </plugin>
  </gazebo>
</robot>
```
- Plugins embedded via non-standard extension
- Gazebo-specific, breaks portability

**SDF**: Native plugin system
```xml
<model name="diff_drive_robot">
  <plugin name="gz::sim::systems::DiffDrive" filename="gz-sim-diffdrive-system">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.3</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <topic>cmd_vel</topic>
  </plugin>
</model>
```
- First-class plugin support at model, world, sensor levels
- Standardized across Gazebo versions

### Material and Visual Properties

**URDF**: Basic colors only
```xml
<visual>
  <material name="red">
    <color rgba="1.0 0 0 1.0"/>  <!-- Red, fully opaque -->
  </material>
</visual>
```
- No texture support without Gazebo extensions
- No advanced lighting (PBR, normal maps)

**SDF**: Full material system
```xml
<visual name="textured_surface">
  <material>
    <ambient>0.2 0.2 0.2 1</ambient>
    <diffuse>0.8 0.8 0.8 1</diffuse>
    <specular>1.0 1.0 1.0 1</specular>
    <pbr>
      <metal>
        <albedo_map>model://robot/textures/metal_albedo.png</albedo_map>
        <normal_map>model://robot/textures/metal_normal.png</normal_map>
        <metalness>0.9</metalness>
        <roughness>0.3</roughness>
      </metal>
    </pbr>
  </material>
</visual>
```
- PBR (Physically-Based Rendering) materials
- Texture maps: albedo, normal, roughness, metalness
- Emissive surfaces for LEDs, displays

### Versioning

**URDF**: Informal versioning
- No version attribute in XML
- Compatibility determined by ROS distribution (Humble, Iron, Jazzy)
- Breaking changes rare but undocumented

**SDF**: Strict semantic versioning
```xml
<sdf version="1.9">  <!-- Explicit version declaration -->
  <model name="versioned_model">
    <!-- New SDF 1.9 features: particle emitters, custom projectors -->
  </model>
</sdf>
```
- Gazebo validates version compatibility
- Migration guides for SDF 1.6 → 1.7 → 1.8 → 1.9 → 1.10
- Backward compatibility maintained

## Conversion Between Formats

### URDF → SDF (Lossless for Robot Structure)
```bash
gz sdf -p robot.urdf > robot.sdf
```
- Preserves links, joints, inertia, visual/collision geometry
- Plugins in `<gazebo>` tags converted to SDF `<plugin>` elements
- Adds default physics properties if missing

### SDF → URDF (Lossy)
```bash
# No official tool; manual extraction required
```
- Loses: world data, sensors, physics config, plugins
- Retains: link/joint kinematic tree (if tree-structured)
- Use case: Extract robot from SDF world for ROS MoveIt planning

## When to Use Each Format

### Use URDF When:
1. ✅ Working exclusively in ROS 2 ecosystem (Navigation2, MoveIt2)
2. ✅ Robot has strict tree structure (no closed loops)
3. ✅ No simulation required (e.g., motion planning only)
4. ✅ Need compatibility with ROS 1 (Kinetic, Melodic, Noetic)
5. ✅ Using xacro for templating and parameterization

**Example Scenario**: Mobile manipulator with tree-structured arm, using MoveIt for motion planning, no simulation.

### Use SDF When:
1. ✅ Simulating in Gazebo (sensors, physics, world environments)
2. ✅ Modeling closed kinematic loops (parallel robots, cable systems)
3. ✅ Need advanced sensors (GPU LiDAR, depth cameras with full specs)
4. ✅ Configuring physics engines (DART, Bullet, ODE)
5. ✅ Defining multi-robot worlds with terrain and lighting
6. ✅ Porting to other simulators (Isaac Sim supports SDF via USD)

**Example Scenario**: Quadruped robot in rough terrain with LiDAR and depth camera, testing SLAM algorithms in Gazebo.

### Hybrid Approach (Best of Both):
1. Maintain canonical robot in URDF (for ROS compatibility)
2. Convert to SDF and augment with sensors/plugins for simulation
3. Use `ros_gz_bridge` to interface Gazebo (SDF world) with ROS 2 stack

```bash
# Workflow
1. Design robot in URDF (with xacro macros)
2. Convert: gz sdf -p robot.urdf > robot.sdf
3. Enhance robot.sdf: add sensor plugins, physics materials
4. Create world.sdf that includes augmented robot.sdf
5. Launch: gz sim world.sdf & ros_gz_bridge
```

## Usage in Book

- **Referenced in**: Chapter 3 (Robot Models), Chapter 4 (Sensor Simulation)
- **Purpose**: Help learners make informed decisions about format choice and understand when to use each
- **Learning Goal**: Recognize that URDF and SDF serve complementary roles: URDF for ROS integration, SDF for comprehensive simulation

## Key Takeaways

1. **URDF**: ROS-standard format for robot kinematics, tree-structured, no simulation features
2. **SDF**: Gazebo-standard format for complete worlds with sensors, physics, terrain, multiple robots
3. **Conversion**: URDF → SDF lossless for structure; SDF → URDF loses simulation data
4. **Hybrid workflow**: Maintain URDF for ROS, augment SDF for simulation
5. **Version discipline**: SDF has strict versioning (1.9, 1.10); URDF is informal
6. **Complexity tradeoff**: URDF simple for basic robots; SDF powerful but more verbose
7. **Closed loops**: SDF supports natively; URDF requires workarounds
8. **Sensors**: SDF fully specifies behavior; URDF only declares existence
