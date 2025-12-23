# SDF Example File Template - Module 2

**Purpose**: Standard structure for SDF/XML example files demonstrating Gazebo simulation concepts

## File Header

Every SDF example must start with:

```xml
<?xml version="1.0"?>
<!--
  [Example Name]

  Purpose: [What this example demonstrates in 1-2 sentences]

  Key Concepts:
  - [Concept 1: e.g., "Gazebo world file structure"]
  - [Concept 2: e.g., "Physics engine configuration"]
  - [Concept 3: e.g., "Ground plane with friction"]

  Usage:
  gz sim [filename]

  Expected Behavior:
  [What learner should observe when running this example]

  Prerequisites:
  - Gazebo Harmonic installed
  - ROS 2 Humble or Iron (if using ROS integration)

  Learning Outcomes:
  - [What learner gains from this example]
-->

<sdf version="1.9">
  <!-- Main content here -->
</sdf>
```

## SDF World File Template

```xml
<?xml version="1.0"?>
<!--
  [World Name]

  Purpose: [Demonstrates basic Gazebo world structure]

  Key Concepts:
  - Physics engine configuration
  - Lighting setup
  - Ground plane with surface properties
  - Model inclusion

  Usage:
  gz sim [world_filename.sdf]
-->

<sdf version="1.9">
  <world name="[world_name]">

    <!-- ========================================
         Physics Engine Configuration
         ======================================== -->
    <physics name="dart_physics" type="dart">
      <!-- Time step for physics updates (smaller = more accurate but slower) -->
      <max_step_size>0.001</max_step_size>

      <!-- Real-time factor: 1.0 = real-time, <1.0 = slow motion, >1.0 = fast forward -->
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Gravity vector: [x, y, z] in m/s² -->
    <!-- Earth: [0, 0, -9.81], Moon: [0, 0, -1.62], Mars: [0, 0, -3.71] -->
    <gravity>0 0 -9.81</gravity>


    <!-- ========================================
         Lighting
         ======================================== -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>


    <!-- ========================================
         Ground Plane
         ======================================== -->
    <model name="ground_plane">
      <static>true</static>  <!-- Does not move -->
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>  <!-- Vertical plane -->
            </plane>
          </geometry>
          <!-- Surface friction properties -->
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>    <!-- Coefficient of friction (0-1) -->
                <mu2>0.8</mu2>  <!-- Friction in perpendicular direction -->
              </ode>
            </friction>
          </surface>
        </collision>

        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>  <!-- Plane size in meters -->
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>


    <!-- ========================================
         Additional Models (Obstacles, Props)
         ======================================== -->
    <!-- Add objects here as needed -->

  </world>
</sdf>
```

## SDF Robot Model Template

```xml
<?xml version="1.0"?>
<!--
  [Robot Name] SDF Model

  Purpose: [Demonstrates robot structure in SDF format]

  Key Concepts:
  - SDF model structure
  - Link and joint definitions (similar to URDF)
  - Gazebo-specific plugins (sensors, controllers)
  - Initial pose and collision configuration

  Usage:
  gz sim -s [world_file] --spawn [this_file]

  Or include in world file:
  <include>
    <uri>[this_file_path]</uri>
  </include>
-->

<sdf version="1.9">
  <model name="[robot_name]">

    <!-- Initial pose: x, y, z, roll, pitch, yaw -->
    <pose>0 0 0.5 0 0 0</pose>

    <!-- ========================================
         Link Definitions (similar to URDF)
         ======================================== -->
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.001</iyy><iyz>0</iyz>
          <izz>0.001</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>0.1 0.1 0.1</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
    </link>


    <!-- ========================================
         Joint Definitions
         ======================================== -->
    <joint name="[joint_name]" type="revolute">
      <parent>base_link</parent>
      <child>[child_link]</child>
      <pose>0 0 0.1 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>  <!-- Rotation axis -->
        <limit>
          <lower>-1.57</lower>  <!-- -90 degrees -->
          <upper>1.57</upper>   <!-- +90 degrees -->
          <effort>30</effort>   <!-- Max torque (Nm) -->
          <velocity>1.57</velocity>  <!-- Max angular velocity (rad/s) -->
        </limit>
        <dynamics>
          <damping>0.1</damping>
          <friction>0.01</friction>
        </dynamics>
      </axis>
    </joint>


    <!-- ========================================
         Gazebo Plugins (Sensors, Controllers)
         ======================================== -->
    <!-- Example: Add sensor plugin here -->
    <!--
    <plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    -->

  </model>
</sdf>
```

## SDF Sensor Configuration Template

```xml
<!-- Add to robot model <link> element -->

<!-- ========================================
     LiDAR Sensor Plugin
     ======================================== -->
<sensor name="lidar" type="gpu_lidar">
  <pose>0 0 0.1 0 0 0</pose>  <!-- Sensor position relative to link -->
  <update_rate>10</update_rate>  <!-- Hz -->

  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>     <!-- Horizontal resolution -->
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
        <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
      </horizontal>
      <vertical>
        <samples>16</samples>      <!-- Vertical resolution (16-beam LiDAR) -->
        <resolution>1.0</resolution>
        <min_angle>-0.2618</min_angle>  <!-- -15 degrees -->
        <max_angle>0.2618</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>   <!-- Minimum detection range (m) -->
      <max>30.0</max>  <!-- Maximum detection range (m) -->
      <resolution>0.01</resolution>  <!-- Range resolution (m) -->
    </range>
    <!-- Noise model -->
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm standard deviation -->
    </noise>
  </lidar>

  <!-- ROS 2 integration -->
  <plugin filename="libgz-sim-ros-gz-bridge-system.so" name="gz::sim::systems::RosGzBridge">
    <ros>
      <namespace>/robot</namespace>
      <topic>/lidar/points</topic>
    </ros>
  </plugin>
</sensor>
```

## Annotation Standards

### Inline Comments
- Every non-obvious element must have a comment
- Explain units: (m), (rad), (Nm), (Hz), (kg)
- Explain typical values: "0.001 = 1kHz update for stability"
- Explain purpose: "Why this parameter matters"

### Section Headers
Use XML comments to create visual sections:
```xml
<!-- ========================================
     Section Name
     ======================================== -->
```

### Parameter Explanations
For complex parameters, add multi-line comments:
```xml
<!--
  Parameter Name: [name]
  Purpose: [what it controls]
  Typical Values: [examples with context]
  Trade-offs: [performance vs accuracy considerations]
-->
```

## Validation Checklist

Before marking example complete, verify:

- [ ] Header comment present with purpose, usage, key concepts
- [ ] XML syntax valid (can be parsed without errors)
- [ ] SDF version specified (1.9 for Gazebo Harmonic)
- [ ] All key elements annotated with inline comments
- [ ] Section headers used to organize content
- [ ] Parameters include units where applicable
- [ ] Typical/example values provided for numeric parameters
- [ ] File is runnable in Gazebo Harmonic (if marked as complete example)
- [ ] ROS 2 integration shown (if sensor/controller example)
- [ ] File length appropriate (50-200 lines for readability)

## Common SDF Elements Reference

### Physics Engines
```xml
<physics name="default_physics" type="dart">  <!-- or "bullet", "ode" -->
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

### Surface Properties
```xml
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>   <!-- Friction coefficient -->
      <mu2>0.8</mu2>
    </ode>
  </friction>
  <contact>
    <ode>
      <soft_cfm>0.0</soft_cfm>  <!-- Constraint force mixing -->
      <soft_erp>0.2</soft_erp>  <!-- Error reduction parameter -->
    </ode>
  </contact>
</surface>
```

### Model Inclusion
```xml
<include>
  <uri>model://[model_name]</uri>  <!-- From Gazebo model database -->
  <pose>0 0 0 0 0 0</pose>
</include>
```

### Sensor Types
- `gpu_lidar`: 3D LiDAR point clouds
- `depth_camera`: RGB-D images
- `imu`: Inertial measurement (accel, gyro, orientation)
- `camera`: RGB images only
- `contact`: Touch/force sensors
- `magnetometer`: Magnetic field
- `altimeter`: Altitude/height

## Teaching Notes

### Common Pitfalls to Address
1. **Physics Explosions**: Timestep too large or insufficient damping → robot jitters or explodes
2. **Slow Simulation**: Too many sensors or high-res meshes → real-time factor drops
3. **Invisible Models**: Missing visual geometry or incorrect material → robot not visible
4. **Joint Warnings**: Limits violated → console spam and unrealistic behavior

### Debugging Tips to Include
- Check Gazebo console output for warnings
- Visualize collision geometry (View → Collisions in Gazebo GUI)
- Monitor real-time factor (should be ~1.0 for smooth simulation)
- Use step-by-step mode for debugging physics issues
