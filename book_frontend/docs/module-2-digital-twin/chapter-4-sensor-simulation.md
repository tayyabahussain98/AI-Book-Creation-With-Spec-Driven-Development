---
sidebar_position: 4
title: Chapter 4 - Sensor Simulation
---

# Chapter 4: Sensor Simulation

## Introduction

In the previous chapter, you learned to import and tune robot models in Gazebo. A robot body without sensors, however, is like a human without sight, hearing, or touch—unable to perceive or interact with its environment. This chapter introduces **sensor simulation**, the critical bridge between your robot's digital twin and the perception algorithms that will power autonomous behavior.

### Prerequisites

- Completed Chapter 3 (robot models in Gazebo)
- Module 1, Chapters 6-8 (ROS 2 topics and messages)
- Familiarity with `sensor_msgs` message types

### Learning Objectives

By the end of this chapter, you will be able to:

1. Configure LiDAR sensors with appropriate resolution and range settings
2. Set up depth cameras with correct intrinsic parameters
3. Add IMU sensors with realistic noise models
4. Understand why sensor noise is essential for robust AI training
5. Bridge simulated sensors to ROS 2 topics for perception pipelines
6. Visualize sensor data in RViz2

---

## Core Concept 1: LiDAR Simulation

**LiDAR** (Light Detection and Ranging) sensors are the "eyes" of many autonomous robots. They emit laser beams in a scanning pattern and measure the time-of-flight to calculate distances to surrounding objects.

### Ray-Casting: How Simulated LiDAR Works

In Gazebo, LiDAR sensors use **GPU-accelerated ray-casting** to generate point clouds:

```text
                    LiDAR Sensor
                         │
                         ▼
         ─────────────────●─────────────────
        ╱  ╱  ╱  ╱  ╱  ╱  │  ╲  ╲  ╲  ╲  ╲  ╲
       ╱  ╱  ╱  ╱  ╱  ╱   │   ╲  ╲  ╲  ╲  ╲  ╲
      ●  ●  ●  ●  ●  ●    │    ●  ●  ●  ●  ●  ●
     Hit Hit Hit Hit     No hit    Hit Hit Hit Hit
                      (max range)
```

For each ray:
1. Cast from sensor origin in computed direction
2. Travel until collision with world geometry or maximum range
3. Return distance and intensity values
4. Convert to 3D point in sensor frame

### Resolution vs. Performance Trade-offs

The number of rays directly impacts both perception quality and simulation speed:

| Configuration | Rays/Scan | GPU Time | Use Case |
|---------------|-----------|----------|----------|
| LOW (180 × 1) | 180 | ~0.5 ms | 2D SLAM, fast prototyping |
| MEDIUM (360 × 8) | 2,880 | ~2 ms | Basic 3D perception |
| HIGH (640 × 16) | 10,240 | ~5 ms | Velodyne VLP-16 equivalent |
| ULTRA (1024 × 64) | 65,536 | ~20 ms | High-end autonomous vehicles |

:::tip Training Tip
For AI training, start with lower resolution during algorithm development, then increase resolution for final training runs. This dramatically reduces iteration time.
:::

### LiDAR Configuration in SDF

Here's the key sensor configuration from a LiDAR-equipped robot:

```xml
<sensor name="gpu_lidar" type="gpu_lidar">
  <always_on>true</always_on>
  <update_rate>10</update_rate>  <!-- 10 Hz typical for LiDAR -->
  <topic>lidar</topic>
  <visualize>true</visualize>

  <lidar>
    <scan>
      <!-- Horizontal: 360° scan with 640 samples -->
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -π -->
        <max_angle>3.14159</max_angle>   <!-- +π -->
      </horizontal>
      <!-- Vertical: 16 channels spanning ±15° -->
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle>   <!-- -15° -->
        <max_angle>0.2618</max_angle>    <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>   <!-- 10cm minimum (avoid self-collision) -->
      <max>30.0</max>  <!-- 30m maximum range -->
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.03</stddev>  <!-- ±3cm noise typical -->
    </noise>
  </lidar>
</sensor>
```

### ROS 2 Topic Output

The LiDAR publishes to a `sensor_msgs/msg/PointCloud2` topic after bridging:

```bash
# Bridge LiDAR from Gazebo to ROS 2
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked

# View topic info
ros2 topic info /lidar

# Echo first message
ros2 topic echo /lidar --once
```

See the complete LiDAR robot configuration in [lidar_robot.sdf](../code-examples/module-2/lidar_robot.sdf) and the detailed ray-casting explanation in the [LiDAR Ray-Casting Diagram](../diagrams/module-2/lidar-raycasting.md).

---

## Core Concept 2: Depth Camera Simulation

**Depth cameras** (RGB-D sensors) provide per-pixel distance information alongside color images. They're essential for object detection, manipulation, and dense 3D reconstruction.

### Depth Image Generation Pipeline

Unlike LiDAR's discrete rays, depth cameras render a complete depth buffer using the GPU graphics pipeline:

```text
Scene Geometry → Vertex Shader → Rasterizer → Fragment Shader → Depth Buffer
                 (transform)     (project)    (compute depth)    (640×480 floats)
```

Each pixel in the depth image contains the distance (in meters) from the camera to the nearest surface.

### Camera Intrinsic Parameters

Understanding camera intrinsics is crucial for converting depth images to 3D points:

```text
INTRINSIC MATRIX K:

    ┌            ┐
    │ fx  0  cx  │     fx, fy = focal length (pixels)
K = │ 0  fy  cy  │     cx, cy = principal point (image center)
    │ 0   0   1  │
    └            ┘

For 640×480 with 60° horizontal FOV:
  fx = 640 / (2 × tan(30°)) ≈ 554.26 pixels
  cx = 320, cy = 240
```

### Depth to 3D Point Conversion

Given pixel coordinates (u, v) and depth value d:

```text
X = (u - cx) × d / fx
Y = (v - cy) × d / fy
Z = d

Example: Pixel (400, 300) with depth 2.5m:
  X = (400 - 320) × 2.5 / 554.26 = 0.361 m
  Y = (300 - 240) × 2.5 / 554.26 = 0.271 m
  Z = 2.5 m
```

### Depth Camera Configuration in SDF

```xml
<sensor name="depth_camera" type="depth_camera">
  <always_on>true</always_on>
  <update_rate>30</update_rate>  <!-- 30 Hz typical for RGB-D -->
  <topic>camera/depth</topic>

  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>  <!-- 32-bit float depth -->
    </image>
    <clip>
      <near>0.1</near>   <!-- 10cm minimum -->
      <far>10.0</far>    <!-- 10m maximum -->
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- 5mm base noise -->
    </noise>
  </camera>
</sensor>
```

### Depth Encoding Formats

| Format | Bits/Pixel | Range | Notes |
|--------|------------|-------|-------|
| 32FC1 (float) | 32 | 0 - ∞ meters | Recommended; NaN for invalid |
| 16UC1 (uint16) | 16 | 0 - 65.535m (mm) | Smaller bandwidth |

:::warning Real-World Difference
Simulated depth cameras have **ideal RGB-depth alignment**. Real sensors (Intel RealSense, Kinect) have a physical offset between sensors, causing alignment issues at depth discontinuities.
:::

See the complete depth camera robot in [depth_camera_robot.sdf](../code-examples/module-2/depth_camera_robot.sdf) and the detailed rendering pipeline in the [Depth Camera Rendering Diagram](../diagrams/module-2/depth-camera-rendering.md).

---

## Core Concept 3: IMU Simulation

**Inertial Measurement Units (IMUs)** measure linear acceleration and angular velocity. They're critical for state estimation, balance control, and sensor fusion.

### IMU Components

```text
┌─────────────────────────────────────────────────┐
│                  IMU PACKAGE                     │
│  ┌──────────────────┐   ┌──────────────────┐   │
│  │  ACCELEROMETER   │   │    GYROSCOPE     │   │
│  │  (m/s²)          │   │    (rad/s)       │   │
│  │  X, Y, Z accel   │   │  X, Y, Z angular │   │
│  └──────────────────┘   └──────────────────┘   │
│  ┌──────────────────┐   ┌──────────────────┐   │
│  │  MAGNETOMETER    │   │   TEMPERATURE    │   │
│  │  (optional)      │   │   SENSOR         │   │
│  │  Heading ref     │   │   Drift comp     │   │
│  └──────────────────┘   └──────────────────┘   │
└─────────────────────────────────────────────────┘
                    │
                    ▼
         Sensor Fusion (EKF/Madgwick)
                    │
                    ▼
         Orientation (quaternion)
```

### Why IMU Noise Matters Most

IMU noise is **the most critical noise model** because errors accumulate through integration:

| Sensor | Error Type | 60-Second Impact |
|--------|------------|------------------|
| Gyroscope | Orientation drift | 0.001 rad/s bias → 3.4° error |
| Accelerometer | Position drift | 0.01 m/s² bias → 18m error |

This is why IMUs **must** be fused with other sensors (GPS, cameras) for long-term accuracy.

### IMU Configuration in SDF

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>  <!-- 100 Hz typical -->
  <topic>imu</topic>

  <imu>
    <orientation_reference_frame>
      <localization>CUSTOM</localization>
      <grav_dir_x>0</grav_dir_x>
      <grav_dir_y>0</grav_dir_y>
      <grav_dir_z>-1</grav_dir_z>
    </orientation_reference_frame>

    <!-- Accelerometer: Consumer IMU noise levels -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.05</stddev>      <!-- m/s² white noise -->
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.0001</bias_stddev>  <!-- Bias drift -->
        </noise>
      </x>
      <!-- Y and Z axes similar -->
    </linear_acceleration>

    <!-- Gyroscope: Key error source -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>      <!-- rad/s white noise -->
          <bias_mean>0.0001</bias_mean>
          <bias_stddev>0.0001</bias_stddev>  <!-- Causes drift! -->
        </noise>
      </x>
      <!-- Y and Z axes similar -->
    </angular_velocity>

    <enable_orientation>true</enable_orientation>
  </imu>
</sensor>
```

### ROS 2 IMU Message

The `sensor_msgs/msg/Imu` message contains:
- `orientation` (quaternion): x, y, z, w
- `angular_velocity` (rad/s): x, y, z
- `linear_acceleration` (m/s²): x, y, z
- Covariance matrices for uncertainty

See the complete IMU configuration in [imu_config.sdf](../code-examples/module-2/imu_config.sdf) and the detailed noise analysis in the [IMU Noise Model Diagram](../diagrams/module-2/imu-noise-model.md).

---

## Core Concept 4: Noise Models for Realism

### Why Add Noise?

Training AI algorithms in **perfect simulation** leads to **brittle policies** that fail in the real world:

```text
PERFECT SIMULATION:                 NOISY SIMULATION:
┌─────────────────────┐            ┌─────────────────────┐
│ Train on clean data │            │ Train on noisy data │
│         ↓           │            │         ↓           │
│ 99% sim accuracy    │            │ 85% sim accuracy    │
│         ↓           │            │         ↓           │
│ Deploy to real      │            │ Deploy to real      │
│         ↓           │            │         ↓           │
│ 40% real accuracy!  │            │ 80% real accuracy!  │
│ (overfitting)       │            │ (robust)            │
└─────────────────────┘            └─────────────────────┘
```

:::tip Domain Randomization
During ML training, **randomize noise parameters** within realistic ranges. This forces algorithms to be robust to sensor variation rather than overfitting to specific noise characteristics.
:::

### Noise Model Components

Real sensors exhibit multiple noise types:

| Noise Type | Description | Mathematical Model |
|------------|-------------|-------------------|
| **White noise** | Random per-measurement | `N(0, σ)` |
| **Bias** | Constant offset | `+ b` |
| **Bias drift** | Slowly varying offset | `b(t) = b(t-1) + N(0, σ_bias × √dt)` |
| **Scale factor** | Gain error | `× (1 + s)` |

Complete model: `measured = true × (1 + s) + b + N(0, σ)`

### Recommended Noise Levels by Sensor Grade

| Grade | Accelerometer (m/s²) | Gyroscope (rad/s) | Use Case |
|-------|---------------------|-------------------|----------|
| Navigation | 0.001 - 0.005 | 0.0001 - 0.0005 | Aerospace, high-end |
| Industrial | 0.01 - 0.02 | 0.001 - 0.003 | Industrial robots |
| Consumer | 0.05 - 0.1 | 0.005 - 0.01 | Mobile phones, drones |
| Low-cost | 0.1 - 0.3 | 0.01 - 0.05 | Toys, basic applications |

### Configuring Noise for Domain Randomization

```xml
<!-- Low noise for algorithm debugging -->
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- Start low -->
</noise>

<!-- High noise for robust training -->
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.1</stddev>   <!-- Challenge the algorithm -->
</noise>

<!-- For ML: Randomize stddev between episodes -->
<!-- stddev ∈ [0.01, 0.2] uniform distribution -->
```

---

## Core Concept 5: ROS 2 Sensor Integration

### The Gazebo-ROS 2 Bridge

Sensor data flows from Gazebo through a multi-stage pipeline to reach your perception nodes:

```text
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                         │
│  GPU LiDAR → gz.msgs.PointCloudPacked                       │
│  Depth Cam → gz.msgs.Image                                   │
│  IMU       → gz.msgs.IMU                                     │
└──────────────────────────┬──────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────┐
│                   ros_gz_bridge                              │
│  Converts Gazebo messages to ROS 2 messages                  │
│  gz.msgs.PointCloudPacked → sensor_msgs/msg/PointCloud2     │
│  gz.msgs.Image            → sensor_msgs/msg/Image           │
│  gz.msgs.IMU              → sensor_msgs/msg/Imu             │
└──────────────────────────┬──────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────┐
│                    ROS 2 DDS                                 │
│  /scan, /camera/depth/image_raw, /imu/data                  │
└──────────────────────────┬──────────────────────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────┐
│              Your Perception Nodes                           │
│  SLAM, Object Detection, State Estimation                    │
└──────────────────────────────────────────────────────────────┘
```

### Bridge Configuration

Create a YAML configuration file for the bridge:

```yaml
# bridge_config.yaml
- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/depth/image_raw"
  gz_topic_name: "/camera/depth"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ  # Commands TO Gazebo
```

Launch the bridge:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=/path/to/bridge_config.yaml
```

### Visualizing Sensors in RViz2

```bash
# Terminal 1: Launch Gazebo with robot
gz sim robot_world.sdf

# Terminal 2: Start the bridge
ros2 run ros_gz_bridge parameter_bridge ...

# Terminal 3: Launch RViz2
rviz2

# In RViz2:
# 1. Set Fixed Frame to "base_link" or "world"
# 2. Add PointCloud2 display for /scan
# 3. Add Image display for /camera/depth/image_raw
# 4. Add Imu display for /imu/data
```

### Latency Considerations

| Component | Typical Latency | Notes |
|-----------|-----------------|-------|
| Sensor computation | 1-10 ms | GPU-bound for LiDAR/camera |
| Gazebo transport | 0.1-0.5 ms | Shared memory |
| ros_gz_bridge | 0.5-2.0 ms | Message conversion |
| DDS middleware | 0.5-2.0 ms | Serialization |
| **Total** | **2-15 ms** | Acceptable for most applications |

For time-critical control loops, ensure sensor update rates exceed your control frequency by at least 2×.

See the complete data flow architecture in the [Sensor Plugin Flow Diagram](../diagrams/module-2/sensor-plugin-flow.md).

---

## Hands-On Example: Multi-Sensor Robot

Let's bring everything together by examining a robot equipped with all three sensor types.

### Step 1: Review the LiDAR Robot Model

Open [lidar_robot.sdf](../code-examples/module-2/lidar_robot.sdf) and examine:

```xml
<!-- LiDAR mounted on sensor tower -->
<link name="lidar_link">
  <pose relative_to="chassis">0 0 0.25 0 0 0</pose>
  <!-- ... -->
  <sensor name="gpu_lidar" type="gpu_lidar">
    <!-- 640×16 rays, 30m range, 3cm noise -->
  </sensor>
</link>
```

Key points:
- LiDAR mounted 25cm above chassis (clear line of sight)
- 640 horizontal × 16 vertical samples (10,240 points/scan)
- 10 Hz update rate (typical for outdoor navigation)
- 3cm Gaussian noise (realistic for mid-range LiDAR)

### Step 2: Review the Depth Camera Robot

Open [depth_camera_robot.sdf](../code-examples/module-2/depth_camera_robot.sdf) and examine:

```xml
<!-- RGB-D camera for close-range perception -->
<sensor name="depth_camera" type="depth_camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>  <!-- 60° -->
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>10.0</far>
    </clip>
  </camera>
</sensor>
```

Key points:
- 640×480 resolution at 30 Hz
- 60° field of view (typical for Intel RealSense)
- 10cm - 10m depth range
- Paired RGB camera for color information

### Step 3: Launch and Visualize

```bash
# Terminal 1: Launch Gazebo with the LiDAR robot
gz sim /path/to/lidar_robot.sdf

# Terminal 2: Bridge all sensors
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /camera/depth@sensor_msgs/msg/Image@gz.msgs.Image \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU

# Terminal 3: Check topics are publishing
ros2 topic list
ros2 topic hz /lidar        # Should show ~10 Hz
ros2 topic hz /camera/depth # Should show ~30 Hz
ros2 topic hz /imu          # Should show ~100 Hz

# Terminal 4: Launch RViz2 for visualization
rviz2
```

### Step 4: Experiment with Noise

Modify the noise parameters in the SDF files and observe the effects:

1. **Increase LiDAR noise** to 0.1m: Watch point cloud become "fuzzy"
2. **Add IMU bias**: Observe orientation drift over 60 seconds
3. **Reduce camera noise**: See sharper depth edges

This experimentation builds intuition for how noise affects downstream algorithms.

---

## Summary

In this chapter, you learned to simulate the three fundamental sensor types for robotic perception:

1. **LiDAR sensors** use GPU ray-casting to generate point clouds; resolution directly trades off against simulation speed

2. **Depth cameras** render per-pixel distance using the graphics pipeline; understanding camera intrinsics is essential for 3D reconstruction

3. **IMU sensors** measure acceleration and rotation; bias drift is the dominant error source requiring sensor fusion

4. **Noise models** are essential for training robust AI—perfect simulation leads to brittle real-world performance

5. **Domain randomization** of noise parameters during training creates algorithms that generalize across sensor variations

6. **ros_gz_bridge** connects Gazebo sensors to ROS 2 perception pipelines with configurable topic mapping and message conversion

---

## Self-Assessment Checklist

Before moving to Chapter 5, verify you can:

- [ ] Configure a LiDAR sensor with appropriate resolution for your use case (horizontal samples, vertical channels, range)
- [ ] Set up a depth camera with correct intrinsic parameters (FOV, resolution, clip distances)
- [ ] Add IMU sensors with realistic noise models (accelerometer stddev, gyroscope bias drift)
- [ ] Explain why sensor noise is necessary for robust AI training (sim-to-real transfer)
- [ ] Bridge simulated sensors to ROS 2 topics using ros_gz_bridge configuration
- [ ] Visualize sensor data in RViz2 (PointCloud2, Image, Imu displays)

---

## Next Steps

With sensor simulation mastered, you're ready to explore **high-fidelity visualization in Unity**. Chapter 5 introduces Unity's visual rendering capabilities and human-robot interaction scenarios—essential for social robotics and demonstration systems.
