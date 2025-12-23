# Sensor Plugin Data Flow Specification

**Purpose**: Illustrate how sensor data flows from Gazebo physics through plugins to ROS 2 perception nodes

**Diagram Type**: data_flow

## Complete Sensor Data Pipeline

```text
┌─────────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                            │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SENSOR SYSTEM PLUGIN                         │
│              (gz-sim-sensors-system)                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐ │
│  │ GPU LiDAR   │  │ Depth Camera│  │ IMU System              │ │
│  │ Plugin      │  │ Plugin      │  │ Plugin                  │ │
│  └──────┬──────┘  └──────┬──────┘  └───────────┬─────────────┘ │
│         │                │                     │               │
│    Ray-casting      Depth buffer          Physics state       │
│    (GPU compute)    (GPU render)          (CPU compute)       │
│         │                │                     │               │
└─────────┼────────────────┼─────────────────────┼───────────────┘
          │                │                     │
          ▼                ▼                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                 GAZEBO TRANSPORT (gz-transport)                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Topic: /lidar         Topic: /camera/depth    Topic: /imu     │
│  Type: gz.msgs.        Type: gz.msgs.Image     Type: gz.msgs.  │
│        LaserScan                                      IMU       │
│                                                                 │
│  Protocol: Google Protocol Buffers (protobuf)                  │
│  Transport: TCP/UDP (intra-process shared memory)              │
│                                                                 │
└───────────────────────────────┬─────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 BRIDGE (ros_gz_bridge)                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Message Type Conversion:                                       │
│  ────────────────────────                                       │
│  gz.msgs.LaserScan  ────► sensor_msgs/msg/LaserScan           │
│  gz.msgs.Image      ────► sensor_msgs/msg/Image               │
│  gz.msgs.IMU        ────► sensor_msgs/msg/Imu                 │
│  gz.msgs.PointCloud ────► sensor_msgs/msg/PointCloud2         │
│                                                                 │
│  Topic Remapping:                                               │
│  ────────────────                                               │
│  /lidar            ────► /scan                                 │
│  /camera/depth     ────► /camera/depth/image_raw              │
│  /imu              ────► /imu/data                            │
│                                                                 │
└───────────────────────────────┬─────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ROS 2 DDS MIDDLEWARE                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Topic: /scan                  QoS: Best Effort, Volatile      │
│  Topic: /camera/depth/image_raw   QoS: Best Effort, Volatile   │
│  Topic: /imu/data              QoS: Reliable, Volatile         │
│                                                                 │
│  Discovery: Automatic via DDS participant                       │
│  Serialization: CDR (Common Data Representation)               │
│                                                                 │
└───────────────────────────────┬─────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 PERCEPTION NODES                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌────────────────┐ │
│  │ SLAM Node       │  │ Object Detection│  │ State Estimator│ │
│  │ (slam_toolbox)  │  │ (YOLO, etc.)    │  │ (robot_locali.)│ │
│  ├─────────────────┤  ├─────────────────┤  ├────────────────┤ │
│  │ Subscribes:     │  │ Subscribes:     │  │ Subscribes:    │ │
│  │ • /scan         │  │ • /camera/image │  │ • /imu/data    │ │
│  │ • /odom         │  │ • /camera/depth │  │ • /odom        │ │
│  ├─────────────────┤  ├─────────────────┤  ├────────────────┤ │
│  │ Publishes:      │  │ Publishes:      │  │ Publishes:     │ │
│  │ • /map          │  │ • /detections   │  │ • /pose        │ │
│  │ • /tf           │  │ • /markers      │  │ • /tf          │ │
│  └─────────────────┘  └─────────────────┘  └────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Sensor Update Timeline

```text
TIME (ms)   SENSOR          ACTION                    OUTPUT
────────────────────────────────────────────────────────────────
0           Physics         Step simulation           World state updated
1           LiDAR           Skip (10 Hz = 100ms)      -
1           Camera          Skip (30 Hz = 33ms)       -
1           IMU             Compute accel/gyro        gz.msgs.IMU
│                           Add noise                  ↓
│                           Publish to gz-transport   /imu topic
2           ros_gz_bridge   Convert IMU message       sensor_msgs/Imu
│                           Publish to DDS            /imu/data topic
│           ...
10          IMU             Compute + publish         /imu/data
│           ...
33          Camera          Render depth buffer       gz.msgs.Image
│                           Add noise                  ↓
│                           Publish to gz-transport   /camera/depth
34          ros_gz_bridge   Convert Image             sensor_msgs/Image
│                           Publish to DDS            /camera/depth/image_raw
│           ...
100         LiDAR           GPU ray-casting           gz.msgs.LaserScan
│                           Add noise                  ↓
│                           Publish to gz-transport   /lidar
101         ros_gz_bridge   Convert LaserScan         sensor_msgs/LaserScan
│                           Publish to DDS            /scan
│           ...
```

## Bridge Configuration

```yaml
# ros_gz_bridge configuration file (bridge.yaml)
# Launch: ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml

- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS  # Gazebo publishes, ROS subscribes

- ros_topic_name: "/camera/depth/image_raw"
  gz_topic_name: "/camera/depth"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name: "/camera/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
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
  direction: ROS_TO_GZ  # ROS publishes commands TO Gazebo
```

## Latency Analysis

```text
SENSOR DATA LATENCY BREAKDOWN
═════════════════════════════

Component                    Typical Latency    Notes
─────────────────────────────────────────────────────────────
Physics step                 0.5 - 1.0 ms       CPU-bound
Sensor computation:
  • LiDAR (GPU ray-cast)     1 - 5 ms           GPU-bound, depends on rays
  • Camera (GPU render)      2 - 10 ms          GPU-bound, depends on resolution
  • IMU (physics query)      < 0.1 ms           Negligible
Gazebo transport             0.1 - 0.5 ms       Shared memory for intra-process
ros_gz_bridge                0.5 - 2.0 ms       Message copy + conversion
DDS middleware               0.5 - 2.0 ms       Serialization + network
ROS node processing          Application-specific

TOTAL END-TO-END:
  • IMU:    2 - 5 ms   (acceptable for 100 Hz)
  • Camera: 5 - 15 ms  (acceptable for 30 Hz)
  • LiDAR:  3 - 10 ms  (acceptable for 10 Hz)

For real-time control:
  • Use intra-process communication when possible
  • Set appropriate QoS (Best Effort for sensors)
  • Match sensor rate to control loop requirements
```

## Usage in Book

- **Referenced in**: Chapter 4 (Core Concept 5: ROS 2 Sensor Integration)
- **Purpose**: Show complete data flow from simulation to perception algorithms
- **Learning Goal**: Understand the pipeline to debug latency, missing data, and topic configuration issues

## Key Takeaways

1. **Three-stage pipeline**: Gazebo plugin → gz-transport → ros_gz_bridge → ROS 2 DDS
2. **Message conversion**: Bridge converts gz.msgs to sensor_msgs types automatically
3. **Update rates differ**: IMU (100 Hz) > Camera (30 Hz) > LiDAR (10 Hz) typical
4. **Latency adds up**: End-to-end latency is sum of all stages; optimize critical path
5. **Configuration via YAML**: Bridge topics, types, and directions specified in config file
6. **QoS matters**: Use Best Effort for high-rate sensors, Reliable for commands
