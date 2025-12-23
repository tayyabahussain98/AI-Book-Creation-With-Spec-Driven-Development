# Unity-ROS 2 Bridge Architecture Specification

**Purpose**: Illustrate the communication architecture between Unity and ROS 2 via ROS-TCP-Connector

**Diagram Type**: architecture

## Communication Architecture

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                          UNITY-ROS 2 BRIDGE ARCHITECTURE                     │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────┐     ┌─────────────────────────────────┐
│            UNITY SIDE               │     │           ROS 2 SIDE            │
│         (C# + Physics)              │     │        (Python/C++)             │
├─────────────────────────────────────┤     ├─────────────────────────────────┤
│                                     │     │                                 │
│  ┌─────────────────────────────┐   │     │   ┌─────────────────────────┐   │
│  │       Unity Scene           │   │     │   │    ROS 2 Ecosystem      │   │
│  │  ┌─────────────────────┐   │   │     │   │  ┌─────────────────┐   │   │
│  │  │  Robot (URDF)       │   │   │     │   │  │ ros2_control    │   │   │
│  │  │  ArticulationBody   │   │   │     │   │  │ Controllers     │   │   │
│  │  │  joints, physics    │   │   │     │   │  └────────┬────────┘   │   │
│  │  └─────────────────────┘   │   │     │   │           │            │   │
│  │  ┌─────────────────────┐   │   │     │   │  ┌────────▼────────┐   │   │
│  │  │  Camera             │   │   │     │   │  │ Navigation      │   │   │
│  │  │  HDRP/URP renderer  │   │   │     │   │  │ Stack (Nav2)    │   │   │
│  │  └─────────────────────┘   │   │     │   │  └────────┬────────┘   │   │
│  │  ┌─────────────────────┐   │   │     │   │           │            │   │
│  │  │  Human NPCs         │   │   │     │   │  ┌────────▼────────┐   │   │
│  │  │  Animations, AI     │   │   │     │   │  │ MoveIt 2        │   │   │
│  │  └─────────────────────┘   │   │     │   │  │ Motion Planning │   │   │
│  └─────────────────────────────┘   │     │   │  └─────────────────┘   │   │
│              │                      │     │   └─────────────────────────┘   │
│              ▼                      │     │               │                 │
│  ┌─────────────────────────────┐   │     │   ┌───────────▼─────────────┐   │
│  │     ROSConnection           │   │     │   │    ros_tcp_endpoint     │   │
│  │     (C# Singleton)          │   │     │   │    (Python Node)        │   │
│  │                             │   │     │   │                         │   │
│  │  • Publisher registry       │   │     │   │  • TCP server           │   │
│  │  • Subscriber callbacks     │   │     │   │  • Message routing      │   │
│  │  • Service clients          │   │     │   │  • ROS 2 pub/sub        │   │
│  │  • Message serialization    │   │     │   │  • Service proxies      │   │
│  └──────────────┬──────────────┘   │     │   └───────────┬─────────────┘   │
│                 │                  │     │               │                 │
└─────────────────┼──────────────────┘     └───────────────┼─────────────────┘
                  │                                        │
                  │           TCP/IP Connection            │
                  │         Port 10000 (default)           │
                  └──────────────────┬─────────────────────┘
                                     │
                          ┌──────────▼──────────┐
                          │   Network Layer     │
                          │  • TCP socket       │
                          │  • Message framing  │
                          │  • Byte ordering    │
                          └─────────────────────┘
```

## Message Flow Patterns

```text
BIDIRECTIONAL MESSAGE FLOW
══════════════════════════

1. JOINT STATE SYNCHRONIZATION (Unity → ROS 2)
───────────────────────────────────────────────

   Unity (50 Hz)                              ROS 2
   ┌────────────────┐                        ┌────────────────┐
   │ ArticulationBody│  JointState msg       │ Joint State    │
   │ .jointPosition ├───────────────────────►│ Broadcaster    │
   │ .jointVelocity │  /joint_states         │                │
   └────────────────┘                        └───────┬────────┘
                                                     │
                                                     ▼
                                             ┌────────────────┐
                                             │ TF2 / RViz2    │
                                             │ Visualization  │
                                             └────────────────┘


2. VELOCITY COMMANDS (ROS 2 → Unity)
────────────────────────────────────

   ROS 2                                     Unity
   ┌────────────────┐                        ┌────────────────┐
   │ Nav2 / teleop  │  Twist msg             │ Robot Base     │
   │ Controller     ├───────────────────────►│ Controller     │
   │                │  /cmd_vel              │                │
   └────────────────┘                        └────────────────┘


3. CAMERA IMAGES (Unity → ROS 2)
─────────────────────────────────

   Unity (30 Hz)                              ROS 2
   ┌────────────────┐                        ┌────────────────┐
   │ Unity Camera   │  Image msg             │ CV / ML Node   │
   │ HDRP Rendering ├───────────────────────►│ Object Det.    │
   │ 640x480 RGB    │  /camera/image_raw     │ YOLO, etc.     │
   └────────────────┘                        └────────────────┘


4. TRAJECTORY EXECUTION (ROS 2 → Unity)
───────────────────────────────────────

   ROS 2                                     Unity
   ┌────────────────┐                        ┌────────────────┐
   │ MoveIt 2       │  JointTrajectory msg   │ Trajectory     │
   │ Motion Planner ├───────────────────────►│ Executor       │
   │                │  /joint_trajectory     │ Interpolation  │
   └────────────────┘                        └────────────────┘
```

## Message Serialization Protocol

```text
MESSAGE FRAME FORMAT
════════════════════

┌─────────────────────────────────────────────────────────────┐
│                    TCP MESSAGE FRAME                         │
├───────────────┬───────────────────┬─────────────────────────┤
│  HEADER       │  TOPIC NAME       │  SERIALIZED MESSAGE     │
│  (8 bytes)    │  (variable)       │  (variable)             │
├───────────────┼───────────────────┼─────────────────────────┤
│ Length (4B)   │ "/joint_states"   │ ROS 2 CDR serialized    │
│ TopicLen (4B) │ (14 bytes)        │ sensor_msgs/JointState  │
└───────────────┴───────────────────┴─────────────────────────┘

BYTE ORDERING:
• Little-endian for all multi-byte values
• Matches x86/x64 native ordering
• Consistent with ROS 2 CDR format


MESSAGE TYPE MAPPING
════════════════════

Unity C# Type              ROS 2 Message Type           Size
─────────────────────────────────────────────────────────────
JointStateMsg          →   sensor_msgs/JointState       variable
TwistMsg               →   geometry_msgs/Twist          48 bytes
ImageMsg               →   sensor_msgs/Image            ~900 KB (640x480 RGB)
PoseMsg                →   geometry_msgs/Pose           56 bytes
TransformMsg           →   geometry_msgs/Transform      56 bytes
TFMessageMsg           →   tf2_msgs/TFMessage           variable
BoolMsg                →   std_msgs/Bool                1 byte
Float64Msg             →   std_msgs/Float64             8 bytes
StringMsg              →   std_msgs/String              variable
JointTrajectoryMsg     →   trajectory_msgs/JointTraj    variable
```

## Latency Analysis

```text
END-TO-END LATENCY BREAKDOWN
════════════════════════════

Component                         Typical Latency    Notes
───────────────────────────────────────────────────────────────
Unity physics step                1 - 2 ms           Fixed timestep
C# message construction           < 0.1 ms           Negligible
ROSConnection serialization       0.1 - 0.5 ms       Depends on message size
TCP transmission (localhost)      0.1 - 0.5 ms       Kernel overhead
TCP transmission (network)        1 - 50 ms          Network dependent
ros_tcp_endpoint processing       0.5 - 1.0 ms       Python overhead
ROS 2 DDS publication             0.5 - 1.0 ms       Middleware
Subscriber callback               Application-specific
───────────────────────────────────────────────────────────────
TOTAL (localhost)                 3 - 6 ms           Acceptable for 100 Hz
TOTAL (network)                   5 - 55 ms          Consider for latency-critical


BANDWIDTH CONSIDERATIONS
════════════════════════

Topic Type                   Rate      Bandwidth      Notes
───────────────────────────────────────────────────────────
JointState (12 joints)       50 Hz     ~50 KB/s       Low overhead
Camera 640x480 RGB           30 Hz     ~27 MB/s       Consider compression
Camera 1920x1080 RGB         30 Hz     ~178 MB/s      JPEG recommended
PointCloud2 (64k points)     10 Hz     ~25 MB/s       Subsample if needed
TF (50 transforms)           50 Hz     ~140 KB/s      Modest overhead

OPTIMIZATION STRATEGIES:
• Use JPEG compression for images (10:1 ratio typical)
• Subsample point clouds for visualization
• Reduce update rates for non-critical topics
• Batch transforms into single TFMessage
```

## Usage in Book

- **Referenced in**: Chapter 5 (Core Concept 3: ROS 2 Bridge Configuration)
- **Purpose**: Show how Unity and ROS 2 communicate for joint control and visualization
- **Learning Goal**: Configure bidirectional communication between Unity rendering and ROS 2 control

## Key Takeaways

1. **TCP-based bridge**: ROS-TCP-Connector uses TCP sockets (not DDS) for Unity-ROS 2 communication
2. **Bidirectional flow**: Unity publishes joint states/images, subscribes to commands/trajectories
3. **Message serialization**: Custom protocol with CDR-compatible message format
4. **Latency**: ~3-6ms localhost, 5-55ms over network; acceptable for most HRI applications
5. **Bandwidth**: Camera images dominate; use compression for high-resolution streams
6. **ros_tcp_endpoint**: Python node bridges TCP socket to native ROS 2 topics
