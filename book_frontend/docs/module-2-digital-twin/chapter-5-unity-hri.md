---
sidebar_position: 5
title: Chapter 5 - Unity for Human-Robot Interaction
---

# Chapter 5: Unity for Human-Robot Interaction

## Introduction

Throughout this module, you've built digital twins in Gazebo—accurate physics simulations essential for control algorithm development. But Gazebo's rendering, while functional, lacks the visual fidelity needed for **human-robot interaction (HRI)** research. When robots must recognize human gestures, navigate social spaces, or hand objects to people, we need **photorealistic humans** and **rich visual environments**.

This chapter introduces **Unity** as a complementary simulation platform. Unity's game-engine heritage provides stunning graphics, realistic human avatars, and a massive asset ecosystem—all connected to your ROS 2 stack via the Unity Robotics Hub.

### Prerequisites

- Completed Chapter 4 (sensor simulation in Gazebo)
- Basic familiarity with Unity Editor (optional but helpful)
- ROS 2 Humble installed and working

### Learning Objectives

By the end of this chapter, you will be able to:

1. Install and configure the Unity Robotics Hub packages
2. Import URDF robot models into Unity with correct physics settings
3. Set up bidirectional ROS 2 communication via ROS-TCP-Connector
4. Design human-robot interaction scenarios with Unity's animation and NPC systems
5. Understand when to use Unity vs. Gazebo for different simulation tasks

---

## Core Concept 1: Unity Robotics Hub

**Unity Robotics Hub** is a collection of packages that bridge Unity's powerful rendering engine with ROS 2's robotics ecosystem.

### Key Packages

| Package | Purpose |
|---------|---------|
| **URDF Importer** | Import robot models from URDF files |
| **ROS-TCP-Connector** | Bidirectional communication with ROS 2 |
| **ROS Message Types** | C# implementations of common ROS messages |
| **Visualizations** | Debug tools for transforms, paths, markers |

### Installation Steps

```text
INSTALLING UNITY ROBOTICS HUB
═════════════════════════════

1. Create new Unity project (2022.3 LTS recommended)
   → Unity Hub → New Project → 3D (URP) or 3D (HDRP)

2. Open Package Manager
   → Window → Package Manager

3. Add packages by name (click + → Add package by name):

   Package 1: com.unity.robotics.urdf-importer
   Version: 0.5.2-preview (or latest)

   Package 2: com.unity.robotics.ros-tcp-connector
   Version: 0.7.0-preview (or latest)

4. Import ROS message packages (for common message types):

   Package 3: com.unity.robotics.ros-tcp-connector-deps
   → Includes: sensor_msgs, geometry_msgs, std_msgs, etc.
```

### Unity Rendering Pipelines

Unity offers three rendering pipelines with different visual quality/performance trade-offs:

| Pipeline | Visual Quality | Performance | Use Case |
|----------|---------------|-------------|----------|
| **Built-in** | ★★★☆☆ | ★★★★★ | Legacy, maximum compatibility |
| **URP** (Universal) | ★★★★☆ | ★★★★☆ | Balanced quality/performance |
| **HDRP** (High Definition) | ★★★★★ | ★★★☆☆ | Photorealistic HRI training |

:::tip Recommendation
For HRI research, use **HDRP** to generate photorealistic training data. For real-time interaction testing, **URP** provides better frame rates while maintaining good visuals.
:::

### Gazebo vs Unity: Complementary Tools

```text
SIMULATION TOOL SELECTION
═════════════════════════

┌─────────────────────────────────────────────────────────────────────────────┐
│                     USE GAZEBO WHEN:                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│  • Developing control algorithms (accurate physics essential)               │
│  • Testing sensor-based navigation (LiDAR, depth cameras)                   │
│  • Running headless simulation (no GPU required)                            │
│  • Integrating with ROS 2 natively (DDS, parameters, lifecycle)            │
│  • Simulating contact-rich manipulation                                     │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                     USE UNITY WHEN:                                          │
├─────────────────────────────────────────────────────────────────────────────┤
│  • Training vision models (photorealistic images essential)                 │
│  • Simulating human-robot interaction (realistic humans needed)             │
│  • Creating demonstrations/visualizations (marketing, papers)               │
│  • Testing in diverse environments (vast Unity Asset Store)                 │
│  • Synthetic data generation (domain randomization with variety)            │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                     USE BOTH (Hybrid Approach):                              │
├─────────────────────────────────────────────────────────────────────────────┤
│  • Gazebo for physics simulation + control                                   │
│  • Unity for rendering only (camera images from Unity, physics from Gazebo)│
│  • Sync state via ROS 2 topics                                               │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Core Concept 2: Importing Robot Models

Unity's URDF Importer converts your existing robot descriptions into Unity GameObjects with proper physics components.

### URDF Import Process

```text
URDF IMPORT WORKFLOW
════════════════════

    ┌─────────────────┐
    │ robot.urdf      │
    │ + mesh files    │
    └────────┬────────┘
             │
             ▼
    ┌─────────────────┐
    │ URDF Importer   │  Settings:
    │                 │  • Axis: Z-Up (ROS convention)
    │ Right-click →   │  • Use URDF Inertia: ✓
    │ Import Robot    │  • ArticulationBody: ✓
    └────────┬────────┘
             │
             ▼
    ┌─────────────────┐
    │ Unity Prefab    │
    │ • GameObjects   │
    │ • ArticulationBody
    │ • Colliders     │
    │ • Materials     │
    └─────────────────┘
```

### ArticulationBody Physics

Unity provides two physics systems for connected bodies. **Always use ArticulationBody** for robots:

| Feature | ArticulationBody | Rigidbody + Joints |
|---------|------------------|-------------------|
| Long kinematic chains | ✓ Stable | ✗ Jittery |
| Joint limit enforcement | ✓ Strict | ✗ Can violate |
| Direct drive control | ✓ SetDriveTarget() | ✗ Motor forces |
| ROS joint model match | ✓ Close mapping | ✗ Different semantics |

### Drive Configuration

After import, configure joint drives for position control:

```csharp
// Each joint has X, Y, Z drives (typically one active per joint)
var drive = articulationBody.xDrive;
drive.stiffness = 10000f;    // Position gain (Kp) - N·m/rad
drive.damping = 100f;        // Velocity gain (Kd) - N·m·s/rad
drive.forceLimit = 1000f;    // Maximum torque - N·m
drive.target = targetAngle;  // Desired position - degrees
articulationBody.xDrive = drive;
```

:::warning Units Mismatch
URDF uses **radians** for joint limits; Unity uses **degrees**. The importer converts automatically, but be careful when setting targets from ROS messages (which use radians).
:::

### Common Import Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| "Mesh not found" | Path mismatch | Use `package://` or relative paths |
| Wrong orientation | Axis convention | Select "Z-Up" in import settings |
| Joints explode | Using Rigidbody | Switch to ArticulationBody |
| Robot falls through floor | No ground collider | Add plane with MeshCollider |

See the complete import workflow in the [Unity URDF Import Diagram](../diagrams/module-2/unity-urdf-import.md).

---

## Core Concept 3: ROS 2 Bridge Configuration

The **ROS-TCP-Connector** enables bidirectional communication between Unity and ROS 2 via TCP sockets.

### Architecture Overview

```text
UNITY-ROS 2 COMMUNICATION
═════════════════════════

  Unity                              ROS 2
┌──────────────┐                 ┌──────────────┐
│ ROSConnection│   TCP Socket    │ros_tcp_      │
│ (C# class)   │◄───────────────►│endpoint      │
│              │   Port 10000    │(Python node) │
└──────────────┘                 └──────────────┘
      │                                │
      │ Publishers                     │ Standard ROS 2
      │ Subscribers                    │ Topics/Services
      │ Service Clients                │
      ▼                                ▼
┌──────────────┐                 ┌──────────────┐
│ Unity Scene  │                 │ ROS 2 Nodes  │
│ Robot, Cameras│                │ Nav2, MoveIt │
└──────────────┘                 └──────────────┘
```

### Setting Up the ROS 2 Endpoint

On the ROS 2 side, run the TCP endpoint:

```bash
# Install ros_tcp_endpoint
sudo apt install ros-humble-ros-tcp-endpoint

# Or from source (for custom configuration)
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws && colcon build --packages-select ros_tcp_endpoint

# Launch the endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Unity ROSConnection Setup

In Unity, configure the connection:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class RobotBridge : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        // Get singleton instance
        ros = ROSConnection.GetOrCreateInstance();

        // Configure connection (also settable in Editor)
        ros.RosIPAddress = "127.0.0.1";  // ROS 2 machine IP
        ros.RosPort = 10000;

        // Register publishers (Unity → ROS 2)
        ros.RegisterPublisher<JointStateMsg>("/joint_states");
        ros.RegisterPublisher<ImageMsg>("/unity/camera/image_raw");

        // Register subscribers (ROS 2 → Unity)
        ros.Subscribe<JointStateMsg>("/joint_commands", OnJointCommand);
        ros.Subscribe<TwistMsg>("/cmd_vel", OnVelocityCommand);
    }

    void OnJointCommand(JointStateMsg msg)
    {
        // Apply commanded joint positions to ArticulationBody
        for (int i = 0; i < msg.position.Length; i++)
        {
            // Convert radians to degrees for Unity
            float targetDegrees = (float)(msg.position[i] * Mathf.Rad2Deg);
            // Apply to corresponding joint drive
        }
    }

    void FixedUpdate()
    {
        // Publish current joint states at physics rate
        var jointState = new JointStateMsg();
        // Fill with current joint positions
        ros.Publish("/joint_states", jointState);
    }
}
```

### Message Type Support

Common message types are pre-generated. For custom messages:

```bash
# Generate C# classes from .msg files
# In Unity: Robotics → Generate ROS Messages
# Point to your custom_msgs package
```

### Latency Considerations

| Path | Typical Latency | Notes |
|------|-----------------|-------|
| Unity → ROS 2 (localhost) | 3-6 ms | Acceptable for 100 Hz |
| Unity → ROS 2 (network) | 5-55 ms | Consider for real-time control |
| High-res images | +10-50 ms | Use JPEG compression |

For detailed architecture, see the [Unity-ROS Bridge Diagram](../diagrams/module-2/unity-ros-bridge.md).

---

## Core Concept 4: Human-Robot Interaction Scenarios

Unity's greatest advantage for robotics is simulating **realistic humans**. This is essential for:

- **Gesture recognition** training data
- **Social navigation** algorithms
- **Object handoff** interactions
- **Collaborative manipulation** tasks

### Human Avatar Setup

Unity supports humanoid characters via:

1. **Mixamo** (Adobe): Free animated character library
2. **Unity Asset Store**: Thousands of human models
3. **Custom avatars**: Import from Blender, Maya, etc.

```text
HUMANOID AVATAR PIPELINE
════════════════════════

    ┌─────────────────┐
    │ Mixamo.com      │  Free animated humans
    │ Select character│
    │ Select animation│
    └────────┬────────┘
             │ Download .fbx
             ▼
    ┌─────────────────┐
    │ Unity Import    │  Rig: Humanoid
    │ Avatar setup    │  Animation Type: Generic/Humanoid
    └────────┬────────┘
             │
             ▼
    ┌─────────────────┐
    │ Animator        │  State machine for:
    │ Controller      │  • Idle, Walk, Run
    │                 │  • Gesture animations
    │                 │  • Interaction states
    └────────┬────────┘
             │
             ▼
    ┌─────────────────┐
    │ NavMesh Agent   │  AI navigation:
    │                 │  • Pathfinding
    │                 │  • Obstacle avoidance
    │                 │  • Crowd simulation
    └─────────────────┘
```

### Common HRI Scenarios

#### 1. Object Handoff

Robot extends object; human receives it.

```text
Requirements:
• Human hand tracking (pose estimation)
• Gripper force sensing
• Coordinated timing state machine

ROS 2 Topics:
  /human_pose        ← geometry_msgs/PoseStamped
  /gripper_force     ← std_msgs/Float64
  /handoff_state     → std_msgs/String
```

#### 2. Social Navigation

Robot navigates respecting human personal space.

```text
Requirements:
• Multiple human NPCs with NavMesh
• Proxemics zones (intimate, personal, social, public)
• Trajectory prediction

ROS 2 Topics:
  /people_tracker    ← people_msgs/People
  /social_costmap    ← nav_msgs/OccupancyGrid
```

#### 3. Gesture Recognition

Robot responds to human gestures (stop, come, point).

```text
Requirements:
• Human with gesture animations
• Camera positioned to capture gestures
• Training data generation (varied lighting, angles)

ROS 2 Topics:
  /camera/image_raw  → sensor_msgs/Image
  /gesture_detected  ← std_msgs/String
```

### Domain Randomization for HRI

To train robust perception models, randomize:

- **Human appearance**: Different avatars, clothing, skin tones
- **Lighting**: Time of day, indoor/outdoor, shadows
- **Camera viewpoint**: Height, angle, distance
- **Background**: Different environments, clutter levels
- **Occlusion**: Partial human visibility

See detailed HRI patterns in the [HRI Interaction Patterns Diagram](../diagrams/module-2/hri-interaction-patterns.md).

---

## Hands-On Example: Unity Robot Setup

Let's walk through setting up a robot in Unity connected to ROS 2.

### Step 1: Install Packages

```text
1. Create new Unity 2022.3 LTS project (3D URP template)

2. Package Manager → Add by name:
   • com.unity.robotics.urdf-importer
   • com.unity.robotics.ros-tcp-connector

3. Wait for compilation to complete
```

### Step 2: Import Your Robot

```text
1. Copy your robot_description folder into Assets/

2. Right-click on robot.urdf → "Import Robot from URDF"

3. Settings:
   • Axis Type: Z-Up
   • Mesh Decomposer: VHACD
   • Use URDF Inertia: ✓
   • Create Articulation Bodies: ✓

4. Click "Import"

5. Drag the generated prefab into your scene
```

### Step 3: Configure ROS Connection

Create a new C# script `RobotROSController.cs`:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RobotROSController : MonoBehaviour
{
    [SerializeField] private string rosIP = "127.0.0.1";
    [SerializeField] private int rosPort = 10000;

    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        // Setup ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RosIPAddress = rosIP;
        ros.RosPort = rosPort;

        // Find all joints
        joints = GetComponentsInChildren<ArticulationBody>();

        // Register publisher
        ros.RegisterPublisher<JointStateMsg>("/joint_states");

        // Subscribe to commands
        ros.Subscribe<JointStateMsg>("/joint_commands", OnJointCommand);

        Debug.Log($"Connected to ROS at {rosIP}:{rosPort}");
    }

    void OnJointCommand(JointStateMsg msg)
    {
        for (int i = 0; i < Mathf.Min(msg.position.Length, joints.Length); i++)
        {
            var drive = joints[i].xDrive;
            drive.target = (float)(msg.position[i] * Mathf.Rad2Deg);
            joints[i].xDrive = drive;
        }
    }

    void FixedUpdate()
    {
        // Publish current state
        var jointState = new JointStateMsg();
        jointState.name = new string[joints.Length];
        jointState.position = new double[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            jointState.name[i] = joints[i].name;
            jointState.position[i] = joints[i].jointPosition[0] * Mathf.Deg2Rad;
        }

        ros.Publish("/joint_states", jointState);
    }
}
```

### Step 4: Launch ROS 2 Endpoint

```bash
# Terminal 1: Start ROS 2 TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

# Terminal 2: Verify topics appear
ros2 topic list
# Should see /joint_states after Unity connects

# Terminal 3: Send test command
ros2 topic pub /joint_commands sensor_msgs/JointState \
  "{name: ['joint1'], position: [0.5]}"
```

### Step 5: Verify Communication

```bash
# Echo joint states from Unity
ros2 topic echo /joint_states

# Should see joint positions updating at ~50 Hz
```

See the complete bridge configuration reference in [unity_bridge_config.xml](../code-examples/module-2/unity_bridge_config.xml).

---

## Summary

In this chapter, you explored Unity as a complementary simulation platform for human-robot interaction:

1. **Unity Robotics Hub** provides URDF import, ROS 2 connectivity, and visualization tools for robotics development

2. **URDF import** converts robot models to Unity prefabs with ArticulationBody physics for stable joint simulation

3. **ROS-TCP-Connector** bridges Unity and ROS 2 via TCP sockets, enabling joint state synchronization and command reception

4. **Human-robot interaction** scenarios leverage Unity's photorealistic humans, animations, and physics for gesture recognition, social navigation, and collaborative tasks

5. **Complementary workflow**: Use Gazebo for physics-critical control, Unity for vision training and HRI—or combine both for hybrid simulation

6. **Domain randomization** in Unity generates diverse training data (varied humans, lighting, environments) for robust perception models

---

## Self-Assessment Checklist

Before concluding Module 2, verify you can:

- [ ] Install Unity Robotics Hub packages (URDF Importer, ROS-TCP-Connector)
- [ ] Import a URDF robot model with correct axis convention and ArticulationBody physics
- [ ] Configure bidirectional ROS 2 communication via ros_tcp_endpoint
- [ ] Create a simple human-robot interaction scenario with an animated NPC
- [ ] Explain when to use Unity vs. Gazebo for different simulation requirements
- [ ] Set up domain randomization for perception model training

---

## Module 2 Complete

Congratulations! You've completed Module 2: The Digital Twin. You now understand how to:

- Design physics-accurate simulations in Gazebo
- Configure sensors with realistic noise models
- Import robots and connect them to ROS 2
- Create human-robot interaction scenarios in Unity

In **Module 3**, you'll apply these simulation skills to train AI policies using reinforcement learning and imitation learning—turning your digital twin into a learning environment for physical AI.
