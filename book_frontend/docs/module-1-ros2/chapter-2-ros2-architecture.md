---
sidebar_position: 2
title: Chapter 2 - ROS 2 Architecture
---

# Chapter 2: ROS 2 Architecture

## Introduction

In Chapter 1, you learned **why** humanoid robots need middleware and how ROS 2 serves as the "nervous system" connecting AI decision-making to physical hardware. This chapter dives deeper into **how** ROS 2 actually works under the hood. You'll explore the core architectural components that enable distributed, real-time robot control: **nodes** (independent processes), **topics** (asynchronous message streams), **services** (synchronous request-response), **actions** (goal-oriented tasks), and **DDS** (the underlying communication layer).

By the end of this chapter, you'll have a mental model for tracing the complete signal flow from an AI decision (e.g., "move arm to position X") through ROS 2 topics and services to a motor controller executing the command. This architectural understanding is prerequisite to Chapter 3's hands-on coding - without knowing how nodes and topics interact, you'll struggle to write effective ROS 2 code.

**Prerequisites**: Chapter 1 (understanding of physical AI, middleware concepts, and ROS 2's role).

---

## Core Concept 1: Nodes and Executors

### What is a Node?

A **node** in ROS 2 is an independent computational process that encapsulates a specific piece of functionality. Think of nodes as modular building blocks:

- **Camera Driver Node**: Captures images from a camera and publishes them to a topic.
- **Vision Node**: Subscribes to camera images, runs object detection (e.g., YOLO), and publishes detected objects.
- **Motion Planner Node**: Subscribes to detected objects, computes arm trajectories using inverse kinematics, and publishes joint commands.
- **Motor Controller Node**: Subscribes to joint commands and sends low-level signals to actuators.

Each node runs as a separate operating system process (or thread) with its own execution context. Nodes communicate by publishing and subscribing to **topics**, calling **services**, or sending **action goals** - but they **never** share memory or directly call each other's functions. This decoupling means:

- **Fault Isolation**: If the Vision Node crashes, the Camera Node keeps running.
- **Language Flexibility**: A Python node can communicate with a C++ node seamlessly (ROS 2 handles serialization).
- **Distributed Execution**: Nodes can run on different machines across a network without code changes.

### Node Lifecycle

ROS 2 supports two types of nodes:

1. **Unmanaged Nodes**: Standard nodes that start immediately and run until explicitly shut down. Most simple nodes (e.g., data loggers, basic publishers) are unmanaged.

2. **Managed Nodes (Lifecycle Nodes)**: Nodes with explicit state management for safety-critical systems. A managed node transitions through states:

   - **Unconfigured** (initial): Node exists but hasn't allocated resources (no topic connections, no hardware access).
   - **Inactive**: Resources allocated (topics created, parameters loaded) but not actively processing data. Useful for "standby" mode.
   - **Active**: Fully operational - publishing, subscribing, executing callbacks.
   - **Finalized**: Shutdown state, resources released.

See the [Node Lifecycle Diagram](/docs/diagrams/module-1/node-lifecycle.md) for the complete state machine with transitions like `configure()`, `activate()`, `deactivate()`, and `shutdown()`.

**Why Managed Nodes Matter**: For humanoid robots, you want to ensure all sensors are configured before activating motor controllers. Managed nodes let you orchestrate startup: configure all nodes → verify health → activate in dependency order.

### Executors

An **executor** is the ROS 2 component responsible for scheduling node callbacks (subscription callbacks, timer callbacks, service handlers). In Python (rclpy), you typically use:

```python
rclpy.spin(node)  # Single-threaded executor - one callback at a time
```

For advanced use cases (e.g., real-time control requiring parallel processing), ROS 2 provides multi-threaded executors and real-time executors that prioritize critical callbacks.

**Key Insight**: Nodes don't poll for messages. The executor waits for incoming data (topics, service requests) and invokes callbacks when messages arrive. This event-driven model minimizes CPU usage and latency.

---

## Core Concept 2: Topics (Publish-Subscribe)

### What is a Topic?

A **topic** is a named channel for asynchronous, many-to-many message passing. Topics implement the **publish-subscribe pattern**:

- **Publishers** send messages to a topic without knowing who (if anyone) is listening.
- **Subscribers** receive messages from a topic without knowing who published them.
- **Middleware** (DDS) routes messages from publishers to all matching subscribers.

Example:
```
[Camera Node] --publishes--> /camera/image <--subscribes-- [Vision Node]
                                         <--subscribes-- [Data Logger Node]
                                         <--subscribes-- [Debug Visualizer]
```

The Camera Node publishes once; three subscribers automatically receive the image.

### Message Types

Topics transport **typed messages** defined by ROS 2 message definitions (.msg files). Common types:

- **std_msgs/String**: Simple text messages.
- **sensor_msgs/Image**: Camera images (width, height, encoding, pixel data).
- **geometry_msgs/Twist**: Velocity commands (linear velocity, angular velocity).
- **sensor_msgs/JointState**: Joint positions, velocities, efforts for all robot joints.

You can define custom message types for domain-specific data (e.g., `humanoid_msgs/BalanceState` with center-of-mass coordinates, stability margin, etc.).

### When to Use Topics

Use topics for:
- **Continuous data streams**: Sensor readings (camera images at 30 Hz, IMU data at 200 Hz, joint states at 100 Hz).
- **Commands**: Motor velocity commands, gripper position targets.
- **Status updates**: Battery level, robot mode (idle/walking/manipulating).

**Don't use topics for**: Request-response interactions (use **services**) or long-running tasks with feedback (use **actions**).

### Quality of Service (QoS)

Not all robot data has the same delivery requirements. ROS 2's QoS policies let you tune message delivery guarantees:

- **Reliability**:
  - `RELIABLE`: Guarantee delivery (TCP-like). Use for critical commands (motor control).
  - `BEST_EFFORT`: Lossy, like UDP. Use for high-frequency sensor data where latest data matters more than history (camera images - if a frame is lost, the next one arrives in 33ms anyway).

- **Durability**:
  - `VOLATILE`: New subscribers only receive messages published after subscription (default).
  - `TRANSIENT_LOCAL`: New subscribers receive the last N messages published before they joined. Useful for latecomer nodes that need recent state (e.g., "what's the current robot pose?").

- **History**:
  - `KEEP_LAST(N)`: Buffer the last N messages. If subscriber is slow, older messages are dropped.
  - `KEEP_ALL`: Buffer all messages (risk memory overflow if subscriber can't keep up).

**Example**: A balance controller publishes foot pressure at 1000 Hz with `BEST_EFFORT` reliability (missing one sample is tolerable). An emergency stop command uses `RELIABLE` delivery (cannot afford to lose).

---

## Core Concept 3: Services (Request-Response)

### What is a Service?

A **service** is a synchronous request-response communication pattern. Unlike topics (fire-and-forget), services involve:

1. **Client** sends a request and blocks (waits) for a response.
2. **Server** receives the request, computes a result, and sends a response back.

Example:
```
[Planner Node] --request--> /compute_ik(target_pose) <-- [IK Server Node]
               <--response-- (joint_angles or error)
```

The Planner Node calls the `/compute_ik` service with a target pose (x, y, z coordinates), and the IK Server computes the required joint angles (or returns an error if unreachable).

### Service Types

Services use `.srv` files defining request and response fields:

**Example: `ComputeIK.srv`**
```
# Request
geometry_msgs/Pose target_pose
---
# Response
float64[] joint_angles
bool success
string error_message
```

Common ROS 2 services:
- **std_srvs/SetBool**: Turn something on/off (e.g., enable/disable a sensor).
- **std_srvs/Trigger**: Execute a one-time action (e.g., calibrate IMU).
- **nav2_msgs/SaveMap**: Save the current navigation map to disk.

### When to Use Services

Use services for:
- **One-time queries**: "What's the current robot pose?" "Is path to goal collision-free?"
- **Configuration changes**: "Set PID gains to (Kp=2.0, Ki=0.1, Kd=0.5)."
- **Computationally expensive tasks**: Inverse kinematics, collision checking, path planning (tasks that take 10-500ms).

**Don't use services for**: High-frequency control loops (use **topics**) or tasks taking >5 seconds (use **actions** for feedback).

### Synchronous Blocking Consideration

**Warning**: Service calls block the caller until the server responds. If a service takes 200ms to respond and you're in a 100 Hz control loop (10ms period), your loop will stall. Solutions:

- Use **asynchronous service clients** (call service without blocking, handle response in a callback).
- Reserve services for infrequent operations outside critical control loops.

---

## Core Concept 4: Actions (Goal-Oriented)

### What is an Action?

An **action** is a long-running, goal-based task with feedback and preemption support. Actions are built on top of topics and services and provide:

1. **Goal**: Client sends a goal to the action server (e.g., "Navigate to coordinates (x=10, y=5)").
2. **Feedback**: Server periodically sends progress updates (e.g., "Distance remaining: 7.2 meters, ETA: 12 seconds").
3. **Result**: Server sends final outcome when done (e.g., "Goal reached successfully" or "Navigation failed: obstacle detected").
4. **Preemption**: Client can cancel the goal mid-execution (e.g., user presses emergency stop).

Example:
```
[Task Planner] --send goal--> "Navigate to (10, 5)" --> [Navigation Server]
               <--feedback-- "Distance remaining: 7.2m"
               <--feedback-- "Distance remaining: 3.1m"
               <--result--- "Goal reached" (success)
```

### Action Definition

Actions use `.action` files:

**Example: `NavigateToGoal.action`**
```
# Goal
geometry_msgs/Point target_position
---
# Result
bool success
string message
---
# Feedback
float64 distance_remaining
float64 estimated_time
```

### When to Use Actions

Use actions for:
- **Long-running tasks**: Navigation (10-60 seconds), pick-and-place (5-20 seconds), full-body motion sequences.
- **Tasks requiring progress updates**: User wants to see "30% complete, 15 seconds remaining."
- **Cancellable tasks**: User might abort navigation if plans change.

Common ROS 2 actions:
- **nav2_msgs/NavigateToPose**: Autonomous navigation to a goal.
- **control_msgs/FollowJointTrajectory**: Execute a pre-planned arm motion.
- **manipulation_msgs/Grasp**: Close gripper around an object with force feedback.

### Actions vs Topics vs Services

| Communication | Topics | Services | Actions |
|--------------|--------|----------|---------|
| **Pattern** | Publish-subscribe (async) | Request-response (sync) | Goal-based (async) |
| **Timing** | Fire-and-forget | Blocking (or async) | Long-running |
| **Feedback** | No | No | Yes (periodic updates) |
| **Cancellation** | No | No | Yes (preemptable) |
| **Use Case** | Continuous data streams | One-time queries | Multi-second tasks |
| **Example** | `/camera/image` | `/compute_ik` | `/navigate_to_pose` |

---

## Core Concept 5: DDS and Communication Lifecycle

### What is DDS?

**DDS (Data Distribution Service)** is the middleware layer beneath ROS 2. While ROS 2 provides the API (nodes, topics, services, actions), DDS handles the low-level networking:

- **Discovery**: Nodes automatically find each other when they start (no central server like ROS 1's roscore).
- **Transport**: Serializes messages, sends them over UDP/TCP/shared memory, and deserializes on the receiver side.
- **QoS Enforcement**: Implements reliability, durability, and history policies.
- **Security**: Supports encrypted communication and access control lists (DDS-Security).

ROS 2 supports multiple DDS implementations (vendors):
- **Fast DDS** (eProsima): Default, widely used, good performance.
- **CycloneDDS** (Eclipse): Lightweight, excellent for embedded systems.
- **RTI Connext DDS**: Commercial-grade, used in safety-critical applications (aerospace, medical).

You can switch DDS implementations without changing your ROS 2 code by setting the `RMW_IMPLEMENTATION` environment variable.

### Discovery Process

When a ROS 2 node starts:

1. **Node announces itself**: "I am node `/camera_node`, I publish to `/camera/image` (sensor_msgs/Image), QoS: BEST_EFFORT."
2. **Other nodes respond**: "I am `/vision_node`, I subscribe to `/camera/image` (sensor_msgs/Image), QoS: BEST_EFFORT."
3. **DDS matches publishers and subscribers**: QoS policies must be compatible (e.g., RELIABLE publisher can send to BEST_EFFORT subscriber, but not vice versa without QoS override).
4. **Connection established**: Messages flow from publisher to subscriber over the network (or shared memory if same machine).

**No Central Broker**: Unlike ROS 1 (which required `roscore`), ROS 2 nodes discover each other peer-to-peer. This eliminates single points of failure and scales better to multi-robot systems.

### Communication Lifecycle

**Message Flow**:
1. Publisher creates a message: `msg = Image(width=640, height=480, data=pixels)`.
2. Publisher calls `publisher.publish(msg)`.
3. DDS serializes the message (converts Python object to byte stream).
4. DDS sends bytes over network (UDP multicast or unicast).
5. Subscriber's DDS layer receives bytes and deserializes.
6. ROS 2 executor invokes subscriber's callback: `callback(msg)`.

**Latency Sources**:
- **Serialization/Deserialization**: 10-100 microseconds for typical messages.
- **Network Transport**: 100 microseconds (shared memory) to 1-5 milliseconds (Ethernet, same subnet).
- **Executor Scheduling**: Depends on callback queue depth and CPU load.

For real-time control (e.g., motor control at 1000 Hz), use shared memory transport (DDS intra-process communication) and real-time executors to minimize latency.

---

## Diagrams

### Node Lifecycle State Machine

See the [Node Lifecycle Diagram](/docs/diagrams/module-1/node-lifecycle.md) for the complete managed node state machine:

- **States**: Unconfigured (initial), Inactive (resources allocated), Active (processing), Finalized (shutdown)
- **Transitions**: `configure()`, `activate()`, `deactivate()`, `cleanup()`, `shutdown()`
- **Use Case**: Safety-critical humanoid robots where sensors must be validated before activating actuators

**Key Takeaway**: Lifecycle nodes provide deterministic startup/shutdown, preventing "motors activated before sensors ready" scenarios.

### Signal Flow: AI Decision to Motor Actuation

See the [Signal Flow Diagram](/docs/diagrams/module-1/signal-flow.md) for the complete data flow from an AI decision (e.g., "move arm to position X") through ROS 2 middleware to physical motor execution:

**Stages**:
1. **Perception**: Camera hardware → `/camera/image` topic → AI Vision Node (object detection)
2. **Decision**: AI Vision Node → AI Decision Node (compute target pose)
3. **Planning**: AI Decision Node → `/joint_commands` topic (inverse kinematics)
4. **Control**: Motion Controller subscribes to `/joint_commands` → PID loop → `/motor_control` topic
5. **Actuation**: Motor Driver subscribes to `/motor_control` → sends PWM signals to motors
6. **Feedback**: Motor encoders → `/joint_states` topic → back to AI Decision Node (closed loop)

**Timing Example**: At 10 Hz control loop (100ms period):
- 0ms: Camera publishes image
- 10ms: AI processes image, detects object
- 20ms: AI publishes joint commands
- 30-100ms: PID control loop executes at 100 Hz (10ms period)
- 100ms: Encoders publish updated joint states

**Key Insight**: Each stage (perception, decision, control, actuation) runs independently. If vision is slow (200ms), it doesn't block motor control (still runs at 100 Hz with last received command).

---

## Summary

This chapter explored ROS 2's architectural components:

1. **Nodes** are independent processes encapsulating functionality. Managed nodes (lifecycle nodes) provide state management for safety-critical systems with explicit `configure()`, `activate()`, and `shutdown()` transitions.

2. **Topics** enable asynchronous, many-to-many publish-subscribe communication. Use topics for continuous data streams (sensors, commands). QoS policies (reliability, durability, history) tune delivery guarantees for different data types.

3. **Services** provide synchronous request-response for one-time queries and configuration changes. Services block the caller until response arrives - use asynchronously for non-critical operations.

4. **Actions** support long-running, goal-based tasks with progress feedback and cancellation. Use actions for multi-second operations like navigation, manipulation, or trajectory execution.

5. **DDS** is the underlying middleware handling discovery, transport, QoS enforcement, and security. ROS 2's peer-to-peer discovery (no central server) eliminates single points of failure and scales to multi-robot systems.

**Mental Model**: A ROS 2 system is a graph of nodes communicating via topics (data streams), services (queries), and actions (goals). DDS automatically routes messages between nodes based on topic names and QoS compatibility. Each node runs independently - failures isolate, and you can restart individual nodes without rebooting the robot.

---

## Self-Assessment Checklist

After completing this chapter, you should be able to:

- [ ] **I can describe** the role of a ROS 2 node and explain how nodes communicate without directly calling each other's functions.
- [ ] **I can differentiate** between unmanaged nodes (simple, immediate startup) and managed lifecycle nodes (state-based, controlled initialization).
- [ ] **I can explain** when to use topics (continuous data), services (one-time queries), and actions (long-running tasks with feedback).
- [ ] **I can trace** the signal flow from an AI decision through topics/services to motor actuation, identifying each stage (perception, decision, planning, control, actuation, feedback).
- [ ] **I can justify** QoS policy choices (e.g., RELIABLE for critical commands, BEST_EFFORT for high-frequency sensor data, TRANSIENT_LOCAL for latecomer state synchronization).
- [ ] **I can articulate** how DDS enables peer-to-peer discovery without a central server and why this improves fault tolerance compared to ROS 1.
- [ ] **I can identify** appropriate communication patterns for given scenarios: "Send camera images" (topic), "Query current pose" (service), "Navigate to goal" (action).

**Next Chapter**: With architectural knowledge in place, Chapter 3 provides hands-on practice writing Python nodes using rclpy - publishers, subscribers, parameters, and logging.
