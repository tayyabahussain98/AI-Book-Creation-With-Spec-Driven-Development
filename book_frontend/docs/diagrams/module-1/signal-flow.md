# Signal Flow Diagram Specification: AI Decision to Motor Actuation

**Purpose**: Trace the complete data flow from AI decision to physical motor movement in a humanoid robot

**Diagram Type**: Sequence diagram / Data flow diagram

## Actors and Components

### 1. AI Agent Node (`/decision_maker`)
- **Type**: ROS 2 Node (Python/C++)
- **Role**: High-level decision making (e.g., "move arm to position X")
- **Input**: Sensor data (camera, joint states)
- **Output**: Motion commands

### 2. Topics (Communication Channels)
- **`/camera/image`**: Vision input (sensor → AI)
- **`/joint_states`**: Current robot pose (controller → AI)
- **`/joint_commands`**: Desired joint positions (AI → controller)
- **`/motor_control`**: Low-level PWM signals (controller → hardware)

### 3. Motion Controller Node (`/motion_controller`)
- **Type**: ROS 2 Node (usually C++ for real-time performance)
- **Role**: Translate high-level commands to motor signals
- **Input**: Joint position/velocity targets
- **Output**: Motor control signals (PWM, CAN bus messages)

### 4. Motor Hardware
- **Type**: Physical actuator (servo, brushless motor)
- **Interface**: Serial (UART), CAN bus, or PWM
- **Feedback**: Encoder data (position, velocity, torque)

## Complete Signal Flow

### Step 1: Perception (Sensor → AI)
```
Camera Hardware → ROS 2 Image Topic → AI Vision Node
                                        ↓
                                   Object detected at (x, y, z)
```

### Step 2: Decision (AI Processing)
```
AI Vision Node → AI Decision Node
                       ↓
                  Compute target:
                  "Move gripper to (x, y, z)"
                       ↓
                  Inverse kinematics:
                  Joint angles: [θ1=45°, θ2=60°, θ3=30°]
```

### Step 3: Command Publication (AI → Middleware)
```
AI Decision Node → Publish to /joint_commands topic
                         Message Type: JointTrajectory
                         Data: [45°, 60°, 30°]
```

### Step 4: Motion Control (Middleware → Controller)
```
/joint_commands topic → Motion Controller Node subscribes
                              ↓
                        PID control loop:
                        - Read current position: [40°, 55°, 28°]
                        - Calculate error: [5°, 5°, 2°]
                        - Compute PWM signals: [1.2V, 1.5V, 0.8V]
```

### Step 5: Actuation (Controller → Hardware)
```
Motion Controller → Publish to /motor_control topic
                          Message Type: MotorCommands
                          Data: [PWM1=1.2V, PWM2=1.5V, PWM3=0.8V]
                                 ↓
                          Motor Driver Hardware
                                 ↓
                          Physical motors rotate
```

### Step 6: Feedback Loop (Hardware → AI)
```
Motor Encoders → Publish to /joint_states topic
                      ↓
                Motion Controller (verifies position)
                      ↓
                AI Decision Node (confirms goal reached)
```

## Timing Example (10Hz Control Loop)

| Time (ms) | Event                                    |
|-----------|------------------------------------------|
| 0         | Camera publishes image to /camera/image  |
| 10        | AI processes image, detects object       |
| 20        | AI publishes joint commands              |
| 30        | Controller receives commands             |
| 30-100    | PID loop executes (10Hz)                 |
| 40        | Motors start moving                      |
| 100       | Encoders publish updated joint states    |
| 110       | AI confirms motion progress              |

## Key ROS 2 Concepts Illustrated

### 1. Asynchronous Communication
- AI doesn't wait for motor confirmation
- Each node operates independently
- Topics decouple sender from receiver

### 2. Message Types
- **Sensor Data**: `sensor_msgs/Image`, `sensor_msgs/JointState`
- **Control Commands**: `trajectory_msgs/JointTrajectory`, `std_msgs/Float64MultiArray`
- **Custom Messages**: User-defined for specific robots

### 3. QoS (Quality of Service)
- **Sensor Topics**: Best-effort (lossy OK, need latest data)
- **Control Topics**: Reliable (critical, cannot lose commands)

### 4. Multiple Subscribers
- Multiple AI agents can listen to /joint_states
- Visualization tools (RViz) subscribe to all topics for debugging

## Failure Modes

### Scenario: AI Node Crashes
```
Result: Motion controller still runs (uses last command or enters safe state)
Recovery: AI node restarts, re-subscribes to topics, resumes
```

### Scenario: Motor Hardware Fault
```
Result: Encoder stops publishing → Controller detects timeout → Publishes error
Recovery: AI node reads error, stops sending commands, alerts operator
```

## Visual Layout

```
┌─────────────┐         ┌────────────┐         ┌──────────┐
│ Camera HW   │────────>│/camera/img │────────>│ AI Node  │
└─────────────┘         └────────────┘         └────┬─────┘
                                                     │ Decision
                                                     ▼
                        ┌──────────────┐      ┌──────────────┐
                        │/joint_commands│<─────│  AI Publishes│
                        └──────┬───────┘      └──────────────┘
                               │
                               ▼
                        ┌──────────────┐
                        │Motion Ctrl   │ (PID Loop)
                        └──────┬───────┘
                               │
                               ▼
                        ┌──────────────┐
                        │/motor_control│
                        └──────┬───────┘
                               │
                               ▼
                        ┌──────────────┐      ┌────────────┐
                        │ Motor Driver │─────>│ Motor HW   │
                        └──────────────┘      └─────┬──────┘
                                                    │ Encoder
                                                    ▼
                                             ┌──────────────┐
                                             │/joint_states │
                                             └──────────────┘
                                                    │
                                                    └──────> (Back to AI Node)
```

## Usage in Book

- **Referenced in**:
  - Chapter 1 (Core Concept 3: ROS 2 Architecture Overview)
  - Chapter 2 (Core Concept 2: Topics)
- **Purpose**: Demystify "how does AI actually make a robot move?"
- **Learning Goal**: Learners can trace signal flow from perception to actuation
