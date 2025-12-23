# Physical AI Components Diagram Specification

**Purpose**: Show the high-level architecture of a physical AI system with robot middleware

**Diagram Type**: Block diagram / System architecture

## Components

### 1. AI Agent (Top Layer)
- **Label**: "AI Agent / Decision Engine"
- **Description**: High-level intelligence (e.g., LLM, policy network, planner)
- **Color**: Blue
- **Examples**: "Vision model", "Path planner", "Task executor"

### 2. Middleware Layer (Middle Layer)
- **Label**: "ROS 2 Middleware"
- **Description**: Communication and coordination infrastructure
- **Color**: Green
- **Sub-components**:
  - **Nodes**: Computational processes
  - **Topics**: Asynchronous message streams
  - **Services**: Synchronous request-response
  - **Actions**: Long-running goal-based tasks

### 3. Sensors (Left Side)
- **Label**: "Sensors"
- **Description**: Input devices providing environment data
- **Color**: Orange
- **Examples**:
  - Camera (vision)
  - LiDAR (depth)
  - IMU (orientation)
  - Force sensors (touch)
  - Joint encoders (proprioception)

### 4. Actuators (Right Side)
- **Label**: "Actuators"
- **Description**: Output devices executing physical actions
- **Color**: Red
- **Examples**:
  - Motors (joint movement)
  - Grippers (manipulation)
  - Wheels (locomotion)
  - LED indicators (status)

## Data Flow

1. **Sensors → Middleware**: Raw sensor data published to topics (e.g., `/camera/image`, `/imu/data`)
2. **Middleware → AI Agent**: Processed data consumed by AI decision nodes
3. **AI Agent → Middleware**: Commands/goals published to control topics (e.g., `/joint_commands`, `/gripper/goal`)
4. **Middleware → Actuators**: Motor controllers subscribe to command topics and execute actions

## Key Insight

The middleware (ROS 2) **decouples** AI logic from hardware:
- AI Agent doesn't need to know specific sensor protocols
- Actuators can be swapped without changing AI code
- Multiple AI agents can share the same sensor data
- Distributed processing across multiple computers

## Visual Layout

```
┌─────────────────────────────────────────┐
│         AI Agent / Decision Engine       │
│    (LLM, Vision Model, Path Planner)    │
└─────────────────┬───────────────────────┘
                  │
        ┌─────────▼─────────┐
        │   ROS 2 Middleware  │
        │ (Nodes/Topics/Srv)  │
        └───┬────────────┬────┘
            │            │
    ┌───────▼──────┐ ┌──▼───────────┐
    │   Sensors    │ │   Actuators   │
    │  (Camera,    │ │   (Motors,    │
    │   LiDAR)     │ │   Grippers)   │
    └──────────────┘ └───────────────┘
```

## Usage in Book

- **Referenced in**: Chapter 1 (Core Concept 1: Physical AI and Embodied Intelligence)
- **Purpose**: Help learners visualize the role of ROS 2 in connecting AI to physical hardware
- **Key Question**: "Why can't AI just directly control motors?" → Answer: Middleware provides abstraction, communication, and modularity
