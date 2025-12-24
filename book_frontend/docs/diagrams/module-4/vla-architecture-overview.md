<<<<<<< HEAD
---
title: VLA Architecture Overview
module: 4
chapter: 1
type: architecture
---

# VLA Architecture Overview

## Overview

This diagram illustrates the complete Vision-Language-Action (VLA) system architecture, showing how natural language commands flow through perception, planning, and execution layers to produce robot actions. This three-layer design represents the fundamental pattern used in modern humanoid robotics systems.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                      VISION-LANGUAGE-ACTION SYSTEM                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     PERCEPTION LAYER                                 │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │   │
│  │  │    Voice    │  │   Vision    │  │  Multimodal │  │   Scene    │  │   │
│  │  │   Input     │  │   Input     │  │   Fusion    │  │   State    │  │   │
│  │  │  (Whisper)  │  │  (Cameras)  │  │   Engine    │  │   Graph    │  │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  │   │
│  │         │                │                │               │         │   │
│  │         └────────────────┼────────────────┘               │         │   │
│  │                          ▼                                │         │   │
│  │                  ┌───────────────┐                        │         │   │
│  │                  │    Intent     │◄───────────────────────┘         │   │
│  │                  │  Extraction   │                                  │   │
│  │                  └───────┬───────┘                                  │   │
│  └──────────────────────────┼──────────────────────────────────────────┘   │
│                             │                                               │
│                             ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      PLANNING LAYER                                  │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │   │
│  │  │    Task     │  │   Action    │  │  Constraint │  │  Resource  │  │   │
│  │  │ Decomposer  │  │    Graph    │  │   Checker   │  │  Allocator │  │   │
│  │  │   (LLM)     │  │  Generator  │  │   (Safety)  │  │            │  │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  │   │
│  │         │                │                │               │         │   │
│  │         └────────────────┼────────────────┴───────────────┘         │   │
│  │                          ▼                                          │   │
│  │                  ┌───────────────┐                                  │   │
│  │                  │   Validated   │                                  │   │
│  │                  │  Action Plan  │                                  │   │
│  │                  └───────┬───────┘                                  │   │
│  └──────────────────────────┼──────────────────────────────────────────┘   │
│                             │                                               │
│                             ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     EXECUTION LAYER                                  │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │   │
│  │  │   Action    │  │   Motion    │  │   Sensor    │  │  Feedback  │  │   │
│  │  │   Server    │  │  Planner    │  │  Monitor    │  │    Loop    │  │   │
│  │  │  (ROS 2)    │  │ (MoveIt 2)  │  │             │  │            │  │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  │   │
│  │         │                │                │               │         │   │
│  │         └────────────────┴────────────────┴───────────────┘         │   │
│  │                          │                                          │   │
│  └──────────────────────────┼──────────────────────────────────────────┘   │
│                             │                                               │
│                             ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      ROBOT HARDWARE                                  │   │
│  │                                                                      │   │
│  │    ┌────────┐    ┌────────┐    ┌────────┐    ┌────────┐            │   │
│  │    │ Motors │    │Grippers│    │Cameras │    │Sensors │            │   │
│  │    └────────┘    └────────┘    └────────┘    └────────┘            │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Descriptions

### Perception Layer

**Purpose**: Convert raw sensory data into structured understanding of commands and environment.

**Voice Input (Whisper)**
- **Purpose**: Convert speech to text
- **Inputs**: Raw audio stream from microphone
- **Outputs**: Transcribed text with confidence scores
- **Key Details**: Uses OpenAI Whisper (local or API) for speech recognition

**Vision Input (Cameras)**
- **Purpose**: Capture visual information about environment
- **Inputs**: RGB-D camera feeds (color + depth)
- **Outputs**: Point clouds, images, detected objects
- **Key Details**: Typically stereo or RGB-D cameras at 30-60 FPS

**Multimodal Fusion Engine**
- **Purpose**: Combine voice and vision for grounded understanding
- **Inputs**: Transcribed text, visual features, scene context
- **Outputs**: Unified representation of command with visual grounding
- **Key Details**: Late fusion strategy with confidence weighting

**Scene State Graph**
- **Purpose**: Maintain structured representation of environment
- **Inputs**: Object detections, spatial relationships
- **Outputs**: Queryable scene graph with object positions
- **Key Details**: Updated continuously, supports referring expressions

**Intent Extraction**
- **Purpose**: Parse natural language into actionable intents
- **Inputs**: Text + visual context
- **Outputs**: Intent (action type) + entities (parameters)
- **Key Details**: Slot-filling approach with disambiguation

### Planning Layer

**Task Decomposer (LLM)**
- **Purpose**: Break high-level commands into primitive actions
- **Inputs**: Intent + entities + scene state
- **Outputs**: Ordered list of subtasks
- **Key Details**: Uses LLM reasoning for complex decomposition

**Action Graph Generator**
- **Purpose**: Convert subtasks into executable DAG
- **Inputs**: Subtask list, preconditions, postconditions
- **Outputs**: Directed acyclic graph of primitives
- **Key Details**: Handles parallel actions and dependencies

**Constraint Checker (Safety)**
- **Purpose**: Validate plan against safety constraints
- **Inputs**: Action graph, safety rules
- **Outputs**: Validated plan or rejection with reason
- **Key Details**: Force limits, collision avoidance, exclusion zones

**Resource Allocator**
- **Purpose**: Assign robot resources to actions
- **Inputs**: Action requirements, available resources
- **Outputs**: Resource assignments (arms, grippers, etc.)
- **Key Details**: Prevents resource conflicts in parallel execution

### Execution Layer

**Action Server (ROS 2)**
- **Purpose**: Execute primitive actions with feedback
- **Inputs**: Primitive action requests
- **Outputs**: Success/failure status, feedback during execution
- **Key Details**: ROS 2 action server pattern with preemption

**Motion Planner (MoveIt 2)**
- **Purpose**: Generate collision-free trajectories
- **Inputs**: Target poses, current state, obstacles
- **Outputs**: Joint trajectories
- **Key Details**: Integrates with ROS 2 for real-time control

**Sensor Monitor**
- **Purpose**: Monitor execution through sensors
- **Inputs**: Sensor streams during execution
- **Outputs**: Success verification, anomaly detection
- **Key Details**: Compares expected vs actual outcomes

**Feedback Loop**
- **Purpose**: Provide continuous feedback to planning layer
- **Inputs**: Execution status, sensor data
- **Outputs**: Replanning triggers, progress updates
- **Key Details**: Enables adaptive behavior and error recovery

---

## Data Flow

1. **Voice Input**: User speaks command ("Pick up the red cup")
2. **Speech Recognition**: Whisper transcribes to text
3. **Visual Grounding**: Camera identifies objects, fusion links "red cup" to detected object
4. **Intent Extraction**: Parse command → PICK action + red_cup entity
5. **Task Decomposition**: LLM breaks into: locate → approach → grasp → lift
6. **Action Graph**: Generate DAG with preconditions (e.g., gripper open)
7. **Safety Check**: Validate force limits, collision-free path exists
8. **Motion Planning**: MoveIt 2 generates joint trajectories
9. **Execution**: Action server executes with sensor monitoring
10. **Feedback**: Report success or trigger replanning on failure

---

## Key Insights

### Layered Abstraction

The three-layer architecture provides clean separation of concerns:
- **Perception** handles "what" (understanding the command and environment)
- **Planning** handles "how" (deciding the sequence of actions)
- **Execution** handles "do" (actual robot control)

This separation allows each layer to be developed, tested, and upgraded independently.

### Bidirectional Information Flow

Unlike simple pipelines, VLA systems require bidirectional flow:
- Forward: Commands flow down to actions
- Backward: Execution feedback flows up for replanning

This enables adaptive behavior when initial plans fail.

### Safety as a Cross-Cutting Concern

Safety constraints are checked at multiple levels:
- Planning layer validates high-level safety
- Execution layer enforces real-time limits
- Feedback loop detects unsafe states

---

## Related Concepts

- **From Module 1**: ROS 2 action servers, topics, and services patterns
- **From Module 2**: Simulation for testing VLA pipelines
- **From Module 3**: Isaac-accelerated perception for real-time operation
- **In This Module**: Voice Pipeline, LLM Planning Pipeline, Multimodal Fusion

---

## Real-World Application

This three-layer architecture is used by leading humanoid robotics systems:

**NVIDIA's Gr00t** uses a similar pattern for their humanoid foundation model, with perception from Isaac Vision, planning from LLM-based reasoning, and execution through Isaac Control.

**Boston Dynamics' Spot** implements language-grounded navigation using this architecture, where voice commands are decomposed into navigation goals and executed through their control stack.

**Tesla's Optimus** employs multimodal fusion to understand workplace instructions and execute manipulation tasks, using neural networks for each layer while maintaining the same logical separation.

The key to success in production systems is maintaining clear interfaces between layers while allowing for specialized optimization within each.
=======
---
title: VLA Architecture Overview
module: 4
chapter: 1
type: architecture
---

# VLA Architecture Overview

## Overview

This diagram illustrates the complete Vision-Language-Action (VLA) system architecture, showing how natural language commands flow through perception, planning, and execution layers to produce robot actions. This three-layer design represents the fundamental pattern used in modern humanoid robotics systems.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                      VISION-LANGUAGE-ACTION SYSTEM                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     PERCEPTION LAYER                                 │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │   │
│  │  │    Voice    │  │   Vision    │  │  Multimodal │  │   Scene    │  │   │
│  │  │   Input     │  │   Input     │  │   Fusion    │  │   State    │  │   │
│  │  │  (Whisper)  │  │  (Cameras)  │  │   Engine    │  │   Graph    │  │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  │   │
│  │         │                │                │               │         │   │
│  │         └────────────────┼────────────────┘               │         │   │
│  │                          ▼                                │         │   │
│  │                  ┌───────────────┐                        │         │   │
│  │                  │    Intent     │◄───────────────────────┘         │   │
│  │                  │  Extraction   │                                  │   │
│  │                  └───────┬───────┘                                  │   │
│  └──────────────────────────┼──────────────────────────────────────────┘   │
│                             │                                               │
│                             ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      PLANNING LAYER                                  │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │   │
│  │  │    Task     │  │   Action    │  │  Constraint │  │  Resource  │  │   │
│  │  │ Decomposer  │  │    Graph    │  │   Checker   │  │  Allocator │  │   │
│  │  │   (LLM)     │  │  Generator  │  │   (Safety)  │  │            │  │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  │   │
│  │         │                │                │               │         │   │
│  │         └────────────────┼────────────────┴───────────────┘         │   │
│  │                          ▼                                          │   │
│  │                  ┌───────────────┐                                  │   │
│  │                  │   Validated   │                                  │   │
│  │                  │  Action Plan  │                                  │   │
│  │                  └───────┬───────┘                                  │   │
│  └──────────────────────────┼──────────────────────────────────────────┘   │
│                             │                                               │
│                             ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     EXECUTION LAYER                                  │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │   │
│  │  │   Action    │  │   Motion    │  │   Sensor    │  │  Feedback  │  │   │
│  │  │   Server    │  │  Planner    │  │  Monitor    │  │    Loop    │  │   │
│  │  │  (ROS 2)    │  │ (MoveIt 2)  │  │             │  │            │  │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  │   │
│  │         │                │                │               │         │   │
│  │         └────────────────┴────────────────┴───────────────┘         │   │
│  │                          │                                          │   │
│  └──────────────────────────┼──────────────────────────────────────────┘   │
│                             │                                               │
│                             ▼                                               │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      ROBOT HARDWARE                                  │   │
│  │                                                                      │   │
│  │    ┌────────┐    ┌────────┐    ┌────────┐    ┌────────┐            │   │
│  │    │ Motors │    │Grippers│    │Cameras │    │Sensors │            │   │
│  │    └────────┘    └────────┘    └────────┘    └────────┘            │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Descriptions

### Perception Layer

**Purpose**: Convert raw sensory data into structured understanding of commands and environment.

**Voice Input (Whisper)**
- **Purpose**: Convert speech to text
- **Inputs**: Raw audio stream from microphone
- **Outputs**: Transcribed text with confidence scores
- **Key Details**: Uses OpenAI Whisper (local or API) for speech recognition

**Vision Input (Cameras)**
- **Purpose**: Capture visual information about environment
- **Inputs**: RGB-D camera feeds (color + depth)
- **Outputs**: Point clouds, images, detected objects
- **Key Details**: Typically stereo or RGB-D cameras at 30-60 FPS

**Multimodal Fusion Engine**
- **Purpose**: Combine voice and vision for grounded understanding
- **Inputs**: Transcribed text, visual features, scene context
- **Outputs**: Unified representation of command with visual grounding
- **Key Details**: Late fusion strategy with confidence weighting

**Scene State Graph**
- **Purpose**: Maintain structured representation of environment
- **Inputs**: Object detections, spatial relationships
- **Outputs**: Queryable scene graph with object positions
- **Key Details**: Updated continuously, supports referring expressions

**Intent Extraction**
- **Purpose**: Parse natural language into actionable intents
- **Inputs**: Text + visual context
- **Outputs**: Intent (action type) + entities (parameters)
- **Key Details**: Slot-filling approach with disambiguation

### Planning Layer

**Task Decomposer (LLM)**
- **Purpose**: Break high-level commands into primitive actions
- **Inputs**: Intent + entities + scene state
- **Outputs**: Ordered list of subtasks
- **Key Details**: Uses LLM reasoning for complex decomposition

**Action Graph Generator**
- **Purpose**: Convert subtasks into executable DAG
- **Inputs**: Subtask list, preconditions, postconditions
- **Outputs**: Directed acyclic graph of primitives
- **Key Details**: Handles parallel actions and dependencies

**Constraint Checker (Safety)**
- **Purpose**: Validate plan against safety constraints
- **Inputs**: Action graph, safety rules
- **Outputs**: Validated plan or rejection with reason
- **Key Details**: Force limits, collision avoidance, exclusion zones

**Resource Allocator**
- **Purpose**: Assign robot resources to actions
- **Inputs**: Action requirements, available resources
- **Outputs**: Resource assignments (arms, grippers, etc.)
- **Key Details**: Prevents resource conflicts in parallel execution

### Execution Layer

**Action Server (ROS 2)**
- **Purpose**: Execute primitive actions with feedback
- **Inputs**: Primitive action requests
- **Outputs**: Success/failure status, feedback during execution
- **Key Details**: ROS 2 action server pattern with preemption

**Motion Planner (MoveIt 2)**
- **Purpose**: Generate collision-free trajectories
- **Inputs**: Target poses, current state, obstacles
- **Outputs**: Joint trajectories
- **Key Details**: Integrates with ROS 2 for real-time control

**Sensor Monitor**
- **Purpose**: Monitor execution through sensors
- **Inputs**: Sensor streams during execution
- **Outputs**: Success verification, anomaly detection
- **Key Details**: Compares expected vs actual outcomes

**Feedback Loop**
- **Purpose**: Provide continuous feedback to planning layer
- **Inputs**: Execution status, sensor data
- **Outputs**: Replanning triggers, progress updates
- **Key Details**: Enables adaptive behavior and error recovery

---

## Data Flow

1. **Voice Input**: User speaks command ("Pick up the red cup")
2. **Speech Recognition**: Whisper transcribes to text
3. **Visual Grounding**: Camera identifies objects, fusion links "red cup" to detected object
4. **Intent Extraction**: Parse command → PICK action + red_cup entity
5. **Task Decomposition**: LLM breaks into: locate → approach → grasp → lift
6. **Action Graph**: Generate DAG with preconditions (e.g., gripper open)
7. **Safety Check**: Validate force limits, collision-free path exists
8. **Motion Planning**: MoveIt 2 generates joint trajectories
9. **Execution**: Action server executes with sensor monitoring
10. **Feedback**: Report success or trigger replanning on failure

---

## Key Insights

### Layered Abstraction

The three-layer architecture provides clean separation of concerns:
- **Perception** handles "what" (understanding the command and environment)
- **Planning** handles "how" (deciding the sequence of actions)
- **Execution** handles "do" (actual robot control)

This separation allows each layer to be developed, tested, and upgraded independently.

### Bidirectional Information Flow

Unlike simple pipelines, VLA systems require bidirectional flow:
- Forward: Commands flow down to actions
- Backward: Execution feedback flows up for replanning

This enables adaptive behavior when initial plans fail.

### Safety as a Cross-Cutting Concern

Safety constraints are checked at multiple levels:
- Planning layer validates high-level safety
- Execution layer enforces real-time limits
- Feedback loop detects unsafe states

---

## Related Concepts

- **From Module 1**: ROS 2 action servers, topics, and services patterns
- **From Module 2**: Simulation for testing VLA pipelines
- **From Module 3**: Isaac-accelerated perception for real-time operation
- **In This Module**: Voice Pipeline, LLM Planning Pipeline, Multimodal Fusion

---

## Real-World Application

This three-layer architecture is used by leading humanoid robotics systems:

**NVIDIA's Gr00t** uses a similar pattern for their humanoid foundation model, with perception from Isaac Vision, planning from LLM-based reasoning, and execution through Isaac Control.

**Boston Dynamics' Spot** implements language-grounded navigation using this architecture, where voice commands are decomposed into navigation goals and executed through their control stack.

**Tesla's Optimus** employs multimodal fusion to understand workplace instructions and execute manipulation tasks, using neural networks for each layer while maintaining the same logical separation.

The key to success in production systems is maintaining clear interfaces between layers while allowing for specialized optimization within each.
>>>>>>> 5cf5f6818272e652910633754d301908ff0bb236
