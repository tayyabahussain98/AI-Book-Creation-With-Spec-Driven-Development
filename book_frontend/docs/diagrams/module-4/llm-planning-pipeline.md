<<<<<<< HEAD
---
title: LLM Planning Pipeline
module: 4
chapter: 3
type: flow
---

# LLM Planning Pipeline

## Overview

This diagram illustrates how Large Language Models (LLMs) transform high-level natural language commands into executable robot action plans. The pipeline shows task decomposition, action graph generation, and plan validation stages.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                        LLM PLANNING PIPELINE                                │
└─────────────────────────────────────────────────────────────────────────────┘

┌──────────────────┐
│  From: Voice     │   Intent: PICK | Entity: red_cup | Location: table
│        Pipeline  │
└────────┬─────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 1: CONTEXT ASSEMBLY                                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      LLM Prompt Construction                         │   │
│  │                                                                      │   │
│  │  System Prompt:                                                      │   │
│  │  "You are a robot task planner. Given a command, scene state,       │   │
│  │   and robot capabilities, decompose the task into primitive         │   │
│  │   actions. Output valid JSON only."                                 │   │
│  │                                                                      │   │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐           │   │
│  │  │ Scene State   │  │    Robot      │  │   Command     │           │   │
│  │  │               │  │ Capabilities  │  │               │           │   │
│  │  │ • Objects     │  │ • MOVE_TO     │  │ • Intent      │           │   │
│  │  │ • Positions   │  │ • GRASP       │  │ • Entities    │           │   │
│  │  │ • Relations   │  │ • RELEASE     │  │ • Constraints │           │   │
│  │  │ • Obstacles   │  │ • LOOK_AT     │  │               │           │   │
│  │  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘           │   │
│  │          │                  │                  │                    │   │
│  │          └──────────────────┼──────────────────┘                    │   │
│  │                             │                                       │   │
│  │                             ▼                                       │   │
│  │              ┌──────────────────────────────┐                       │   │
│  │              │      Assembled Prompt        │                       │   │
│  │              └──────────────────────────────┘                       │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 2: LLM TASK DECOMPOSITION                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         LLM Reasoning                                │   │
│  │                                                                      │   │
│  │  Input: "Pick up the red cup from the table"                        │   │
│  │                                                                      │   │
│  │  Chain-of-Thought:                                                  │   │
│  │  ┌────────────────────────────────────────────────────────────┐     │   │
│  │  │ 1. The target object is "red_cup" at position (0.5, 0.3)   │     │   │
│  │  │ 2. Robot arm is currently at home position (0.0, 0.0)      │     │   │
│  │  │ 3. Gripper is currently OPEN                               │     │   │
│  │  │ 4. No obstacles between arm and cup                        │     │   │
│  │  │ 5. Cup is graspable (within reach, stable orientation)     │     │   │
│  │  │                                                            │     │   │
│  │  │ Required steps:                                            │     │   │
│  │  │  a) Look at the cup to confirm position                    │     │   │
│  │  │  b) Move arm to pre-grasp position above cup               │     │   │
│  │  │  c) Lower arm to grasp position                            │     │   │
│  │  │  d) Close gripper on cup                                   │     │   │
│  │  │  e) Lift cup to safe height                                │     │   │
│  │  └────────────────────────────────────────────────────────────┘     │   │
│  │                                                                      │   │
│  │  Output: Subtask List                                               │   │
│  │  ┌────────────────────────────────────────────────────────────┐     │   │
│  │  │ 1. LOOK_AT(target: red_cup)                                │     │   │
│  │  │ 2. MOVE_TO(position: pre_grasp_above_cup)                  │     │   │
│  │  │ 3. MOVE_TO(position: grasp_position)                       │     │   │
│  │  │ 4. GRASP(object: red_cup, force: medium)                   │     │   │
│  │  │ 5. MOVE_TO(position: lift_position)                        │     │   │
│  │  └────────────────────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 3: ACTION GRAPH GENERATION                                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Subtasks → DAG Conversion                         │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │                     ACTION GRAPH                            │   │   │
│  │   │                                                             │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   LOOK_AT    │                               │   │   │
│  │   │              │  (red_cup)   │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   MOVE_TO    │                               │   │   │
│  │   │              │ (pre_grasp)  │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   MOVE_TO    │                               │   │   │
│  │   │              │   (grasp)    │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │    GRASP     │                               │   │   │
│  │   │              │  (red_cup)   │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   MOVE_TO    │                               │   │   │
│  │   │              │   (lift)     │                               │   │   │
│  │   │              └──────────────┘                               │   │   │
│  │   │                                                             │   │   │
│  │   │   Properties:                                               │   │   │
│  │   │    • Nodes: 5 primitive actions                             │   │   │
│  │   │    • Edges: 4 sequential dependencies                       │   │   │
│  │   │    • Acyclic: ✓                                             │   │   │
│  │   │    • Single root: ✓                                         │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                Preconditions & Postconditions                        │   │
│  │                                                                      │   │
│  │  LOOK_AT:                                                           │   │
│  │    Pre:  robot.cameras.active = true                                │   │
│  │    Post: target.position.confidence > 0.9                           │   │
│  │                                                                      │   │
│  │  MOVE_TO (pre_grasp):                                               │   │
│  │    Pre:  target.position.known = true                               │   │
│  │    Post: arm.position = pre_grasp_position                          │   │
│  │                                                                      │   │
│  │  GRASP:                                                             │   │
│  │    Pre:  gripper.state = OPEN, arm.near(target)                     │   │
│  │    Post: gripper.holding = target                                   │   │
│  │                                                                      │   │
│  │  MOVE_TO (lift):                                                    │   │
│  │    Pre:  gripper.holding = target                                   │   │
│  │    Post: target.height > safe_height                                │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 4: PLAN VALIDATION                                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐   │
│  │  Safety Checker    │  │ Feasibility Check  │  │ Resource Check     │   │
│  │                    │  │                    │  │                    │   │
│  │  ✓ Force limits    │  │  ✓ All positions   │  │  ✓ Single arm      │   │
│  │  ✓ Velocity caps   │  │    reachable       │  │    required        │   │
│  │  ✓ No collision    │  │  ✓ Object is       │  │  ✓ Gripper         │   │
│  │    paths exist     │  │    graspable       │  │    available       │   │
│  │  ✓ Exclusion zones │  │  ✓ No obstacles    │  │  ✓ No conflicts    │   │
│  │    avoided         │  │    blocking path   │  │                    │   │
│  │                    │  │                    │  │                    │   │
│  │  Status: PASS ✓    │  │  Status: PASS ✓    │  │  Status: PASS ✓    │   │
│  └────────────────────┘  └────────────────────┘  └────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     Validation Result                                │   │
│  │                                                                      │   │
│  │   Overall Status: VALIDATED ✓                                       │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │ If FAILED:                                                  │   │   │
│  │   │  • Return to LLM with failure reason                        │   │   │
│  │   │  • Request alternative plan                                 │   │   │
│  │   │  • Max 3 replanning attempts before human escalation        │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 5: ROS 2 ACTION DISPATCH                                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                   Action Graph → ROS 2 Actions                       │   │
│  │                                                                      │   │
│  │   ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐ │   │
│  │   │ Action Client   │    │ Action Server   │    │ Action Server   │ │   │
│  │   │   (Planner)     │───►│  (Look At)      │    │  (Move To)      │ │   │
│  │   └─────────────────┘    └─────────────────┘    └─────────────────┘ │   │
│  │                                                                      │   │
│  │   Topic: /vla/action_graph                                          │   │
│  │   Service: /vla/execute_plan                                        │   │
│  │   Action: /arm_controller/move_to                                   │   │
│  │   Action: /gripper_controller/grasp                                 │   │
│  │                                                                      │   │
│  │   Execution Pattern:                                                │   │
│  │   ┌───────────────────────────────────────────────────────────────┐ │   │
│  │   │ for action in topological_sort(action_graph):                 │ │   │
│  │   │     result = await execute_action(action)                     │ │   │
│  │   │     if result.failed:                                         │ │   │
│  │   │         trigger_replanning(action, result.error)              │ │   │
│  │   │     update_state(action, result)                              │ │   │
│  │   └───────────────────────────────────────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌──────────────────┐
│   To: Execution  │
│       Layer      │
└──────────────────┘
```

---

## Component Descriptions

### Stage 1: Context Assembly

**LLM Prompt Construction**
- **Purpose**: Build complete context for LLM reasoning
- **Inputs**: Scene state, robot capabilities, user command
- **Outputs**: Structured prompt with all necessary information
- **Key Details**: Prompt engineering is critical for reliable decomposition

**Scene State**
- **Purpose**: Provide environmental context
- **Inputs**: Object positions, spatial relationships, obstacles
- **Outputs**: JSON representation of current world state
- **Key Details**: Must be accurate and up-to-date

**Robot Capabilities**
- **Purpose**: Define available primitive actions
- **Inputs**: Robot configuration
- **Outputs**: List of primitives with parameters
- **Key Details**: Constrains LLM to valid actions only

### Stage 2: LLM Task Decomposition

**LLM Reasoning**
- **Purpose**: Break high-level command into primitives
- **Inputs**: Assembled prompt with context
- **Outputs**: Ordered list of primitive actions
- **Key Details**: Chain-of-thought improves reliability

**Chain-of-Thought**
- **Purpose**: Make LLM reasoning explicit and verifiable
- **Inputs**: Task and context
- **Outputs**: Step-by-step reasoning trace
- **Key Details**: Helps catch logical errors

### Stage 3: Action Graph Generation

**DAG Conversion**
- **Purpose**: Convert subtask list to dependency graph
- **Inputs**: Ordered subtask list
- **Outputs**: Directed acyclic graph of actions
- **Key Details**: Enables parallel execution where possible

**Preconditions & Postconditions**
- **Purpose**: Define execution requirements and effects
- **Inputs**: Action specifications
- **Outputs**: Formal pre/post conditions
- **Key Details**: Enables automated validation and replanning

### Stage 4: Plan Validation

**Safety Checker**
- **Purpose**: Verify plan respects safety constraints
- **Inputs**: Action graph, safety rules
- **Outputs**: Pass/fail with violation details
- **Key Details**: Non-negotiable safety enforcement

**Feasibility Check**
- **Purpose**: Verify physical execution is possible
- **Inputs**: Action graph, robot kinematics, scene
- **Outputs**: Reachability and collision analysis
- **Key Details**: Catches impossible plans early

**Resource Check**
- **Purpose**: Verify no resource conflicts
- **Inputs**: Action graph, resource requirements
- **Outputs**: Resource allocation or conflicts
- **Key Details**: Handles parallel action resource sharing

### Stage 5: ROS 2 Action Dispatch

**Action Graph to ROS 2**
- **Purpose**: Translate graph to ROS 2 action calls
- **Inputs**: Validated action graph
- **Outputs**: ROS 2 action goal messages
- **Key Details**: Uses standard ROS 2 action interface

**Execution Pattern**
- **Purpose**: Execute actions in dependency order
- **Inputs**: Topologically sorted actions
- **Outputs**: Execution results with feedback
- **Key Details**: Handles failures with replanning

---

## Data Flow

1. **Input Reception**: Voice pipeline delivers intent + entities
2. **Context Assembly**: Gather scene state, capabilities into prompt
3. **LLM Inference**: Generate decomposed subtask list
4. **Graph Creation**: Convert subtasks to DAG with dependencies
5. **Condition Attachment**: Add preconditions and postconditions
6. **Safety Validation**: Check against safety constraints
7. **Feasibility Validation**: Verify physical executability
8. **Resource Validation**: Check for conflicts
9. **ROS 2 Dispatch**: Send actions to execution layer
10. **Feedback Loop**: Handle failures with replanning

---

## Key Insights

### LLM as Reasoning Engine, Not Controller

The LLM's role is strictly reasoning and decomposition:
- It does NOT directly control motors
- It does NOT make real-time decisions
- It ONLY produces structured plans

This separation ensures safety and reliability.

### Structured Output Guarantees

LLM output must be parseable:
- JSON schema validation
- Constrained decoding (only valid primitives)
- Retry on malformed output

Production systems use constrained generation to guarantee valid outputs.

### Replanning as First-Class Feature

Plans rarely execute perfectly:
- Execution feedback triggers replanning
- LLM receives failure context for better retry
- Maximum retry limits prevent infinite loops

Robust systems plan for failure from the start.

---

## Related Concepts

- **From Module 1**: ROS 2 action servers and clients, topic-based feedback
- **From Module 2**: Simulation for plan testing before deployment
- **From Module 3**: GPU-accelerated LLM inference for low latency
- **In This Module**: VLA Architecture, Action Graph Structure, Voice Pipeline

---

## Real-World Application

LLM-based planning is being deployed in real robotic systems:

**Google's SayCan** combines LLM reasoning with learned affordances, using the LLM to propose actions while a separate model scores physical feasibility.

**Microsoft's ChatGPT for Robotics** demonstrates task decomposition for drone control, with the LLM generating Python code that interfaces with robot APIs.

**NVIDIA's Eureka** uses LLMs to generate reward functions for robot learning, showing how LLMs can contribute to robotics beyond direct planning.

The key insight from deployed systems is that LLMs excel at high-level reasoning but must be combined with physics-aware validation to ensure executable, safe plans.
=======
---
title: LLM Planning Pipeline
module: 4
chapter: 3
type: flow
---

# LLM Planning Pipeline

## Overview

This diagram illustrates how Large Language Models (LLMs) transform high-level natural language commands into executable robot action plans. The pipeline shows task decomposition, action graph generation, and plan validation stages.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                        LLM PLANNING PIPELINE                                │
└─────────────────────────────────────────────────────────────────────────────┘

┌──────────────────┐
│  From: Voice     │   Intent: PICK | Entity: red_cup | Location: table
│        Pipeline  │
└────────┬─────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 1: CONTEXT ASSEMBLY                                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      LLM Prompt Construction                         │   │
│  │                                                                      │   │
│  │  System Prompt:                                                      │   │
│  │  "You are a robot task planner. Given a command, scene state,       │   │
│  │   and robot capabilities, decompose the task into primitive         │   │
│  │   actions. Output valid JSON only."                                 │   │
│  │                                                                      │   │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐           │   │
│  │  │ Scene State   │  │    Robot      │  │   Command     │           │   │
│  │  │               │  │ Capabilities  │  │               │           │   │
│  │  │ • Objects     │  │ • MOVE_TO     │  │ • Intent      │           │   │
│  │  │ • Positions   │  │ • GRASP       │  │ • Entities    │           │   │
│  │  │ • Relations   │  │ • RELEASE     │  │ • Constraints │           │   │
│  │  │ • Obstacles   │  │ • LOOK_AT     │  │               │           │   │
│  │  └───────┬───────┘  └───────┬───────┘  └───────┬───────┘           │   │
│  │          │                  │                  │                    │   │
│  │          └──────────────────┼──────────────────┘                    │   │
│  │                             │                                       │   │
│  │                             ▼                                       │   │
│  │              ┌──────────────────────────────┐                       │   │
│  │              │      Assembled Prompt        │                       │   │
│  │              └──────────────────────────────┘                       │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 2: LLM TASK DECOMPOSITION                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         LLM Reasoning                                │   │
│  │                                                                      │   │
│  │  Input: "Pick up the red cup from the table"                        │   │
│  │                                                                      │   │
│  │  Chain-of-Thought:                                                  │   │
│  │  ┌────────────────────────────────────────────────────────────┐     │   │
│  │  │ 1. The target object is "red_cup" at position (0.5, 0.3)   │     │   │
│  │  │ 2. Robot arm is currently at home position (0.0, 0.0)      │     │   │
│  │  │ 3. Gripper is currently OPEN                               │     │   │
│  │  │ 4. No obstacles between arm and cup                        │     │   │
│  │  │ 5. Cup is graspable (within reach, stable orientation)     │     │   │
│  │  │                                                            │     │   │
│  │  │ Required steps:                                            │     │   │
│  │  │  a) Look at the cup to confirm position                    │     │   │
│  │  │  b) Move arm to pre-grasp position above cup               │     │   │
│  │  │  c) Lower arm to grasp position                            │     │   │
│  │  │  d) Close gripper on cup                                   │     │   │
│  │  │  e) Lift cup to safe height                                │     │   │
│  │  └────────────────────────────────────────────────────────────┘     │   │
│  │                                                                      │   │
│  │  Output: Subtask List                                               │   │
│  │  ┌────────────────────────────────────────────────────────────┐     │   │
│  │  │ 1. LOOK_AT(target: red_cup)                                │     │   │
│  │  │ 2. MOVE_TO(position: pre_grasp_above_cup)                  │     │   │
│  │  │ 3. MOVE_TO(position: grasp_position)                       │     │   │
│  │  │ 4. GRASP(object: red_cup, force: medium)                   │     │   │
│  │  │ 5. MOVE_TO(position: lift_position)                        │     │   │
│  │  └────────────────────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 3: ACTION GRAPH GENERATION                                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Subtasks → DAG Conversion                         │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │                     ACTION GRAPH                            │   │   │
│  │   │                                                             │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   LOOK_AT    │                               │   │   │
│  │   │              │  (red_cup)   │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   MOVE_TO    │                               │   │   │
│  │   │              │ (pre_grasp)  │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   MOVE_TO    │                               │   │   │
│  │   │              │   (grasp)    │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │    GRASP     │                               │   │   │
│  │   │              │  (red_cup)   │                               │   │   │
│  │   │              └──────┬───────┘                               │   │   │
│  │   │                     │                                       │   │   │
│  │   │                     ▼                                       │   │   │
│  │   │              ┌──────────────┐                               │   │   │
│  │   │              │   MOVE_TO    │                               │   │   │
│  │   │              │   (lift)     │                               │   │   │
│  │   │              └──────────────┘                               │   │   │
│  │   │                                                             │   │   │
│  │   │   Properties:                                               │   │   │
│  │   │    • Nodes: 5 primitive actions                             │   │   │
│  │   │    • Edges: 4 sequential dependencies                       │   │   │
│  │   │    • Acyclic: ✓                                             │   │   │
│  │   │    • Single root: ✓                                         │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                Preconditions & Postconditions                        │   │
│  │                                                                      │   │
│  │  LOOK_AT:                                                           │   │
│  │    Pre:  robot.cameras.active = true                                │   │
│  │    Post: target.position.confidence > 0.9                           │   │
│  │                                                                      │   │
│  │  MOVE_TO (pre_grasp):                                               │   │
│  │    Pre:  target.position.known = true                               │   │
│  │    Post: arm.position = pre_grasp_position                          │   │
│  │                                                                      │   │
│  │  GRASP:                                                             │   │
│  │    Pre:  gripper.state = OPEN, arm.near(target)                     │   │
│  │    Post: gripper.holding = target                                   │   │
│  │                                                                      │   │
│  │  MOVE_TO (lift):                                                    │   │
│  │    Pre:  gripper.holding = target                                   │   │
│  │    Post: target.height > safe_height                                │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 4: PLAN VALIDATION                                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌────────────────────┐  ┌────────────────────┐  ┌────────────────────┐   │
│  │  Safety Checker    │  │ Feasibility Check  │  │ Resource Check     │   │
│  │                    │  │                    │  │                    │   │
│  │  ✓ Force limits    │  │  ✓ All positions   │  │  ✓ Single arm      │   │
│  │  ✓ Velocity caps   │  │    reachable       │  │    required        │   │
│  │  ✓ No collision    │  │  ✓ Object is       │  │  ✓ Gripper         │   │
│  │    paths exist     │  │    graspable       │  │    available       │   │
│  │  ✓ Exclusion zones │  │  ✓ No obstacles    │  │  ✓ No conflicts    │   │
│  │    avoided         │  │    blocking path   │  │                    │   │
│  │                    │  │                    │  │                    │   │
│  │  Status: PASS ✓    │  │  Status: PASS ✓    │  │  Status: PASS ✓    │   │
│  └────────────────────┘  └────────────────────┘  └────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     Validation Result                                │   │
│  │                                                                      │   │
│  │   Overall Status: VALIDATED ✓                                       │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │ If FAILED:                                                  │   │   │
│  │   │  • Return to LLM with failure reason                        │   │   │
│  │   │  • Request alternative plan                                 │   │   │
│  │   │  • Max 3 replanning attempts before human escalation        │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 5: ROS 2 ACTION DISPATCH                                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                   Action Graph → ROS 2 Actions                       │   │
│  │                                                                      │   │
│  │   ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐ │   │
│  │   │ Action Client   │    │ Action Server   │    │ Action Server   │ │   │
│  │   │   (Planner)     │───►│  (Look At)      │    │  (Move To)      │ │   │
│  │   └─────────────────┘    └─────────────────┘    └─────────────────┘ │   │
│  │                                                                      │   │
│  │   Topic: /vla/action_graph                                          │   │
│  │   Service: /vla/execute_plan                                        │   │
│  │   Action: /arm_controller/move_to                                   │   │
│  │   Action: /gripper_controller/grasp                                 │   │
│  │                                                                      │   │
│  │   Execution Pattern:                                                │   │
│  │   ┌───────────────────────────────────────────────────────────────┐ │   │
│  │   │ for action in topological_sort(action_graph):                 │ │   │
│  │   │     result = await execute_action(action)                     │ │   │
│  │   │     if result.failed:                                         │ │   │
│  │   │         trigger_replanning(action, result.error)              │ │   │
│  │   │     update_state(action, result)                              │ │   │
│  │   └───────────────────────────────────────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │
         ▼
┌──────────────────┐
│   To: Execution  │
│       Layer      │
└──────────────────┘
```

---

## Component Descriptions

### Stage 1: Context Assembly

**LLM Prompt Construction**
- **Purpose**: Build complete context for LLM reasoning
- **Inputs**: Scene state, robot capabilities, user command
- **Outputs**: Structured prompt with all necessary information
- **Key Details**: Prompt engineering is critical for reliable decomposition

**Scene State**
- **Purpose**: Provide environmental context
- **Inputs**: Object positions, spatial relationships, obstacles
- **Outputs**: JSON representation of current world state
- **Key Details**: Must be accurate and up-to-date

**Robot Capabilities**
- **Purpose**: Define available primitive actions
- **Inputs**: Robot configuration
- **Outputs**: List of primitives with parameters
- **Key Details**: Constrains LLM to valid actions only

### Stage 2: LLM Task Decomposition

**LLM Reasoning**
- **Purpose**: Break high-level command into primitives
- **Inputs**: Assembled prompt with context
- **Outputs**: Ordered list of primitive actions
- **Key Details**: Chain-of-thought improves reliability

**Chain-of-Thought**
- **Purpose**: Make LLM reasoning explicit and verifiable
- **Inputs**: Task and context
- **Outputs**: Step-by-step reasoning trace
- **Key Details**: Helps catch logical errors

### Stage 3: Action Graph Generation

**DAG Conversion**
- **Purpose**: Convert subtask list to dependency graph
- **Inputs**: Ordered subtask list
- **Outputs**: Directed acyclic graph of actions
- **Key Details**: Enables parallel execution where possible

**Preconditions & Postconditions**
- **Purpose**: Define execution requirements and effects
- **Inputs**: Action specifications
- **Outputs**: Formal pre/post conditions
- **Key Details**: Enables automated validation and replanning

### Stage 4: Plan Validation

**Safety Checker**
- **Purpose**: Verify plan respects safety constraints
- **Inputs**: Action graph, safety rules
- **Outputs**: Pass/fail with violation details
- **Key Details**: Non-negotiable safety enforcement

**Feasibility Check**
- **Purpose**: Verify physical execution is possible
- **Inputs**: Action graph, robot kinematics, scene
- **Outputs**: Reachability and collision analysis
- **Key Details**: Catches impossible plans early

**Resource Check**
- **Purpose**: Verify no resource conflicts
- **Inputs**: Action graph, resource requirements
- **Outputs**: Resource allocation or conflicts
- **Key Details**: Handles parallel action resource sharing

### Stage 5: ROS 2 Action Dispatch

**Action Graph to ROS 2**
- **Purpose**: Translate graph to ROS 2 action calls
- **Inputs**: Validated action graph
- **Outputs**: ROS 2 action goal messages
- **Key Details**: Uses standard ROS 2 action interface

**Execution Pattern**
- **Purpose**: Execute actions in dependency order
- **Inputs**: Topologically sorted actions
- **Outputs**: Execution results with feedback
- **Key Details**: Handles failures with replanning

---

## Data Flow

1. **Input Reception**: Voice pipeline delivers intent + entities
2. **Context Assembly**: Gather scene state, capabilities into prompt
3. **LLM Inference**: Generate decomposed subtask list
4. **Graph Creation**: Convert subtasks to DAG with dependencies
5. **Condition Attachment**: Add preconditions and postconditions
6. **Safety Validation**: Check against safety constraints
7. **Feasibility Validation**: Verify physical executability
8. **Resource Validation**: Check for conflicts
9. **ROS 2 Dispatch**: Send actions to execution layer
10. **Feedback Loop**: Handle failures with replanning

---

## Key Insights

### LLM as Reasoning Engine, Not Controller

The LLM's role is strictly reasoning and decomposition:
- It does NOT directly control motors
- It does NOT make real-time decisions
- It ONLY produces structured plans

This separation ensures safety and reliability.

### Structured Output Guarantees

LLM output must be parseable:
- JSON schema validation
- Constrained decoding (only valid primitives)
- Retry on malformed output

Production systems use constrained generation to guarantee valid outputs.

### Replanning as First-Class Feature

Plans rarely execute perfectly:
- Execution feedback triggers replanning
- LLM receives failure context for better retry
- Maximum retry limits prevent infinite loops

Robust systems plan for failure from the start.

---

## Related Concepts

- **From Module 1**: ROS 2 action servers and clients, topic-based feedback
- **From Module 2**: Simulation for plan testing before deployment
- **From Module 3**: GPU-accelerated LLM inference for low latency
- **In This Module**: VLA Architecture, Action Graph Structure, Voice Pipeline

---

## Real-World Application

LLM-based planning is being deployed in real robotic systems:

**Google's SayCan** combines LLM reasoning with learned affordances, using the LLM to propose actions while a separate model scores physical feasibility.

**Microsoft's ChatGPT for Robotics** demonstrates task decomposition for drone control, with the LLM generating Python code that interfaces with robot APIs.

**NVIDIA's Eureka** uses LLMs to generate reward functions for robot learning, showing how LLMs can contribute to robotics beyond direct planning.

The key insight from deployed systems is that LLMs excel at high-level reasoning but must be combined with physics-aware validation to ensure executable, safe plans.
>>>>>>> 5cf5f6818272e652910633754d301908ff0bb236
