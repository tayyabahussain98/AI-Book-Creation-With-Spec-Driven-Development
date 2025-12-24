---
title: Action Graph Structure
module: 4
chapter: 3
type: entity
---

# Action Graph Structure

## Overview

This diagram illustrates the structure of action graphs—directed acyclic graphs (DAGs) that represent decomposed robot tasks. Each node is a primitive action, and edges represent dependencies. Understanding this structure is essential for planning and executing complex robot behaviors.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                      ACTION GRAPH STRUCTURE                                 │
└─────────────────────────────────────────────────────────────────────────────┘

COMMAND: "Clean the table by putting all items in the bin"

┌─────────────────────────────────────────────────────────────────────────────┐
│                      GRAPH REPRESENTATION                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│                          ┌───────────────────┐                              │
│                          │    START (root)   │                              │
│                          │    action_id: 0   │                              │
│                          └─────────┬─────────┘                              │
│                                    │                                        │
│                                    ▼                                        │
│                          ┌───────────────────┐                              │
│                          │    LOOK_AT        │                              │
│                          │    target: table  │                              │
│                          │    action_id: 1   │                              │
│                          └─────────┬─────────┘                              │
│                                    │                                        │
│                                    ▼                                        │
│                          ┌───────────────────┐                              │
│                          │  IDENTIFY_OBJECTS │                              │
│                          │    on: table      │                              │
│                          │    action_id: 2   │                              │
│                          └─────────┬─────────┘                              │
│                                    │                                        │
│           ┌────────────────────────┼────────────────────────┐              │
│           │                        │                        │              │
│           ▼                        ▼                        ▼              │
│  ┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐     │
│  │  PICK (cup_01)  │      │  PICK (cup_02)  │      │  PICK (plate)   │     │
│  │  action_id: 3   │      │  action_id: 4   │      │  action_id: 5   │     │
│  └────────┬────────┘      └────────┬────────┘      └────────┬────────┘     │
│           │                        │                        │              │
│           ▼                        ▼                        ▼              │
│  ┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐     │
│  │  PLACE (bin)    │      │  PLACE (bin)    │      │  PLACE (bin)    │     │
│  │  action_id: 6   │      │  action_id: 7   │      │  action_id: 8   │     │
│  └────────┬────────┘      └────────┬────────┘      └────────┬────────┘     │
│           │                        │                        │              │
│           └────────────────────────┼────────────────────────┘              │
│                                    │                                        │
│                                    ▼                                        │
│                          ┌───────────────────┐                              │
│                          │    VERIFY         │                              │
│                          │    table: empty   │                              │
│                          │    action_id: 9   │                              │
│                          └─────────┬─────────┘                              │
│                                    │                                        │
│                                    ▼                                        │
│                          ┌───────────────────┐                              │
│                          │    END (leaf)     │                              │
│                          │    action_id: 10  │                              │
│                          └───────────────────┘                              │
│                                                                             │
│  Legend:                                                                    │
│    ─────► Sequential dependency (must complete before next)                │
│    Parallel branches can execute simultaneously                            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                      NODE STRUCTURE                                         │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     PrimitiveAction Node                             │   │
│  │                                                                      │   │
│  │   action_id: "pick_001"                                             │   │
│  │   action_type: PICK                                                 │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │ Parameters                                                  │   │   │
│  │   │   object_id: "cup_01"                                       │   │   │
│  │   │   grasp_type: "power"                                       │   │   │
│  │   │   force_limit: 15.0  # Newtons                              │   │   │
│  │   │   approach_direction: [0, 0, -1]                            │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │ Preconditions                                               │   │   │
│  │   │   • gripper.state == OPEN                                   │   │   │
│  │   │   • object.reachable == true                                │   │   │
│  │   │   • object.graspable == true                                │   │   │
│  │   │   • no_collision_path_exists == true                        │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │ Postconditions                                              │   │   │
│  │   │   • gripper.holding == object_id                            │   │   │
│  │   │   • object.attached_to == gripper                           │   │   │
│  │   │   • gripper.state == CLOSED                                 │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  │                                                                      │   │
│  │   ┌─────────────────────────────────────────────────────────────┐   │   │
│  │   │ Metadata                                                    │   │   │
│  │   │   timeout: 30.0  # seconds                                  │   │   │
│  │   │   retry_count: 2                                            │   │   │
│  │   │   priority: NORMAL                                          │   │   │
│  │   │   status: PENDING                                           │   │   │
│  │   └─────────────────────────────────────────────────────────────┘   │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                      EDGE TYPES                                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  1. SEQUENTIAL DEPENDENCY                                                   │
│     ┌────────┐                ┌────────┐                                   │
│     │  PICK  │───── must ────►│ PLACE  │                                   │
│     │        │    complete    │        │                                   │
│     └────────┘    before      └────────┘                                   │
│                                                                             │
│     • Postcondition of PICK enables precondition of PLACE                  │
│     • Strict ordering: PICK must succeed before PLACE starts               │
│                                                                             │
│  2. PARALLEL INDEPENDENCE                                                   │
│     ┌────────┐                                                             │
│     │PICK(A) │                                                             │
│     └────────┘   These can execute                                         │
│                  simultaneously if                                          │
│     ┌────────┐   resources allow                                           │
│     │PICK(B) │                                                             │
│     └────────┘                                                             │
│                                                                             │
│     • No shared preconditions                                              │
│     • Different target objects                                             │
│     • May require multi-arm robot or sequential with same arm              │
│                                                                             │
│  3. SYNCHRONIZATION (Join)                                                  │
│     ┌────────┐  ┌────────┐  ┌────────┐                                    │
│     │PLACE(A)│  │PLACE(B)│  │PLACE(C)│                                    │
│     └───┬────┘  └───┬────┘  └───┬────┘                                    │
│         │           │           │                                          │
│         └───────────┼───────────┘                                          │
│                     ▼                                                       │
│              ┌────────────┐                                                │
│              │   VERIFY   │                                                │
│              └────────────┘                                                │
│                                                                             │
│     • VERIFY waits for ALL preceding actions to complete                   │
│     • Synchronization point for parallel branches                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                      PRIMITIVE ACTION TYPES                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  MANIPULATION ACTIONS                                                │   │
│  │                                                                      │   │
│  │  PICK                                                               │   │
│  │    params: object_id, grasp_type, force_limit                       │   │
│  │    pre: gripper_open, object_reachable, path_clear                  │   │
│  │    post: gripper_holding_object                                     │   │
│  │                                                                      │   │
│  │  PLACE                                                              │   │
│  │    params: target_location, placement_pose                          │   │
│  │    pre: gripper_holding_object, location_reachable                  │   │
│  │    post: object_at_location, gripper_open                           │   │
│  │                                                                      │   │
│  │  RELEASE                                                            │   │
│  │    params: speed                                                    │   │
│  │    pre: gripper_closed                                              │   │
│  │    post: gripper_open                                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  NAVIGATION ACTIONS                                                  │   │
│  │                                                                      │   │
│  │  MOVE_TO                                                            │   │
│  │    params: target_pose, speed                                       │   │
│  │    pre: path_exists, no_collision                                   │   │
│  │    post: robot_at_pose                                              │   │
│  │                                                                      │   │
│  │  MOVE_BASE                                                          │   │
│  │    params: target_position, orientation                             │   │
│  │    pre: navigation_path_exists                                      │   │
│  │    post: base_at_position                                           │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  PERCEPTION ACTIONS                                                  │   │
│  │                                                                      │   │
│  │  LOOK_AT                                                            │   │
│  │    params: target_point                                             │   │
│  │    pre: cameras_active                                              │   │
│  │    post: target_in_view, pose_estimated                             │   │
│  │                                                                      │   │
│  │  IDENTIFY_OBJECTS                                                   │   │
│  │    params: region, object_types                                     │   │
│  │    pre: region_in_view                                              │   │
│  │    post: objects_detected, poses_estimated                          │   │
│  │                                                                      │   │
│  │  VERIFY                                                             │   │
│  │    params: condition                                                │   │
│  │    pre: sensors_active                                              │   │
│  │    post: condition_checked                                          │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  COMMUNICATION ACTIONS                                               │   │
│  │                                                                      │   │
│  │  SPEAK                                                              │   │
│  │    params: text, volume, language                                   │   │
│  │    pre: speaker_available                                           │   │
│  │    post: message_delivered                                          │   │
│  │                                                                      │   │
│  │  WAIT                                                               │   │
│  │    params: duration or condition                                    │   │
│  │    pre: none                                                        │   │
│  │    post: time_elapsed or condition_met                              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                      EXECUTION SEMANTICS                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  TOPOLOGICAL EXECUTION ORDER                                                │
│                                                                             │
│  Given the graph above, valid execution orders include:                     │
│                                                                             │
│  Sequential (single arm):                                                   │
│    START → LOOK_AT → IDENTIFY → PICK(cup_01) → PLACE → PICK(cup_02) →      │
│    PLACE → PICK(plate) → PLACE → VERIFY → END                              │
│                                                                             │
│  Parallel (dual arm):                                                       │
│    START → LOOK_AT → IDENTIFY →                                            │
│      Arm1: PICK(cup_01) → PLACE                                            │
│      Arm2: PICK(cup_02) → PLACE                                            │
│    Sync → PICK(plate) → PLACE → VERIFY → END                               │
│                                                                             │
│  ─────────────────────────────────────────────────────────────────────     │
│                                                                             │
│  STATE MACHINE FOR EACH NODE                                                │
│                                                                             │
│                    ┌─────────┐                                             │
│                    │ PENDING │                                             │
│                    └────┬────┘                                             │
│                         │ dependencies_satisfied                           │
│                         ▼                                                   │
│                    ┌─────────┐                                             │
│           ┌────────│EXECUTING│────────┐                                    │
│           │        └────┬────┘        │                                    │
│           │             │             │                                    │
│      timeout/      success       retry_possible                            │
│       error            │              │                                    │
│           │             │             │                                    │
│           ▼             ▼             ▼                                    │
│     ┌─────────┐   ┌─────────┐   ┌─────────┐                               │
│     │ FAILED  │   │COMPLETED│   │RETRYING │                               │
│     └─────────┘   └─────────┘   └────┬────┘                               │
│                                      │                                     │
│                                      └───► EXECUTING                       │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Descriptions

### Graph Properties

**Directed Acyclic Graph (DAG)**
- Nodes: Primitive actions to execute
- Edges: Dependencies between actions
- Acyclic: No circular dependencies allowed
- Single root: Entry point for execution

**Topological Ordering**
- Execution follows topological sort of nodes
- All predecessors must complete before a node starts
- Multiple valid orderings may exist

### Node Structure

**PrimitiveAction**
- **action_id**: Unique identifier for tracking
- **action_type**: Type of primitive (PICK, PLACE, MOVE_TO, etc.)
- **parameters**: Action-specific configuration
- **preconditions**: Conditions required before execution
- **postconditions**: State changes after successful execution
- **metadata**: Timeout, retry count, priority, current status

### Edge Types

**Sequential Dependency**
- Strict ordering between actions
- Postconditions enable preconditions of dependent actions
- Failure propagates to dependent actions

**Parallel Independence**
- Actions on different branches can run simultaneously
- No shared resources or state dependencies
- Requires multi-arm robot or time-sliced execution

**Synchronization (Join)**
- Multiple branches converge at a single node
- Join node waits for all predecessors
- Used for verification and completion checks

---

## Data Flow

1. **Task Decomposition**: LLM generates ordered subtask list
2. **Graph Construction**: Subtasks become nodes with dependencies
3. **Precondition Analysis**: System determines execution constraints
4. **Topological Sort**: Generate valid execution order
5. **Parallel Analysis**: Identify independent branches
6. **Resource Allocation**: Assign arms, grippers to actions
7. **Execution**: Run actions following graph structure
8. **State Updates**: Update postconditions as actions complete

---

## Key Insights

### Why DAGs, Not Sequences?

Sequential lists cannot express:
- Parallel opportunities (multiple objects to pick)
- Conditional branches (if object exists, pick it)
- Join points (wait for all picks before verify)

DAGs capture the true structure of robot tasks.

### Preconditions Enable Scheduling

By explicitly stating preconditions, the system can:
- Verify actions are safe before execution
- Identify earliest valid start time
- Detect conflicts between parallel actions
- Enable replanning when preconditions fail

### Failure Handling with Graphs

When an action fails, the graph structure enables:
- Identifying affected downstream actions
- Preserving completed work on other branches
- Replanning from the failure point
- Graceful degradation (partial task completion)

---

## Related Concepts

- **From Module 1**: ROS 2 action servers for primitive execution
- **From Module 2**: Simulated execution for plan validation
- **From Module 3**: GPU-accelerated motion planning
- **In This Module**: LLM Planning Pipeline, Execution Monitoring

---

## Real-World Application

Action graphs are used in industrial and research robotics:

**Task and Motion Planning (TAMP)** systems like PDDLStream generate action graphs from symbolic goals, with motion planners filling in continuous parameters.

**Behavior Trees** in game AI and robotics (used by Boston Dynamics) are a related formalism, representing tasks as trees with sequence, selector, and parallel nodes.

**Amazon Robotics** uses similar graph structures for warehouse fulfillment, where multiple robots coordinate pick-and-pack operations with complex dependencies.

The key insight is that real-world tasks rarely have a single correct sequence—graphs express the flexibility inherent in physical tasks.
