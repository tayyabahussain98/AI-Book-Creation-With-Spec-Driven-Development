<<<<<<< HEAD
---
sidebar_position: 3
title: Chapter 3 - LLM Planning
---

# Chapter 3: LLM-Based Cognitive Planning

## Introduction

When you ask a robot to "clean the table," how does it know to look at the table, identify each object, pick them up one by one, and place them in the bin? This chapter explores how Large Language Models enable robots to bridge the gap between abstract instructions and concrete action sequences through task decomposition and cognitive planning.

LLM-based planning represents a paradigm shift in robotics. Traditional planners required explicit domain models with hand-crafted rules. Modern LLM planners leverage vast world knowledge to decompose tasks, reason about preconditions and effects, and generate feasible action sequences—all from natural language specifications.

Building on the voice pipeline from Chapter 2, we now examine how structured intents become executable plans. This chapter covers task decomposition strategies, action graph generation, ROS 2 integration patterns, and the critical topic of error handling and replanning when the physical world doesn't cooperate with our plans.

### Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapters 1-2 of this module
- Understanding of directed acyclic graphs (DAGs)
- Familiarity with ROS 2 action servers from Module 1
- Basic understanding of LLM prompting techniques

### Learning Objectives

By the end of this chapter, you will be able to:
1. Design prompts that enable LLMs to decompose robot tasks into primitive actions
2. Construct action graphs with proper dependencies and execution semantics
3. Implement ROS 2 integration patterns for executing LLM-generated plans
4. Build replanning systems that recover from execution failures

---

## Core Concept 1: Natural Language to Action Graphs

### The Task Decomposition Challenge

Consider the command "Make me a coffee." A human instantly understands this implies:
- Navigate to the coffee machine
- Check if there's a cup (get one if not)
- Check water level (refill if needed)
- Check coffee beans (refill if needed)
- Insert cup, press brew button
- Wait for brewing to complete
- Bring the cup to the person

This implicit knowledge—called common-sense reasoning—is exactly what LLMs provide. They've learned from vast text corpora how tasks typically decompose, what preconditions exist, and what can go wrong.

### LLM as Task Decomposer

The LLM's role in planning is to generate **action graphs** from high-level commands. This involves:

1. **Understanding the Goal**: Parse what the user actually wants achieved
2. **Inferring Steps**: Generate the sequence of actions required
3. **Identifying Dependencies**: Determine which actions must precede others
4. **Parameterizing Actions**: Fill in specific targets, positions, and constraints

### Prompt Engineering for Decomposition

Effective task decomposition requires carefully engineered prompts:

```text
System Prompt:
You are a robot task planner. Given a command and world state,
decompose the task into primitive actions.

Available primitive actions:
- LOOK_AT(target): Point camera at target
- IDENTIFY(region): Detect objects in region
- MOVE_TO(position): Move robot arm to position
- PICK(object_id, grasp_type): Grasp specified object
- PLACE(location): Place held object at location
- RELEASE(): Open gripper
- VERIFY(condition): Check if condition is true

Current World State:
{scene_description}

Command: "{user_command}"

Output a JSON array of actions in execution order, with dependencies.
```

Key prompt design principles:
- **Constrain the action space**: List only available primitives
- **Provide world state**: LLM needs to know what exists
- **Specify output format**: JSON ensures parseable results
- **Limit scope**: Focus on immediate task, not general conversation

See the [LLM Planning Pipeline](../diagrams/module-4/llm-planning-pipeline.md) for the complete planning flow.

---

## Core Concept 2: Task Decomposition Strategies

### Hierarchical Decomposition

Complex tasks decompose hierarchically into simpler subtasks:

```text
Level 0: "Clean the table"
    │
    ├── Level 1: "Remove all items from table"
    │       │
    │       ├── Level 2: "Pick up cup_01"
    │       │       ├── LOOK_AT(cup_01)
    │       │       ├── MOVE_TO(pre_grasp)
    │       │       ├── MOVE_TO(grasp)
    │       │       ├── PICK(cup_01)
    │       │       └── MOVE_TO(lifted)
    │       │
    │       ├── Level 2: "Place cup_01 in bin"
    │       │       ├── MOVE_TO(above_bin)
    │       │       ├── RELEASE()
    │       │       └── MOVE_TO(retracted)
    │       │
    │       └── Level 2: [repeat for other objects]
    │
    └── Level 1: "Verify table is empty"
            └── VERIFY(table_empty)
```

This hierarchical structure enables:
- **Abstraction**: High levels hide implementation details
- **Reuse**: Common subtasks share implementations
- **Debugging**: Failures localize to specific levels

### Chain-of-Thought Decomposition

Encouraging LLMs to "think aloud" improves decomposition quality:

```text
User: Pick up the cup and give it to me.

LLM Chain-of-Thought:
1. First, I need to locate the cup in the scene
   → I see cup_01 (red) at position (0.5, 0.3, 0.1)

2. To pick it up, I need to:
   a) Look at the cup to confirm position
   b) Move to pre-grasp position above it
   c) Lower to grasp position
   d) Close gripper on cup
   e) Lift cup

3. To give it to the user, I need to:
   a) Move to handover position
   b) Wait for user to grab
   c) Release gripper

4. Generating action sequence...
```

Chain-of-thought improves reliability by:
- Making reasoning explicit and verifiable
- Catching logical errors before execution
- Providing explanation for debugging

### Conditional Decomposition

Real tasks have branches based on world state:

```python
# Pseudo-code for conditional decomposition
def decompose_get_drink(world_state):
    actions = [LOOK_AT("workspace")]

    drink_objects = find_objects(world_state, type="drink")

    if len(drink_objects) == 0:
        # No drinks visible - search first
        actions.append(SEARCH("kitchen"))
        actions.append(IDENTIFY("drinks"))

    if world_state.robot.holding_object:
        # Already holding something - put it down first
        actions.append(PLACE("nearby_surface"))

    # Now do the main task
    actions.extend([
        PICK(drink_objects[0]),
        MOVE_TO("handover_position"),
        WAIT("user_takes_object"),
        RELEASE()
    ])

    return actions
```

LLMs can generate such conditional logic by:
- Analyzing world state in the prompt
- Generating if-then structures in output
- Marking actions as conditional or optional

---

## Core Concept 3: ROS 2 Integration Patterns

### Action Servers for Primitives

Each primitive action maps to a ROS 2 action server:

```python
# Conceptual ROS 2 action server for PICK primitive
class PickActionServer(Node):
    def __init__(self):
        super().__init__('pick_action_server')

        self._action_server = ActionServer(
            self,
            PickAction,
            'pick',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        """Execute pick action with feedback."""
        request = goal_handle.request
        object_id = request.object_id
        grasp_type = request.grasp_type

        # Phase 1: Approach
        feedback = PickAction.Feedback()
        feedback.phase = "approaching"
        feedback.progress = 0.2
        goal_handle.publish_feedback(feedback)

        await self.move_to_pregrasp(object_id)

        # Phase 2: Grasp
        feedback.phase = "grasping"
        feedback.progress = 0.6
        goal_handle.publish_feedback(feedback)

        success = await self.close_gripper(grasp_type)

        # Phase 3: Lift
        if success:
            feedback.phase = "lifting"
            feedback.progress = 0.9
            goal_handle.publish_feedback(feedback)
            await self.lift_object()

        # Complete
        goal_handle.succeed()
        result = PickAction.Result()
        result.success = success
        result.object_held = object_id if success else ""
        return result
```

### Plan Executor Node

The plan executor coordinates action graph execution:

```python
class PlanExecutorNode(Node):
    """Executes action graphs by dispatching to action servers."""

    def __init__(self):
        super().__init__('plan_executor')

        # Action clients for each primitive
        self.action_clients = {
            'pick': ActionClient(self, PickAction, 'pick'),
            'place': ActionClient(self, PlaceAction, 'place'),
            'move_to': ActionClient(self, MoveToAction, 'move_to'),
            'look_at': ActionClient(self, LookAtAction, 'look_at'),
        }

        # Subscribe to plans
        self.plan_sub = self.create_subscription(
            ActionGraph, '/planner/action_graph',
            self.plan_callback, 10
        )

    async def execute_plan(self, graph: ActionGraph):
        """Execute actions in topological order."""
        for action in graph.topological_order():
            self.get_logger().info(f"Executing: {action.action_id}")

            # Get appropriate client
            client = self.action_clients[action.action_type.lower()]

            # Send goal and wait
            goal = self.build_goal(action)
            result = await client.send_goal_async(goal)

            if not result.success:
                self.get_logger().error(f"Action failed: {action.action_id}")
                await self.trigger_replan(action, result.error)
                return False

        return True
```

### Topic-Based Plan Updates

For dynamic replanning, use topics to broadcast plan changes:

```text
Topics:
  /planner/action_graph     : Current plan (ActionGraph msg)
  /planner/current_action   : Active action (String msg)
  /planner/plan_status      : Execution status (PlanStatus msg)

Services:
  /planner/replan           : Request replanning (ReplanRequest)
  /planner/abort            : Abort current plan (Empty)
```

---

## Core Concept 4: Error Handling and Replanning

### Failure Modes in Execution

Robot plans fail for many reasons:

| Failure Type | Example | Detection Method |
|--------------|---------|------------------|
| Object moved | Cup fell over | Visual verification |
| Grasp failed | Slipped during pick | Force/torque sensors |
| Collision | Hit unexpected obstacle | Collision detection |
| Timeout | Action took too long | Watchdog timer |
| Precondition | Required state not achieved | State checking |

### The Replanning Loop

When failure occurs, the system must recover:

```python
async def execute_with_recovery(self, graph: ActionGraph):
    """Execute plan with automatic replanning on failure."""
    max_replans = 3
    replan_count = 0

    while replan_count < max_replans:
        for action in graph.get_ready_actions():
            result = await self.execute_action(action)

            if result.success:
                action.status = ActionStatus.COMPLETED
            else:
                # Collect failure context
                failure_context = {
                    "failed_action": action.to_dict(),
                    "error": result.error,
                    "world_state": self.get_current_state(),
                    "completed_actions": [
                        a.action_id for a in graph.actions
                        if a.status == ActionStatus.COMPLETED
                    ]
                }

                # Request replan from LLM
                new_graph = await self.replan(failure_context)

                if new_graph:
                    graph = new_graph
                    replan_count += 1
                    break  # Restart execution loop
                else:
                    return False  # Cannot recover

        # Check if all done
        if all(a.status == ActionStatus.COMPLETED for a in graph.actions):
            return True

    return False  # Max replans exceeded
```

### Context-Aware Replanning

Effective replanning requires rich failure context:

```text
Replan Prompt:
The previous plan failed during execution. Generate a new plan
considering what happened.

Original Command: "Pick up the red cup"

Completed Actions:
- LOOK_AT(cup_01): SUCCESS
- MOVE_TO(pre_grasp): SUCCESS
- MOVE_TO(grasp): SUCCESS
- PICK(cup_01): FAILED

Failure Details:
- Error: GRASP_SLIP - Object slipped during grasp
- Force sensor reading: 2.1N (expected 5-10N)
- Object still at original position

Current World State:
- cup_01 is at (0.5, 0.3, 0.1) - same position
- Gripper is open

Generate a recovery plan. Consider:
1. Re-attempting with different grasp parameters
2. Repositioning the object if needed
3. Using a different grasp approach
```

The LLM can then generate intelligent recovery strategies based on the specific failure mode.

See the [Action Graph Structure](../diagrams/module-4/action-graph-structure.md) for detailed graph execution semantics.

---

## Hands-On Example

### Example: LLM Task Planner

The following code demonstrates task decomposition with action graph generation:

```python
#!/usr/bin/env python3
"""
LLM Task Planner - Decomposition and Action Graph Generation
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


class ActionType(Enum):
    MOVE_TO = auto()
    LOOK_AT = auto()
    PICK = auto()
    PLACE = auto()
    RELEASE = auto()
    VERIFY = auto()


@dataclass
class PrimitiveAction:
    """A single primitive action in the action graph."""
    action_id: str
    action_type: ActionType
    parameters: dict = field(default_factory=dict)
    dependencies: list[str] = field(default_factory=list)


@dataclass
class ActionGraph:
    """Directed acyclic graph of primitive actions."""
    actions: list[PrimitiveAction] = field(default_factory=list)

    def add_action(self, action: PrimitiveAction) -> None:
        self.actions.append(action)

    def topological_order(self) -> list[PrimitiveAction]:
        """Return actions in valid execution order."""
        # Kahn's algorithm for topological sort
        in_degree = {a.action_id: len(a.dependencies) for a in self.actions}
        queue = [a for a in self.actions if in_degree[a.action_id] == 0]
        result = []

        while queue:
            action = queue.pop(0)
            result.append(action)
            for a in self.actions:
                if action.action_id in a.dependencies:
                    in_degree[a.action_id] -= 1
                    if in_degree[a.action_id] == 0:
                        queue.append(a)

        return result


class LLMTaskPlanner:
    """Decomposes commands into action graphs."""

    def __init__(self):
        self.action_counter = 0

    def decompose_pick(self, object_id: str) -> ActionGraph:
        """Decompose PICK command into action sequence."""
        graph = ActionGraph()

        # 1. Look at target
        look = PrimitiveAction(
            action_id="look_001",
            action_type=ActionType.LOOK_AT,
            parameters={"target": object_id}
        )
        graph.add_action(look)

        # 2. Move to pre-grasp
        move_pre = PrimitiveAction(
            action_id="move_002",
            action_type=ActionType.MOVE_TO,
            parameters={"position": "pre_grasp"},
            dependencies=["look_001"]
        )
        graph.add_action(move_pre)

        # 3. Move to grasp
        move_grasp = PrimitiveAction(
            action_id="move_003",
            action_type=ActionType.MOVE_TO,
            parameters={"position": "grasp"},
            dependencies=["move_002"]
        )
        graph.add_action(move_grasp)

        # 4. Grasp object
        pick = PrimitiveAction(
            action_id="pick_004",
            action_type=ActionType.PICK,
            parameters={"object_id": object_id},
            dependencies=["move_003"]
        )
        graph.add_action(pick)

        # 5. Lift
        lift = PrimitiveAction(
            action_id="move_005",
            action_type=ActionType.MOVE_TO,
            parameters={"position": "lifted"},
            dependencies=["pick_004"]
        )
        graph.add_action(lift)

        return graph


# Usage
planner = LLMTaskPlanner()
graph = planner.decompose_pick("red_cup")

print("Action Graph for 'Pick up red cup':")
for action in graph.topological_order():
    deps = f" (after: {action.dependencies})" if action.dependencies else ""
    print(f"  {action.action_id}: {action.action_type.name}{deps}")
```

**Key Points**:
- Action graphs encode dependencies, not just sequences
- Topological sort produces valid execution orders
- Each primitive has explicit parameters
- Dependencies enable parallel execution where possible

For the complete implementation with all action types and validation, see the [LLM Task Planner](../code-examples/module-4/llm_task_planner.py) code example.

---

## Summary

This chapter explored how LLMs enable intelligent task planning for robots:

1. **Task Decomposition**: LLMs transform high-level commands into primitive action sequences by leveraging common-sense knowledge about how tasks decompose, what preconditions exist, and what steps are required.

2. **Decomposition Strategies**: Hierarchical decomposition breaks complex tasks into subtasks, chain-of-thought reasoning makes logic explicit, and conditional decomposition handles different world states.

3. **Action Graph Structure**: Plans are represented as directed acyclic graphs where nodes are primitive actions and edges are dependencies, enabling both sequential and parallel execution patterns.

4. **ROS 2 Integration**: Action servers execute primitives with feedback, plan executor nodes coordinate graph execution, and topic-based communication enables dynamic plan updates.

5. **Error Handling**: Plans fail due to object movement, grasp failures, collisions, and timeouts, requiring detection through sensors and state checking, with context-aware replanning for recovery.

6. **Replanning Strategy**: When actions fail, the system collects failure context, queries the LLM for recovery plans, and resumes execution, with limits on replan attempts to prevent infinite loops.

**Next Steps**: Chapter 4 explores multimodal fusion, showing how visual perception and language understanding combine to enable precise object grounding and context-aware robot behavior.

---

## Self-Assessment

Test your understanding of LLM-based cognitive planning:

1. **What is the role of the LLM in robot task planning, and what are its limitations?**
   <details>
   <summary>Show Answer</summary>
   The LLM's role is reasoning and decomposition: understanding goals, inferring steps, identifying dependencies, and parameterizing actions. Limitations include: no physical grounding (doesn't know if paths are collision-free), potential hallucination (may generate impossible actions), and latency too high for real-time control. LLMs plan; they don't control.
   </details>

2. **Why are action graphs (DAGs) used instead of simple action sequences?**
   <details>
   <summary>Show Answer</summary>
   DAGs capture dependencies that sequences cannot express: parallel opportunities (picking multiple objects simultaneously), conditional branches (different paths based on state), and join points (waiting for all picks before verify). They also enable better failure handling by preserving completed work on other branches and enabling replanning from specific failure points.
   </details>

3. **Describe the key elements of an effective task decomposition prompt.**
   <details>
   <summary>Show Answer</summary>
   Key elements: (1) Constrain action space - list only available primitives; (2) Provide world state - LLM needs to know what exists; (3) Specify output format - JSON ensures parseable results; (4) Define action semantics - what each primitive does; (5) Request dependencies - not just sequence; (6) Limit scope - focus on immediate task.
   </details>

4. **How does chain-of-thought prompting improve task decomposition quality?**
   <details>
   <summary>Show Answer</summary>
   Chain-of-thought makes reasoning explicit by having the LLM "think aloud" about the task. Benefits: (1) Reasoning is verifiable - can check logic before execution; (2) Errors are caught - logical mistakes surface in the reasoning; (3) Debugging is easier - can see why certain actions were chosen; (4) Improves reliability - structured thinking reduces hallucination.
   </details>

5. **Explain the ROS 2 integration pattern for executing LLM-generated plans.**
   <details>
   <summary>Show Answer</summary>
   Pattern: (1) Each primitive maps to a ROS 2 action server (pick_server, place_server); (2) Plan executor node holds action clients for all primitives; (3) Executor receives action graph on topic; (4) Executes actions in topological order using action clients; (5) Monitors feedback during execution; (6) On failure, publishes to replan service; (7) Plan updates broadcast via topics.
   </details>

6. **What information should be included in a replan request to help the LLM generate effective recovery strategies?**
   <details>
   <summary>Show Answer</summary>
   Include: (1) Original command and goal; (2) List of completed actions; (3) Failed action with parameters; (4) Specific error type and sensor readings; (5) Current world state after failure; (6) What changed from expected state; (7) Suggestions for recovery approaches. Rich context enables intelligent recovery rather than simple retry.
   </details>

7. **What are common failure modes in robot plan execution, and how is each detected?**
   <details>
   <summary>Show Answer</summary>
   Common failures: (1) Object moved - detected by visual verification comparing expected vs actual position; (2) Grasp failed - detected by force/torque sensors showing unexpected readings; (3) Collision - detected by collision detection or unexpected forces; (4) Timeout - detected by watchdog timers; (5) Precondition not met - detected by state checking before action execution.
   </details>

8. **Why is there typically a maximum limit on replan attempts, and what happens when it's exceeded?**
   <details>
   <summary>Show Answer</summary>
   Limits prevent infinite loops where the system keeps failing and replanning indefinitely. When exceeded: (1) System reports failure to user with context; (2) Asks for human guidance or modified goal; (3) May suggest alternative approaches; (4) Logs failure for analysis. Some scenarios genuinely can't be solved automatically and require human intervention.
   </details>
=======
---
sidebar_position: 3
title: Chapter 3 - LLM Planning
---

# Chapter 3: LLM-Based Cognitive Planning

## Introduction

When you ask a robot to "clean the table," how does it know to look at the table, identify each object, pick them up one by one, and place them in the bin? This chapter explores how Large Language Models enable robots to bridge the gap between abstract instructions and concrete action sequences through task decomposition and cognitive planning.

LLM-based planning represents a paradigm shift in robotics. Traditional planners required explicit domain models with hand-crafted rules. Modern LLM planners leverage vast world knowledge to decompose tasks, reason about preconditions and effects, and generate feasible action sequences—all from natural language specifications.

Building on the voice pipeline from Chapter 2, we now examine how structured intents become executable plans. This chapter covers task decomposition strategies, action graph generation, ROS 2 integration patterns, and the critical topic of error handling and replanning when the physical world doesn't cooperate with our plans.

### Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapters 1-2 of this module
- Understanding of directed acyclic graphs (DAGs)
- Familiarity with ROS 2 action servers from Module 1
- Basic understanding of LLM prompting techniques

### Learning Objectives

By the end of this chapter, you will be able to:
1. Design prompts that enable LLMs to decompose robot tasks into primitive actions
2. Construct action graphs with proper dependencies and execution semantics
3. Implement ROS 2 integration patterns for executing LLM-generated plans
4. Build replanning systems that recover from execution failures

---

## Core Concept 1: Natural Language to Action Graphs

### The Task Decomposition Challenge

Consider the command "Make me a coffee." A human instantly understands this implies:
- Navigate to the coffee machine
- Check if there's a cup (get one if not)
- Check water level (refill if needed)
- Check coffee beans (refill if needed)
- Insert cup, press brew button
- Wait for brewing to complete
- Bring the cup to the person

This implicit knowledge—called common-sense reasoning—is exactly what LLMs provide. They've learned from vast text corpora how tasks typically decompose, what preconditions exist, and what can go wrong.

### LLM as Task Decomposer

The LLM's role in planning is to generate **action graphs** from high-level commands. This involves:

1. **Understanding the Goal**: Parse what the user actually wants achieved
2. **Inferring Steps**: Generate the sequence of actions required
3. **Identifying Dependencies**: Determine which actions must precede others
4. **Parameterizing Actions**: Fill in specific targets, positions, and constraints

### Prompt Engineering for Decomposition

Effective task decomposition requires carefully engineered prompts:

```text
System Prompt:
You are a robot task planner. Given a command and world state,
decompose the task into primitive actions.

Available primitive actions:
- LOOK_AT(target): Point camera at target
- IDENTIFY(region): Detect objects in region
- MOVE_TO(position): Move robot arm to position
- PICK(object_id, grasp_type): Grasp specified object
- PLACE(location): Place held object at location
- RELEASE(): Open gripper
- VERIFY(condition): Check if condition is true

Current World State:
{scene_description}

Command: "{user_command}"

Output a JSON array of actions in execution order, with dependencies.
```

Key prompt design principles:
- **Constrain the action space**: List only available primitives
- **Provide world state**: LLM needs to know what exists
- **Specify output format**: JSON ensures parseable results
- **Limit scope**: Focus on immediate task, not general conversation

See the [LLM Planning Pipeline](../diagrams/module-4/llm-planning-pipeline.md) for the complete planning flow.

---

## Core Concept 2: Task Decomposition Strategies

### Hierarchical Decomposition

Complex tasks decompose hierarchically into simpler subtasks:

```text
Level 0: "Clean the table"
    │
    ├── Level 1: "Remove all items from table"
    │       │
    │       ├── Level 2: "Pick up cup_01"
    │       │       ├── LOOK_AT(cup_01)
    │       │       ├── MOVE_TO(pre_grasp)
    │       │       ├── MOVE_TO(grasp)
    │       │       ├── PICK(cup_01)
    │       │       └── MOVE_TO(lifted)
    │       │
    │       ├── Level 2: "Place cup_01 in bin"
    │       │       ├── MOVE_TO(above_bin)
    │       │       ├── RELEASE()
    │       │       └── MOVE_TO(retracted)
    │       │
    │       └── Level 2: [repeat for other objects]
    │
    └── Level 1: "Verify table is empty"
            └── VERIFY(table_empty)
```

This hierarchical structure enables:
- **Abstraction**: High levels hide implementation details
- **Reuse**: Common subtasks share implementations
- **Debugging**: Failures localize to specific levels

### Chain-of-Thought Decomposition

Encouraging LLMs to "think aloud" improves decomposition quality:

```text
User: Pick up the cup and give it to me.

LLM Chain-of-Thought:
1. First, I need to locate the cup in the scene
   → I see cup_01 (red) at position (0.5, 0.3, 0.1)

2. To pick it up, I need to:
   a) Look at the cup to confirm position
   b) Move to pre-grasp position above it
   c) Lower to grasp position
   d) Close gripper on cup
   e) Lift cup

3. To give it to the user, I need to:
   a) Move to handover position
   b) Wait for user to grab
   c) Release gripper

4. Generating action sequence...
```

Chain-of-thought improves reliability by:
- Making reasoning explicit and verifiable
- Catching logical errors before execution
- Providing explanation for debugging

### Conditional Decomposition

Real tasks have branches based on world state:

```python
# Pseudo-code for conditional decomposition
def decompose_get_drink(world_state):
    actions = [LOOK_AT("workspace")]

    drink_objects = find_objects(world_state, type="drink")

    if len(drink_objects) == 0:
        # No drinks visible - search first
        actions.append(SEARCH("kitchen"))
        actions.append(IDENTIFY("drinks"))

    if world_state.robot.holding_object:
        # Already holding something - put it down first
        actions.append(PLACE("nearby_surface"))

    # Now do the main task
    actions.extend([
        PICK(drink_objects[0]),
        MOVE_TO("handover_position"),
        WAIT("user_takes_object"),
        RELEASE()
    ])

    return actions
```

LLMs can generate such conditional logic by:
- Analyzing world state in the prompt
- Generating if-then structures in output
- Marking actions as conditional or optional

---

## Core Concept 3: ROS 2 Integration Patterns

### Action Servers for Primitives

Each primitive action maps to a ROS 2 action server:

```python
# Conceptual ROS 2 action server for PICK primitive
class PickActionServer(Node):
    def __init__(self):
        super().__init__('pick_action_server')

        self._action_server = ActionServer(
            self,
            PickAction,
            'pick',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        """Execute pick action with feedback."""
        request = goal_handle.request
        object_id = request.object_id
        grasp_type = request.grasp_type

        # Phase 1: Approach
        feedback = PickAction.Feedback()
        feedback.phase = "approaching"
        feedback.progress = 0.2
        goal_handle.publish_feedback(feedback)

        await self.move_to_pregrasp(object_id)

        # Phase 2: Grasp
        feedback.phase = "grasping"
        feedback.progress = 0.6
        goal_handle.publish_feedback(feedback)

        success = await self.close_gripper(grasp_type)

        # Phase 3: Lift
        if success:
            feedback.phase = "lifting"
            feedback.progress = 0.9
            goal_handle.publish_feedback(feedback)
            await self.lift_object()

        # Complete
        goal_handle.succeed()
        result = PickAction.Result()
        result.success = success
        result.object_held = object_id if success else ""
        return result
```

### Plan Executor Node

The plan executor coordinates action graph execution:

```python
class PlanExecutorNode(Node):
    """Executes action graphs by dispatching to action servers."""

    def __init__(self):
        super().__init__('plan_executor')

        # Action clients for each primitive
        self.action_clients = {
            'pick': ActionClient(self, PickAction, 'pick'),
            'place': ActionClient(self, PlaceAction, 'place'),
            'move_to': ActionClient(self, MoveToAction, 'move_to'),
            'look_at': ActionClient(self, LookAtAction, 'look_at'),
        }

        # Subscribe to plans
        self.plan_sub = self.create_subscription(
            ActionGraph, '/planner/action_graph',
            self.plan_callback, 10
        )

    async def execute_plan(self, graph: ActionGraph):
        """Execute actions in topological order."""
        for action in graph.topological_order():
            self.get_logger().info(f"Executing: {action.action_id}")

            # Get appropriate client
            client = self.action_clients[action.action_type.lower()]

            # Send goal and wait
            goal = self.build_goal(action)
            result = await client.send_goal_async(goal)

            if not result.success:
                self.get_logger().error(f"Action failed: {action.action_id}")
                await self.trigger_replan(action, result.error)
                return False

        return True
```

### Topic-Based Plan Updates

For dynamic replanning, use topics to broadcast plan changes:

```text
Topics:
  /planner/action_graph     : Current plan (ActionGraph msg)
  /planner/current_action   : Active action (String msg)
  /planner/plan_status      : Execution status (PlanStatus msg)

Services:
  /planner/replan           : Request replanning (ReplanRequest)
  /planner/abort            : Abort current plan (Empty)
```

---

## Core Concept 4: Error Handling and Replanning

### Failure Modes in Execution

Robot plans fail for many reasons:

| Failure Type | Example | Detection Method |
|--------------|---------|------------------|
| Object moved | Cup fell over | Visual verification |
| Grasp failed | Slipped during pick | Force/torque sensors |
| Collision | Hit unexpected obstacle | Collision detection |
| Timeout | Action took too long | Watchdog timer |
| Precondition | Required state not achieved | State checking |

### The Replanning Loop

When failure occurs, the system must recover:

```python
async def execute_with_recovery(self, graph: ActionGraph):
    """Execute plan with automatic replanning on failure."""
    max_replans = 3
    replan_count = 0

    while replan_count < max_replans:
        for action in graph.get_ready_actions():
            result = await self.execute_action(action)

            if result.success:
                action.status = ActionStatus.COMPLETED
            else:
                # Collect failure context
                failure_context = {
                    "failed_action": action.to_dict(),
                    "error": result.error,
                    "world_state": self.get_current_state(),
                    "completed_actions": [
                        a.action_id for a in graph.actions
                        if a.status == ActionStatus.COMPLETED
                    ]
                }

                # Request replan from LLM
                new_graph = await self.replan(failure_context)

                if new_graph:
                    graph = new_graph
                    replan_count += 1
                    break  # Restart execution loop
                else:
                    return False  # Cannot recover

        # Check if all done
        if all(a.status == ActionStatus.COMPLETED for a in graph.actions):
            return True

    return False  # Max replans exceeded
```

### Context-Aware Replanning

Effective replanning requires rich failure context:

```text
Replan Prompt:
The previous plan failed during execution. Generate a new plan
considering what happened.

Original Command: "Pick up the red cup"

Completed Actions:
- LOOK_AT(cup_01): SUCCESS
- MOVE_TO(pre_grasp): SUCCESS
- MOVE_TO(grasp): SUCCESS
- PICK(cup_01): FAILED

Failure Details:
- Error: GRASP_SLIP - Object slipped during grasp
- Force sensor reading: 2.1N (expected 5-10N)
- Object still at original position

Current World State:
- cup_01 is at (0.5, 0.3, 0.1) - same position
- Gripper is open

Generate a recovery plan. Consider:
1. Re-attempting with different grasp parameters
2. Repositioning the object if needed
3. Using a different grasp approach
```

The LLM can then generate intelligent recovery strategies based on the specific failure mode.

See the [Action Graph Structure](../diagrams/module-4/action-graph-structure.md) for detailed graph execution semantics.

---

## Hands-On Example

### Example: LLM Task Planner

The following code demonstrates task decomposition with action graph generation:

```python
#!/usr/bin/env python3
"""
LLM Task Planner - Decomposition and Action Graph Generation
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


class ActionType(Enum):
    MOVE_TO = auto()
    LOOK_AT = auto()
    PICK = auto()
    PLACE = auto()
    RELEASE = auto()
    VERIFY = auto()


@dataclass
class PrimitiveAction:
    """A single primitive action in the action graph."""
    action_id: str
    action_type: ActionType
    parameters: dict = field(default_factory=dict)
    dependencies: list[str] = field(default_factory=list)


@dataclass
class ActionGraph:
    """Directed acyclic graph of primitive actions."""
    actions: list[PrimitiveAction] = field(default_factory=list)

    def add_action(self, action: PrimitiveAction) -> None:
        self.actions.append(action)

    def topological_order(self) -> list[PrimitiveAction]:
        """Return actions in valid execution order."""
        # Kahn's algorithm for topological sort
        in_degree = {a.action_id: len(a.dependencies) for a in self.actions}
        queue = [a for a in self.actions if in_degree[a.action_id] == 0]
        result = []

        while queue:
            action = queue.pop(0)
            result.append(action)
            for a in self.actions:
                if action.action_id in a.dependencies:
                    in_degree[a.action_id] -= 1
                    if in_degree[a.action_id] == 0:
                        queue.append(a)

        return result


class LLMTaskPlanner:
    """Decomposes commands into action graphs."""

    def __init__(self):
        self.action_counter = 0

    def decompose_pick(self, object_id: str) -> ActionGraph:
        """Decompose PICK command into action sequence."""
        graph = ActionGraph()

        # 1. Look at target
        look = PrimitiveAction(
            action_id="look_001",
            action_type=ActionType.LOOK_AT,
            parameters={"target": object_id}
        )
        graph.add_action(look)

        # 2. Move to pre-grasp
        move_pre = PrimitiveAction(
            action_id="move_002",
            action_type=ActionType.MOVE_TO,
            parameters={"position": "pre_grasp"},
            dependencies=["look_001"]
        )
        graph.add_action(move_pre)

        # 3. Move to grasp
        move_grasp = PrimitiveAction(
            action_id="move_003",
            action_type=ActionType.MOVE_TO,
            parameters={"position": "grasp"},
            dependencies=["move_002"]
        )
        graph.add_action(move_grasp)

        # 4. Grasp object
        pick = PrimitiveAction(
            action_id="pick_004",
            action_type=ActionType.PICK,
            parameters={"object_id": object_id},
            dependencies=["move_003"]
        )
        graph.add_action(pick)

        # 5. Lift
        lift = PrimitiveAction(
            action_id="move_005",
            action_type=ActionType.MOVE_TO,
            parameters={"position": "lifted"},
            dependencies=["pick_004"]
        )
        graph.add_action(lift)

        return graph


# Usage
planner = LLMTaskPlanner()
graph = planner.decompose_pick("red_cup")

print("Action Graph for 'Pick up red cup':")
for action in graph.topological_order():
    deps = f" (after: {action.dependencies})" if action.dependencies else ""
    print(f"  {action.action_id}: {action.action_type.name}{deps}")
```

**Key Points**:
- Action graphs encode dependencies, not just sequences
- Topological sort produces valid execution orders
- Each primitive has explicit parameters
- Dependencies enable parallel execution where possible

For the complete implementation with all action types and validation, see the [LLM Task Planner](../code-examples/module-4/llm_task_planner.py) code example.

---

## Summary

This chapter explored how LLMs enable intelligent task planning for robots:

1. **Task Decomposition**: LLMs transform high-level commands into primitive action sequences by leveraging common-sense knowledge about how tasks decompose, what preconditions exist, and what steps are required.

2. **Decomposition Strategies**: Hierarchical decomposition breaks complex tasks into subtasks, chain-of-thought reasoning makes logic explicit, and conditional decomposition handles different world states.

3. **Action Graph Structure**: Plans are represented as directed acyclic graphs where nodes are primitive actions and edges are dependencies, enabling both sequential and parallel execution patterns.

4. **ROS 2 Integration**: Action servers execute primitives with feedback, plan executor nodes coordinate graph execution, and topic-based communication enables dynamic plan updates.

5. **Error Handling**: Plans fail due to object movement, grasp failures, collisions, and timeouts, requiring detection through sensors and state checking, with context-aware replanning for recovery.

6. **Replanning Strategy**: When actions fail, the system collects failure context, queries the LLM for recovery plans, and resumes execution, with limits on replan attempts to prevent infinite loops.

**Next Steps**: Chapter 4 explores multimodal fusion, showing how visual perception and language understanding combine to enable precise object grounding and context-aware robot behavior.

---

## Self-Assessment

Test your understanding of LLM-based cognitive planning:

1. **What is the role of the LLM in robot task planning, and what are its limitations?**
   <details>
   <summary>Show Answer</summary>
   The LLM's role is reasoning and decomposition: understanding goals, inferring steps, identifying dependencies, and parameterizing actions. Limitations include: no physical grounding (doesn't know if paths are collision-free), potential hallucination (may generate impossible actions), and latency too high for real-time control. LLMs plan; they don't control.
   </details>

2. **Why are action graphs (DAGs) used instead of simple action sequences?**
   <details>
   <summary>Show Answer</summary>
   DAGs capture dependencies that sequences cannot express: parallel opportunities (picking multiple objects simultaneously), conditional branches (different paths based on state), and join points (waiting for all picks before verify). They also enable better failure handling by preserving completed work on other branches and enabling replanning from specific failure points.
   </details>

3. **Describe the key elements of an effective task decomposition prompt.**
   <details>
   <summary>Show Answer</summary>
   Key elements: (1) Constrain action space - list only available primitives; (2) Provide world state - LLM needs to know what exists; (3) Specify output format - JSON ensures parseable results; (4) Define action semantics - what each primitive does; (5) Request dependencies - not just sequence; (6) Limit scope - focus on immediate task.
   </details>

4. **How does chain-of-thought prompting improve task decomposition quality?**
   <details>
   <summary>Show Answer</summary>
   Chain-of-thought makes reasoning explicit by having the LLM "think aloud" about the task. Benefits: (1) Reasoning is verifiable - can check logic before execution; (2) Errors are caught - logical mistakes surface in the reasoning; (3) Debugging is easier - can see why certain actions were chosen; (4) Improves reliability - structured thinking reduces hallucination.
   </details>

5. **Explain the ROS 2 integration pattern for executing LLM-generated plans.**
   <details>
   <summary>Show Answer</summary>
   Pattern: (1) Each primitive maps to a ROS 2 action server (pick_server, place_server); (2) Plan executor node holds action clients for all primitives; (3) Executor receives action graph on topic; (4) Executes actions in topological order using action clients; (5) Monitors feedback during execution; (6) On failure, publishes to replan service; (7) Plan updates broadcast via topics.
   </details>

6. **What information should be included in a replan request to help the LLM generate effective recovery strategies?**
   <details>
   <summary>Show Answer</summary>
   Include: (1) Original command and goal; (2) List of completed actions; (3) Failed action with parameters; (4) Specific error type and sensor readings; (5) Current world state after failure; (6) What changed from expected state; (7) Suggestions for recovery approaches. Rich context enables intelligent recovery rather than simple retry.
   </details>

7. **What are common failure modes in robot plan execution, and how is each detected?**
   <details>
   <summary>Show Answer</summary>
   Common failures: (1) Object moved - detected by visual verification comparing expected vs actual position; (2) Grasp failed - detected by force/torque sensors showing unexpected readings; (3) Collision - detected by collision detection or unexpected forces; (4) Timeout - detected by watchdog timers; (5) Precondition not met - detected by state checking before action execution.
   </details>

8. **Why is there typically a maximum limit on replan attempts, and what happens when it's exceeded?**
   <details>
   <summary>Show Answer</summary>
   Limits prevent infinite loops where the system keeps failing and replanning indefinitely. When exceeded: (1) System reports failure to user with context; (2) Asks for human guidance or modified goal; (3) May suggest alternative approaches; (4) Logs failure for analysis. Some scenarios genuinely can't be solved automatically and require human intervention.
   </details>
>>>>>>> 5cf5f6818272e652910633754d301908ff0bb236
