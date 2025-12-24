#!/usr/bin/env python3
"""
LLM Task Planner - Task Decomposition and Action Graph Generation
Module 4, Chapter 3: LLM Planning

This module demonstrates how LLMs can be used to decompose high-level
commands into executable action graphs. It shows prompt construction,
structured output parsing, and plan validation.

Note: This is an educational/architectural example. Production systems
would use actual LLM APIs with more sophisticated prompting.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional
import json


# =============================================================================
# Action and Graph Data Structures
# =============================================================================

class ActionType(Enum):
    """Primitive action types the robot can execute."""
    MOVE_TO = auto()       # Navigate to position
    LOOK_AT = auto()       # Direct gaze/camera
    PICK = auto()          # Grasp object
    PLACE = auto()         # Put object down
    RELEASE = auto()       # Open gripper
    VERIFY = auto()        # Check condition
    SPEAK = auto()         # Generate speech
    WAIT = auto()          # Time delay
    IDENTIFY = auto()      # Object detection


class ActionStatus(Enum):
    """Execution status of an action."""
    PENDING = auto()
    EXECUTING = auto()
    COMPLETED = auto()
    FAILED = auto()
    SKIPPED = auto()


@dataclass
class Condition:
    """A precondition or postcondition for an action."""
    predicate: str  # e.g., "gripper.open", "object.reachable"
    arguments: dict = field(default_factory=dict)
    negated: bool = False

    def __str__(self):
        neg = "NOT " if self.negated else ""
        args = ", ".join(f"{k}={v}" for k, v in self.arguments.items())
        return f"{neg}{self.predicate}({args})"


@dataclass
class PrimitiveAction:
    """
    A single primitive action in the action graph.

    Represents an atomic robot capability with preconditions
    and postconditions that define its execution requirements.
    """
    action_id: str
    action_type: ActionType
    parameters: dict = field(default_factory=dict)
    preconditions: list[Condition] = field(default_factory=list)
    postconditions: list[Condition] = field(default_factory=list)
    dependencies: list[str] = field(default_factory=list)  # IDs of prerequisite actions
    timeout_seconds: float = 30.0
    retry_count: int = 2
    status: ActionStatus = ActionStatus.PENDING

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        return {
            "action_id": self.action_id,
            "action_type": self.action_type.name,
            "parameters": self.parameters,
            "preconditions": [str(c) for c in self.preconditions],
            "postconditions": [str(c) for c in self.postconditions],
            "dependencies": self.dependencies,
            "status": self.status.name
        }


@dataclass
class ActionGraph:
    """
    A directed acyclic graph of primitive actions.

    Represents a decomposed task as a dependency graph where
    nodes are actions and edges are dependencies.
    """
    task_description: str
    actions: list[PrimitiveAction] = field(default_factory=list)
    root_action_id: Optional[str] = None

    def add_action(self, action: PrimitiveAction) -> None:
        """Add an action to the graph."""
        self.actions.append(action)
        if not self.root_action_id:
            self.root_action_id = action.action_id

    def get_action(self, action_id: str) -> Optional[PrimitiveAction]:
        """Get an action by ID."""
        for action in self.actions:
            if action.action_id == action_id:
                return action
        return None

    def get_ready_actions(self) -> list[PrimitiveAction]:
        """Get actions whose dependencies are all completed."""
        completed_ids = {
            a.action_id for a in self.actions
            if a.status == ActionStatus.COMPLETED
        }

        ready = []
        for action in self.actions:
            if action.status != ActionStatus.PENDING:
                continue
            if all(dep in completed_ids for dep in action.dependencies):
                ready.append(action)

        return ready

    def topological_order(self) -> list[PrimitiveAction]:
        """Return actions in valid execution order."""
        # Simple topological sort using Kahn's algorithm
        in_degree = {a.action_id: len(a.dependencies) for a in self.actions}
        queue = [a for a in self.actions if in_degree[a.action_id] == 0]
        result = []

        while queue:
            action = queue.pop(0)
            result.append(action)

            # Find actions that depend on this one
            for a in self.actions:
                if action.action_id in a.dependencies:
                    in_degree[a.action_id] -= 1
                    if in_degree[a.action_id] == 0:
                        queue.append(a)

        return result

    def to_dict(self) -> dict:
        """Convert to dictionary for serialization."""
        return {
            "task_description": self.task_description,
            "root_action_id": self.root_action_id,
            "actions": [a.to_dict() for a in self.actions]
        }


# =============================================================================
# Scene and Robot State
# =============================================================================

@dataclass
class SceneObject:
    """An object in the scene with position and attributes."""
    object_id: str
    object_type: str
    position: tuple[float, float, float]
    attributes: dict = field(default_factory=dict)
    is_reachable: bool = True
    is_graspable: bool = True


@dataclass
class RobotState:
    """Current state of the robot."""
    gripper_open: bool = True
    holding_object: Optional[str] = None
    arm_position: tuple[float, float, float] = (0.0, 0.0, 0.5)
    base_position: tuple[float, float] = (0.0, 0.0)


@dataclass
class WorldState:
    """Complete world state for planning."""
    objects: list[SceneObject] = field(default_factory=list)
    robot: RobotState = field(default_factory=RobotState)
    locations: dict = field(default_factory=dict)  # name -> position

    def to_context_string(self) -> str:
        """Convert to string for LLM prompt."""
        lines = ["Scene Objects:"]
        for obj in self.objects:
            attrs = ", ".join(f"{k}={v}" for k, v in obj.attributes.items())
            lines.append(f"  - {obj.object_id}: {obj.object_type} at {obj.position} [{attrs}]")

        lines.append("\nLocations:")
        for name, pos in self.locations.items():
            lines.append(f"  - {name}: {pos}")

        lines.append(f"\nRobot State:")
        lines.append(f"  - Gripper: {'open' if self.robot.gripper_open else 'closed'}")
        lines.append(f"  - Holding: {self.robot.holding_object or 'nothing'}")

        return "\n".join(lines)


# =============================================================================
# LLM Task Planner
# =============================================================================

class LLMTaskPlanner:
    """
    Decomposes high-level commands into action graphs using LLM reasoning.

    In production, this would call an actual LLM API. This implementation
    uses rule-based decomposition to demonstrate the architecture.
    """

    # Primitive action definitions for prompt
    PRIMITIVE_ACTIONS = """
Available primitive actions:
- LOOK_AT(target): Point camera at target to get visual information
- IDENTIFY(region): Detect and locate objects in a region
- MOVE_TO(position): Move robot arm to specified position
- PICK(object_id, grasp_type): Grasp specified object
- PLACE(location): Place held object at location
- RELEASE(): Open gripper to release object
- VERIFY(condition): Check if condition is true
- SPEAK(message): Say a message to the user
- WAIT(seconds): Pause execution for specified time
"""

    def __init__(self):
        self.action_counter = 0

    def _generate_action_id(self) -> str:
        """Generate unique action ID."""
        self.action_counter += 1
        return f"action_{self.action_counter:03d}"

    def build_prompt(self, command: str, world_state: WorldState) -> str:
        """
        Construct the LLM prompt for task decomposition.

        Args:
            command: Natural language command from user
            world_state: Current state of world and robot

        Returns:
            Complete prompt string for LLM
        """
        prompt = f"""You are a robot task planner. Given a command and world state,
decompose the task into a sequence of primitive actions.

{self.PRIMITIVE_ACTIONS}

Current World State:
{world_state.to_context_string()}

Command: "{command}"

Decompose this command into primitive actions. For each action, specify:
1. Action type and parameters
2. Prerequisites (which actions must complete first)
3. Expected outcome

Output as a JSON array of actions in execution order.
"""
        return prompt

    def decompose_task(
        self,
        command: str,
        world_state: WorldState,
        target_object_id: Optional[str] = None,
        target_location: Optional[str] = None
    ) -> ActionGraph:
        """
        Decompose a high-level command into an action graph.

        This is a rule-based implementation for demonstration.
        Production would use actual LLM inference.

        Args:
            command: Natural language command
            world_state: Current world state
            target_object_id: Resolved target object
            target_location: Target location if specified

        Returns:
            ActionGraph with decomposed actions
        """
        self.action_counter = 0
        graph = ActionGraph(task_description=command)

        command_lower = command.lower()

        # Decompose based on command type
        if "pick" in command_lower or "grab" in command_lower:
            self._decompose_pick(graph, target_object_id, world_state)

        elif "place" in command_lower or "put" in command_lower:
            self._decompose_place(graph, target_location, world_state)

        elif "clean" in command_lower or "clear" in command_lower:
            self._decompose_clean(graph, target_location, world_state)

        elif "give" in command_lower or "hand" in command_lower:
            self._decompose_give(graph, target_object_id, world_state)

        elif "find" in command_lower or "locate" in command_lower:
            self._decompose_find(graph, target_object_id, world_state)

        else:
            # Default: just look at the scene
            self._add_look_action(graph, "workspace")

        return graph

    def _decompose_pick(
        self,
        graph: ActionGraph,
        object_id: Optional[str],
        world_state: WorldState
    ) -> None:
        """Decompose PICK command into action sequence."""
        # 1. Look at target
        look_id = self._add_action(
            graph,
            ActionType.LOOK_AT,
            parameters={"target": object_id or "workspace"},
            preconditions=[Condition("camera.active")],
            postconditions=[Condition("target.visible", {"id": object_id})]
        )

        # 2. Move to pre-grasp position
        move_pre_id = self._add_action(
            graph,
            ActionType.MOVE_TO,
            parameters={"position": "pre_grasp", "relative_to": object_id},
            preconditions=[Condition("target.visible", {"id": object_id})],
            postconditions=[Condition("arm.at", {"position": "pre_grasp"})],
            dependencies=[look_id]
        )

        # 3. Move to grasp position
        move_grasp_id = self._add_action(
            graph,
            ActionType.MOVE_TO,
            parameters={"position": "grasp", "relative_to": object_id},
            preconditions=[Condition("arm.at", {"position": "pre_grasp"})],
            postconditions=[Condition("arm.at", {"position": "grasp"})],
            dependencies=[move_pre_id]
        )

        # 4. Grasp object
        grasp_id = self._add_action(
            graph,
            ActionType.PICK,
            parameters={"object_id": object_id, "grasp_type": "power"},
            preconditions=[
                Condition("gripper.open"),
                Condition("arm.at", {"position": "grasp"})
            ],
            postconditions=[
                Condition("gripper.holding", {"id": object_id})
            ],
            dependencies=[move_grasp_id]
        )

        # 5. Lift object
        self._add_action(
            graph,
            ActionType.MOVE_TO,
            parameters={"position": "lifted", "relative_to": object_id},
            preconditions=[Condition("gripper.holding", {"id": object_id})],
            postconditions=[Condition("object.lifted", {"id": object_id})],
            dependencies=[grasp_id]
        )

    def _decompose_place(
        self,
        graph: ActionGraph,
        location: Optional[str],
        world_state: WorldState
    ) -> None:
        """Decompose PLACE command into action sequence."""
        # Assumes robot is already holding an object

        # 1. Look at target location
        look_id = self._add_action(
            graph,
            ActionType.LOOK_AT,
            parameters={"target": location or "workspace"},
            preconditions=[Condition("camera.active")],
            postconditions=[Condition("location.visible", {"name": location})]
        )

        # 2. Move to pre-place position
        move_pre_id = self._add_action(
            graph,
            ActionType.MOVE_TO,
            parameters={"position": "pre_place", "relative_to": location},
            preconditions=[Condition("location.visible", {"name": location})],
            postconditions=[Condition("arm.at", {"position": "pre_place"})],
            dependencies=[look_id]
        )

        # 3. Move to place position
        move_place_id = self._add_action(
            graph,
            ActionType.MOVE_TO,
            parameters={"position": "place", "relative_to": location},
            preconditions=[Condition("arm.at", {"position": "pre_place"})],
            postconditions=[Condition("arm.at", {"position": "place"})],
            dependencies=[move_pre_id]
        )

        # 4. Release object
        release_id = self._add_action(
            graph,
            ActionType.RELEASE,
            parameters={},
            preconditions=[Condition("arm.at", {"position": "place"})],
            postconditions=[Condition("gripper.open")],
            dependencies=[move_place_id]
        )

        # 5. Retract arm
        self._add_action(
            graph,
            ActionType.MOVE_TO,
            parameters={"position": "retracted"},
            preconditions=[Condition("gripper.open")],
            postconditions=[Condition("arm.at", {"position": "retracted"})],
            dependencies=[release_id]
        )

    def _decompose_clean(
        self,
        graph: ActionGraph,
        location: Optional[str],
        world_state: WorldState
    ) -> None:
        """Decompose CLEAN command - pick up all objects and place in bin."""
        # 1. Look at target location
        look_id = self._add_action(
            graph,
            ActionType.LOOK_AT,
            parameters={"target": location or "table"},
            preconditions=[Condition("camera.active")],
            postconditions=[Condition("location.visible", {"name": location})]
        )

        # 2. Identify objects
        identify_id = self._add_action(
            graph,
            ActionType.IDENTIFY,
            parameters={"region": location or "table"},
            preconditions=[Condition("location.visible", {"name": location})],
            postconditions=[Condition("objects.identified")],
            dependencies=[look_id]
        )

        # 3. For each object in scene on target location, add pick-place sequence
        objects_on_location = [
            obj for obj in world_state.objects
            if obj.is_graspable
        ]

        previous_place_ids = []
        for obj in objects_on_location:
            # Pick action
            pick_id = self._add_action(
                graph,
                ActionType.PICK,
                parameters={"object_id": obj.object_id, "grasp_type": "power"},
                preconditions=[
                    Condition("gripper.open"),
                    Condition("objects.identified")
                ],
                postconditions=[Condition("gripper.holding", {"id": obj.object_id})],
                dependencies=[identify_id] + previous_place_ids
            )

            # Place in bin
            place_id = self._add_action(
                graph,
                ActionType.PLACE,
                parameters={"location": "bin"},
                preconditions=[Condition("gripper.holding", {"id": obj.object_id})],
                postconditions=[
                    Condition("object.at", {"id": obj.object_id, "location": "bin"}),
                    Condition("gripper.open")
                ],
                dependencies=[pick_id]
            )
            previous_place_ids = [place_id]

        # 4. Verify clean
        self._add_action(
            graph,
            ActionType.VERIFY,
            parameters={"condition": f"{location or 'table'}_is_empty"},
            preconditions=[Condition("gripper.open")],
            postconditions=[Condition("location.empty", {"name": location})],
            dependencies=previous_place_ids
        )

    def _decompose_give(
        self,
        graph: ActionGraph,
        object_id: Optional[str],
        world_state: WorldState
    ) -> None:
        """Decompose GIVE command - pick object and hand to person."""
        # First pick the object
        self._decompose_pick(graph, object_id, world_state)
        last_action_id = graph.actions[-1].action_id

        # Move to handover position
        move_handover_id = self._add_action(
            graph,
            ActionType.MOVE_TO,
            parameters={"position": "handover"},
            preconditions=[Condition("gripper.holding", {"id": object_id})],
            postconditions=[Condition("arm.at", {"position": "handover"})],
            dependencies=[last_action_id]
        )

        # Speak to indicate ready
        speak_id = self._add_action(
            graph,
            ActionType.SPEAK,
            parameters={"message": "Here you go. Please take the object."},
            preconditions=[Condition("arm.at", {"position": "handover"})],
            postconditions=[Condition("message.delivered")],
            dependencies=[move_handover_id]
        )

        # Wait for person to take object
        wait_id = self._add_action(
            graph,
            ActionType.WAIT,
            parameters={"seconds": 5.0, "or_condition": "object_taken"},
            preconditions=[Condition("message.delivered")],
            postconditions=[Condition("handover.ready")],
            dependencies=[speak_id]
        )

        # Release
        self._add_action(
            graph,
            ActionType.RELEASE,
            parameters={},
            preconditions=[Condition("handover.ready")],
            postconditions=[Condition("gripper.open")],
            dependencies=[wait_id]
        )

    def _decompose_find(
        self,
        graph: ActionGraph,
        object_id: Optional[str],
        world_state: WorldState
    ) -> None:
        """Decompose FIND command - search for object."""
        # Look around workspace
        look_id = self._add_action(
            graph,
            ActionType.LOOK_AT,
            parameters={"target": "workspace"},
            preconditions=[Condition("camera.active")],
            postconditions=[Condition("workspace.scanned")]
        )

        # Identify objects
        identify_id = self._add_action(
            graph,
            ActionType.IDENTIFY,
            parameters={"region": "workspace", "target_type": object_id},
            preconditions=[Condition("workspace.scanned")],
            postconditions=[Condition("objects.identified")],
            dependencies=[look_id]
        )

        # Report result
        self._add_action(
            graph,
            ActionType.SPEAK,
            parameters={"message": f"I {'found' if object_id else 'scanned'} the area."},
            preconditions=[Condition("objects.identified")],
            postconditions=[Condition("result.reported")],
            dependencies=[identify_id]
        )

    def _add_action(
        self,
        graph: ActionGraph,
        action_type: ActionType,
        parameters: dict,
        preconditions: list[Condition],
        postconditions: list[Condition],
        dependencies: list[str] = None
    ) -> str:
        """Helper to add action to graph."""
        action_id = self._generate_action_id()
        action = PrimitiveAction(
            action_id=action_id,
            action_type=action_type,
            parameters=parameters,
            preconditions=preconditions,
            postconditions=postconditions,
            dependencies=dependencies or []
        )
        graph.add_action(action)
        return action_id

    def _add_look_action(self, graph: ActionGraph, target: str) -> str:
        """Add a simple look action."""
        return self._add_action(
            graph,
            ActionType.LOOK_AT,
            parameters={"target": target},
            preconditions=[Condition("camera.active")],
            postconditions=[Condition("target.visible", {"target": target})]
        )


# =============================================================================
# Plan Validator
# =============================================================================

class PlanValidator:
    """Validates action graphs for safety and feasibility."""

    def validate(self, graph: ActionGraph, world_state: WorldState) -> tuple[bool, list[str]]:
        """
        Validate an action graph.

        Args:
            graph: The action graph to validate
            world_state: Current world state

        Returns:
            Tuple of (is_valid, list of issues)
        """
        issues = []

        # Check 1: Graph is acyclic
        if not self._is_acyclic(graph):
            issues.append("Graph contains cycles - invalid execution order")

        # Check 2: All dependencies exist
        action_ids = {a.action_id for a in graph.actions}
        for action in graph.actions:
            for dep in action.dependencies:
                if dep not in action_ids:
                    issues.append(f"Action {action.action_id} depends on non-existent action {dep}")

        # Check 3: Root action has no dependencies
        if graph.root_action_id:
            root = graph.get_action(graph.root_action_id)
            if root and root.dependencies:
                issues.append("Root action should not have dependencies")

        # Check 4: PICK requires valid target
        for action in graph.actions:
            if action.action_type == ActionType.PICK:
                target_id = action.parameters.get("object_id")
                if target_id and not any(o.object_id == target_id for o in world_state.objects):
                    issues.append(f"PICK target {target_id} not found in scene")

        # Check 5: No consecutive PICK without RELEASE/PLACE
        prev_action = None
        for action in graph.topological_order():
            if action.action_type == ActionType.PICK:
                if prev_action and prev_action.action_type == ActionType.PICK:
                    issues.append("Two consecutive PICK actions without RELEASE/PLACE")
            if action.action_type in [ActionType.PICK, ActionType.RELEASE, ActionType.PLACE]:
                prev_action = action

        return len(issues) == 0, issues

    def _is_acyclic(self, graph: ActionGraph) -> bool:
        """Check if graph is acyclic using DFS."""
        visited = set()
        rec_stack = set()

        def dfs(action_id: str) -> bool:
            visited.add(action_id)
            rec_stack.add(action_id)

            action = graph.get_action(action_id)
            if action:
                for dep in action.dependencies:
                    if dep not in visited:
                        if not dfs(dep):
                            return False
                    elif dep in rec_stack:
                        return False

            rec_stack.remove(action_id)
            return True

        for action in graph.actions:
            if action.action_id not in visited:
                if not dfs(action.action_id):
                    return False

        return True


# =============================================================================
# Usage Example
# =============================================================================

def main():
    """Demonstrate LLM task planning pipeline."""
    print("=" * 60)
    print("LLM Task Planner Demo")
    print("=" * 60)

    # Set up world state
    world_state = WorldState(
        objects=[
            SceneObject("cup_01", "cup", (0.5, 0.3, 0.1), {"color": "red"}),
            SceneObject("cup_02", "cup", (0.6, 0.3, 0.1), {"color": "blue"}),
            SceneObject("plate_01", "plate", (0.4, 0.4, 0.05), {"color": "white"}),
        ],
        robot=RobotState(gripper_open=True),
        locations={
            "table": (0.5, 0.35, 0.0),
            "bin": (0.0, 0.5, 0.0),
            "shelf": (0.0, 0.0, 0.5)
        }
    )

    # Initialize planner and validator
    planner = LLMTaskPlanner()
    validator = PlanValidator()

    # Test commands
    test_commands = [
        ("Pick up the red cup", "cup_01", None),
        ("Put it on the shelf", None, "shelf"),
        ("Clean the table", None, "table"),
    ]

    for command, target_obj, target_loc in test_commands:
        print(f"\n--- Command: \"{command}\" ---\n")

        # Generate action graph
        graph = planner.decompose_task(
            command=command,
            world_state=world_state,
            target_object_id=target_obj,
            target_location=target_loc
        )

        # Validate
        is_valid, issues = validator.validate(graph, world_state)

        # Display results
        print(f"Generated {len(graph.actions)} actions:")
        for action in graph.topological_order():
            deps = f" (after: {', '.join(action.dependencies)})" if action.dependencies else ""
            params = ", ".join(f"{k}={v}" for k, v in action.parameters.items())
            print(f"  {action.action_id}: {action.action_type.name}({params}){deps}")

        print(f"\nValidation: {'PASS' if is_valid else 'FAIL'}")
        for issue in issues:
            print(f"  - {issue}")

    print("\n" + "=" * 60)
    print("Demo complete")
    print("=" * 60)


if __name__ == "__main__":
    main()
