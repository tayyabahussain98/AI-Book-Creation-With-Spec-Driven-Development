<<<<<<< HEAD
---
sidebar_position: 4
title: Chapter 4 - Multimodal Fusion
---

# Chapter 4: Multimodal Fusion

## Introduction

When you say "pick up the red cup on the table," your robot must do something remarkable: it must connect abstract language to concrete visual reality. The word "cup" must become a specific object with a specific position that the robot can actually grasp. This chapter explores multimodal fusion—the techniques that enable robots to bridge language and vision into unified understanding.

Multimodal fusion is the cornerstone of practical VLA systems. Without it, language understanding remains theoretical—the robot might know you want "a cup" but not which physical object in the scene matches that description. This visual grounding transforms intent into action by providing the precise coordinates, orientation, and identity needed for robot manipulation.

Building on the planning capabilities from Chapter 3, we now explore how vision and language combine to create context-aware robot behavior. This chapter covers fusion architectures, visual grounding algorithms, and how temporal dynamics affect multimodal understanding in real robot deployments.

### Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapters 1-3 of this module
- Understanding of transformer attention mechanisms
- Familiarity with object detection concepts from Module 3
- Basic linear algebra knowledge (vectors, matrices)

### Learning Objectives

By the end of this chapter, you will be able to:
1. Compare early, late, and hybrid fusion architectures for vision-language integration
2. Implement visual grounding to resolve natural language references to scene objects
3. Design context-aware systems that leverage scene understanding for disambiguation
4. Handle temporal dynamics in multimodal perception for moving objects and changing scenes

---

## Core Concept 1: Vision-Language-Motion Fusion

### The Multimodal Challenge

Robots receive information through multiple channels simultaneously:
- **Vision**: RGB images, depth maps, point clouds
- **Language**: Voice commands, text instructions
- **Audio**: Environmental sounds, speech
- **Proprioception**: Joint positions, forces, velocities

The challenge is combining these disparate signals into coherent understanding. Each modality has different:
- **Dimensionality**: Images are high-dimensional, text is sequential
- **Sampling rates**: Vision at 30Hz, audio at 16kHz, language as events
- **Semantics**: Images show appearance, language conveys intent

### Fusion Architecture Options

Three main approaches exist for combining modalities:

**Early Fusion** concatenates raw or lightly-processed features:
```text
Vision features ─┐
                 ├──► Concatenate ──► Joint Model ──► Output
Language features┘
```

Advantages:
- Enables rich cross-modal interactions
- Can learn implicit alignments
- Single model to optimize

Disadvantages:
- Requires large aligned training data
- Computationally expensive
- All modalities must be present

**Late Fusion** processes each modality independently, combining only at decision time:
```text
Vision ──► Vision Model ──► Vision Decision ─┐
                                             ├──► Aggregate
Language ──► Language Model ──► Lang Decision┘
```

Advantages:
- Modular and interpretable
- Robust to missing modalities
- Uses pre-trained unimodal models

Disadvantages:
- Limited cross-modal reasoning
- May miss complementary information
- Decision aggregation is simplistic

**Hybrid Fusion** (recommended for robotics) combines both:
```text
Vision ────► Cross-Attention ◄──── Language
                   │
            Fused Features
                   │
            ┌──────┴──────┐
            ▼             ▼
        Action Head   Grounding Head
```

See the [Multimodal Fusion Architecture](../diagrams/module-4/multimodal-fusion-architecture.md) for detailed visualization.

### Cross-Attention for Vision-Language

Cross-attention is the key mechanism for vision-language alignment:

```python
# Conceptual cross-attention
def cross_attention(query, key, value):
    """
    Query: Language tokens [batch, seq_len, dim]
    Key/Value: Vision patches [batch, num_patches, dim]
    """
    # Compute attention weights
    scores = query @ key.transpose(-2, -1) / sqrt(dim)
    weights = softmax(scores, dim=-1)

    # Attend to visual features
    output = weights @ value
    return output, weights
```

When processing "the red cup":
1. "red" token attends strongly to patches with red color
2. "cup" token attends to cup-shaped regions
3. Combined attention localizes the red cup

---

## Core Concept 2: Visual Grounding and Referring Expressions

### The Grounding Problem

Visual grounding resolves natural language references to specific scene objects:

| Expression | Challenge |
|------------|-----------|
| "the cup" | Multiple cups may exist |
| "the red one" | Requires attribute matching |
| "the cup on the left" | Requires spatial reasoning |
| "that one" | Requires dialogue context |

### Referring Expression Structure

Referring expressions have compositional structure:

```text
"the large red cup on the table"
 │    │     │    │   │    │
 │    │     │    │   │    └── Anchor noun
 │    │     │    │   └── Spatial preposition
 │    │     │    └── Head noun (target class)
 │    │     └── Color attribute
 │    └── Size attribute
 └── Determiner
```

Parsing this structure enables systematic grounding:
1. Find objects of class "cup"
2. Filter by color = red
3. Filter by size = large (relative to other cups)
4. Verify spatial relation: ON(target, table)

### The Grounding Pipeline

Visual grounding proceeds through stages:

```text
1. CANDIDATE GENERATION
   └── Find all objects matching head noun class

2. ATTRIBUTE FILTERING
   ├── Color: detected_color == "red"
   └── Size: volume > median(cups)

3. SPATIAL VERIFICATION
   └── Check ON(candidate, anchor)

4. CONFIDENCE SCORING
   └── Combined score = Π(individual_confidences)

5. SELECTION
   ├── If unique high-confidence match → return
   └── If ambiguous → request clarification
```

### Handling Ambiguity

When multiple objects match, the system must handle ambiguity gracefully:

```python
def handle_ambiguity(candidates: list, confidence_threshold: float):
    """Handle ambiguous grounding results."""
    if len(candidates) == 0:
        return ask_clarification("I don't see any matching objects.")

    if len(candidates) == 1 and candidates[0].confidence > confidence_threshold:
        return candidates[0]  # Unique confident match

    # Multiple candidates - need disambiguation
    if candidates[0].confidence - candidates[1].confidence < 0.1:
        # Close scores - ask user
        options = describe_differences(candidates[:3])
        return ask_clarification(f"Which one? {options}")

    # Clear winner despite multiple candidates
    return candidates[0]
```

See the [Visual Grounding Flow](../diagrams/module-4/visual-grounding-flow.md) for the complete grounding process.

---

## Core Concept 3: Context-Aware Decision Making

### Scene Context

Beyond individual object detection, scene context provides crucial information:

**Spatial Layout**: Object positions relative to each other
- "The cup next to the bottle" requires understanding proximity
- "The rightmost cup" requires ordering objects spatially

**Functional Context**: What objects afford
- A cup can be grasped, filled, placed
- A table supports objects, defines workspace

**Historical Context**: What was mentioned before
- "Pick up the cup. Now put it down." - "it" refers to the cup
- Recent mentions have higher salience

### Scene Graph Representation

Scene graphs capture object relationships:

```text
Scene Graph for "table with red cup and blue bottle":

        ┌─────────┐
        │  Table  │
        └────┬────┘
             │ SUPPORTS
     ┌───────┴───────┐
     │               │
┌────▼────┐    ┌────▼────┐
│ Red Cup │    │Blue Btl │
└────┬────┘    └─────────┘
     │ NEXT_TO
     └──────────────────►
```

Scene graphs enable:
- Relation-based queries ("cup on table")
- Spatial reasoning ("cup next to bottle")
- Consistency checking ("cup can't be on floor AND table")

### Contextual Disambiguation

Context resolves ambiguity without explicit clarification:

**Recent Mention**: If user just said "the red cup," then "it" or "that" refers to the red cup.

**Task Context**: In a "set the table" task, "the plate" likely means an unused plate, not one already placed.

**Robot State**: If robot is holding an object, "put it down" refers to that object.

**User Gaze/Pointing**: Gesture input can disambiguate "that one" (though requires additional sensing).

```python
class ContextManager:
    """Manages dialogue and scene context for disambiguation."""

    def __init__(self):
        self.recent_objects = []  # Stack of mentioned objects
        self.current_task = None
        self.robot_holding = None

    def resolve_pronoun(self, pronoun: str) -> Optional[Object]:
        """Resolve pronouns using context."""
        if pronoun in ["it", "that", "this"]:
            if self.recent_objects:
                return self.recent_objects[-1]
            if self.robot_holding:
                return self.robot_holding
        return None

    def update_context(self, grounded_object: Object):
        """Update context with newly grounded object."""
        self.recent_objects.append(grounded_object)
        # Keep only recent history
        self.recent_objects = self.recent_objects[-5:]
```

---

## Core Concept 4: Temporal Fusion for Dynamic Environments

### The Dynamic World Problem

Real environments change during task execution:
- Objects move (pushed, fall, picked up)
- People enter and leave
- Lighting changes
- Occlusions appear and disappear

Static grounding is insufficient—systems must track objects through time.

### Temporal Object Tracking

Multi-object tracking maintains identity over time:

```text
Frame t:    [cup_01 @ (0.5, 0.3)]  [bottle_01 @ (0.6, 0.3)]
                     │                      │
                     │ Association          │ Association
                     │                      │
Frame t+1:  [cup_01 @ (0.5, 0.31)] [bottle_01 @ (0.6, 0.3)]
```

Tracking enables:
- Consistent object references across frames
- Motion prediction for moving targets
- Re-identification after occlusion

### Fusion Across Time

Temporal fusion combines observations over time:

```python
class TemporalFusion:
    """Fuse observations across time for robust grounding."""

    def __init__(self, history_length: int = 10):
        self.observations = {}  # object_id -> list of observations
        self.history_length = history_length

    def update(self, object_id: str, observation: dict):
        """Add new observation for object."""
        if object_id not in self.observations:
            self.observations[object_id] = []

        self.observations[object_id].append(observation)

        # Keep bounded history
        if len(self.observations[object_id]) > self.history_length:
            self.observations[object_id].pop(0)

    def get_fused_estimate(self, object_id: str) -> dict:
        """Get temporally smoothed estimate."""
        history = self.observations.get(object_id, [])

        if not history:
            return None

        # Simple averaging (Kalman filter in production)
        avg_position = [
            sum(obs['position'][i] for obs in history) / len(history)
            for i in range(3)
        ]

        # Confidence increases with consistent observations
        confidence = min(len(history) / self.history_length, 1.0)

        return {
            'position': avg_position,
            'confidence': confidence * history[-1]['detection_confidence']
        }
```

### Handling Object State Changes

Objects change state during manipulation:
- **Grasped**: Attached to gripper, follows arm
- **Placed**: New position on surface
- **Moved by human**: Position update from detection

```python
class ObjectStateTracker:
    """Track object states through manipulation."""

    def __init__(self):
        self.object_states = {}  # object_id -> state

    def grasp_object(self, object_id: str):
        """Mark object as grasped by robot."""
        self.object_states[object_id] = {
            'state': 'GRASPED',
            'attached_to': 'gripper',
            'last_known_position': None  # Now follows gripper
        }

    def release_object(self, object_id: str, position: tuple):
        """Mark object as released at position."""
        self.object_states[object_id] = {
            'state': 'PLACED',
            'attached_to': None,
            'last_known_position': position
        }

    def get_object_position(self, object_id: str, gripper_pos: tuple):
        """Get current object position considering state."""
        state = self.object_states.get(object_id, {})

        if state.get('state') == 'GRASPED':
            return gripper_pos  # Object follows gripper

        return state.get('last_known_position')
```

---

## Hands-On Example

### Example: Multimodal Fusion Node

The following code demonstrates visual grounding for robot commands:

```python
#!/usr/bin/env python3
"""
Multimodal Fusion - Visual Grounding for Robot Commands
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class DetectedObject:
    """An object detected in the visual scene."""
    object_id: str
    class_name: str
    position: tuple[float, float, float]
    confidence: float
    attributes: dict


@dataclass
class GroundedReference:
    """Result of grounding a referring expression."""
    object: Optional[DetectedObject]
    confidence: float
    is_ambiguous: bool


class VisualGrounder:
    """Grounds referring expressions to scene objects."""

    def ground(
        self,
        expression: str,
        detected_objects: list[DetectedObject]
    ) -> GroundedReference:
        """Ground a referring expression to a specific object."""
        # Parse expression (simplified)
        head_noun = self._extract_noun(expression)
        color = self._extract_color(expression)

        # Filter by class
        candidates = [o for o in detected_objects if o.class_name == head_noun]

        # Filter by color
        if color:
            candidates = [o for o in candidates
                         if o.attributes.get("color") == color]

        # Handle results
        if not candidates:
            return GroundedReference(None, 0.0, False)

        if len(candidates) == 1:
            return GroundedReference(candidates[0], candidates[0].confidence, False)

        # Multiple candidates - ambiguous
        best = max(candidates, key=lambda o: o.confidence)
        return GroundedReference(best, best.confidence, True)

    def _extract_noun(self, text: str) -> str:
        nouns = ["cup", "bottle", "box", "ball", "book"]
        for noun in nouns:
            if noun in text.lower():
                return noun
        return ""

    def _extract_color(self, text: str) -> Optional[str]:
        colors = ["red", "blue", "green", "yellow"]
        for color in colors:
            if color in text.lower():
                return color
        return None


# Usage
grounder = VisualGrounder()
scene = [
    DetectedObject("obj_001", "cup", (0.5, 0.3, 0.1), 0.95, {"color": "red"}),
    DetectedObject("obj_002", "cup", (0.7, 0.3, 0.1), 0.92, {"color": "blue"}),
]

result = grounder.ground("the red cup", scene)
if result.object:
    print(f"Grounded to: {result.object.object_id} at {result.object.position}")
```

**Key Points**:
- Parsing extracts head noun and attributes from expression
- Filtering progressively narrows candidates
- Ambiguity is detected and flagged for handling
- Confidence scores enable threshold-based decisions

For the complete implementation with spatial relations and context handling, see the [Multimodal Fusion Node](../code-examples/module-4/multimodal_fusion_node.py) code example.

---

## Summary

This chapter explored how multimodal fusion enables robots to connect language and vision:

1. **Fusion Architectures**: Early fusion enables rich cross-modal learning, late fusion provides robustness and modularity, while hybrid approaches using cross-attention offer the best balance for robotics applications.

2. **Visual Grounding**: The process of resolving referring expressions proceeds through candidate generation, attribute filtering, spatial verification, and confidence scoring to map language to specific scene objects.

3. **Cross-Attention Mechanism**: Transformer attention allows language tokens to attend to relevant visual regions, learning semantic-visual alignment without explicit supervision.

4. **Context-Aware Disambiguation**: Scene graphs, dialogue history, task context, and robot state all contribute to resolving ambiguous references without requiring explicit clarification.

5. **Temporal Dynamics**: Real environments change during execution, requiring object tracking, temporal fusion of observations, and explicit state management for grasped/placed objects.

6. **Graceful Degradation**: When grounding fails or is ambiguous, systems should request clarification with specific options rather than guessing or failing silently.

**Next Steps**: Chapter 5 brings together all VLA components in a capstone system, adding safety constraints and evaluation metrics for deployment-ready autonomous humanoid operation.

---

## Self-Assessment

Test your understanding of multimodal fusion:

1. **Compare early and late fusion architectures. When would you choose each?**
   <details>
   <summary>Show Answer</summary>
   Early fusion concatenates features before joint processing, enabling rich cross-modal interactions but requiring aligned training data and all modalities present. Late fusion processes modalities independently, offering robustness to missing inputs and modularity. Choose early fusion when you have large aligned datasets and need tight cross-modal reasoning. Choose late fusion when modalities may be missing (sensor failure) or when using pre-trained unimodal models. Hybrid (recommended) uses cross-attention for vision-language and late fusion for auxiliary modalities.
   </details>

2. **Explain how cross-attention enables vision-language alignment.**
   <details>
   <summary>Show Answer</summary>
   Cross-attention uses language tokens as queries and vision patches as keys/values. Attention weights indicate which visual regions are relevant to each word. For "red cup," the "red" token attends strongly to patches with red color, "cup" attends to cup-shaped regions. The combined attention localizes the specific red cup. This learns alignment from data without explicit supervision, handling novel combinations of attributes and objects.
   </details>

3. **Describe the stages of the visual grounding pipeline.**
   <details>
   <summary>Show Answer</summary>
   Stages: (1) Candidate Generation - find objects matching the head noun class (e.g., all cups); (2) Attribute Filtering - filter by color, size, and other mentioned attributes; (3) Spatial Verification - verify spatial relations with anchor objects (e.g., "on the table"); (4) Confidence Scoring - compute combined confidence from detection and matching scores; (5) Selection - return unique match, request clarification for ambiguity, or report failure.
   </details>

4. **How do scene graphs help with referring expression understanding?**
   <details>
   <summary>Show Answer</summary>
   Scene graphs represent objects and their relationships (SUPPORTS, NEXT_TO, ON, etc.). They enable: relation-based queries ("cup on table" finds cups with ON relation to tables), spatial reasoning ("cup next to bottle" uses proximity relations), consistency checking (cup can't be on two surfaces), and efficient search (traverse relations instead of checking all pairs). They also cache relationship computation for multiple queries.
   </details>

5. **What context sources can help resolve ambiguous references without asking the user?**
   <details>
   <summary>Show Answer</summary>
   Context sources: (1) Dialogue history - recent mentions ("the red cup" then "it" = red cup); (2) Task context - in "set the table," "the plate" = unused plate; (3) Robot state - if holding something, "put it down" = held object; (4) Scene statistics - if only one object of a type, "the cup" = that cup; (5) User attention - gaze direction or pointing gesture; (6) Spatial proximity - "this" = closest matching object to robot.
   </details>

6. **Why is temporal tracking important for multimodal fusion in robotics?**
   <details>
   <summary>Show Answer</summary>
   Environments change during task execution: objects are moved, people enter/exit, lighting changes, occlusions appear. Static grounding fails because the object detected at command time may have moved. Temporal tracking maintains object identity across frames, enables motion prediction for moving targets, allows re-identification after occlusion, and provides confidence through consistent observations. Without it, "pick up the cup" might target the wrong position.
   </details>

7. **How should a robot handle the command "pick up the cup" when multiple cups are visible?**
   <details>
   <summary>Show Answer</summary>
   Steps: (1) Check for distinguishing context - recent mention, robot proximity, task relevance; (2) If context resolves it, proceed with that cup; (3) If ambiguous, request clarification with specific options: "I see 3 cups. Do you mean the red one, the blue one, or the white one on the shelf?"; (4) Offer pointing/gesture input if available; (5) Allow attribute specification: "Which color?" or "The one where?"; (6) Never guess for manipulation actions - wrong pick could damage objects or be unsafe.
   </details>

8. **Explain how object state tracking handles a grasped object's position.**
   <details>
   <summary>Show Answer</summary>
   When an object is grasped, its state changes to GRASPED with attached_to = gripper. Its position is no longer from detection but computed from gripper position plus grasp offset. When released, state changes to PLACED with the new position recorded. This handles: objects not visible in camera (in gripper), objects that moved from original detection, and consistent reference throughout manipulation. Detection updates position only for non-grasped objects.
   </details>
=======
---
sidebar_position: 4
title: Chapter 4 - Multimodal Fusion
---

# Chapter 4: Multimodal Fusion

## Introduction

When you say "pick up the red cup on the table," your robot must do something remarkable: it must connect abstract language to concrete visual reality. The word "cup" must become a specific object with a specific position that the robot can actually grasp. This chapter explores multimodal fusion—the techniques that enable robots to bridge language and vision into unified understanding.

Multimodal fusion is the cornerstone of practical VLA systems. Without it, language understanding remains theoretical—the robot might know you want "a cup" but not which physical object in the scene matches that description. This visual grounding transforms intent into action by providing the precise coordinates, orientation, and identity needed for robot manipulation.

Building on the planning capabilities from Chapter 3, we now explore how vision and language combine to create context-aware robot behavior. This chapter covers fusion architectures, visual grounding algorithms, and how temporal dynamics affect multimodal understanding in real robot deployments.

### Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapters 1-3 of this module
- Understanding of transformer attention mechanisms
- Familiarity with object detection concepts from Module 3
- Basic linear algebra knowledge (vectors, matrices)

### Learning Objectives

By the end of this chapter, you will be able to:
1. Compare early, late, and hybrid fusion architectures for vision-language integration
2. Implement visual grounding to resolve natural language references to scene objects
3. Design context-aware systems that leverage scene understanding for disambiguation
4. Handle temporal dynamics in multimodal perception for moving objects and changing scenes

---

## Core Concept 1: Vision-Language-Motion Fusion

### The Multimodal Challenge

Robots receive information through multiple channels simultaneously:
- **Vision**: RGB images, depth maps, point clouds
- **Language**: Voice commands, text instructions
- **Audio**: Environmental sounds, speech
- **Proprioception**: Joint positions, forces, velocities

The challenge is combining these disparate signals into coherent understanding. Each modality has different:
- **Dimensionality**: Images are high-dimensional, text is sequential
- **Sampling rates**: Vision at 30Hz, audio at 16kHz, language as events
- **Semantics**: Images show appearance, language conveys intent

### Fusion Architecture Options

Three main approaches exist for combining modalities:

**Early Fusion** concatenates raw or lightly-processed features:
```text
Vision features ─┐
                 ├──► Concatenate ──► Joint Model ──► Output
Language features┘
```

Advantages:
- Enables rich cross-modal interactions
- Can learn implicit alignments
- Single model to optimize

Disadvantages:
- Requires large aligned training data
- Computationally expensive
- All modalities must be present

**Late Fusion** processes each modality independently, combining only at decision time:
```text
Vision ──► Vision Model ──► Vision Decision ─┐
                                             ├──► Aggregate
Language ──► Language Model ──► Lang Decision┘
```

Advantages:
- Modular and interpretable
- Robust to missing modalities
- Uses pre-trained unimodal models

Disadvantages:
- Limited cross-modal reasoning
- May miss complementary information
- Decision aggregation is simplistic

**Hybrid Fusion** (recommended for robotics) combines both:
```text
Vision ────► Cross-Attention ◄──── Language
                   │
            Fused Features
                   │
            ┌──────┴──────┐
            ▼             ▼
        Action Head   Grounding Head
```

See the [Multimodal Fusion Architecture](../diagrams/module-4/multimodal-fusion-architecture.md) for detailed visualization.

### Cross-Attention for Vision-Language

Cross-attention is the key mechanism for vision-language alignment:

```python
# Conceptual cross-attention
def cross_attention(query, key, value):
    """
    Query: Language tokens [batch, seq_len, dim]
    Key/Value: Vision patches [batch, num_patches, dim]
    """
    # Compute attention weights
    scores = query @ key.transpose(-2, -1) / sqrt(dim)
    weights = softmax(scores, dim=-1)

    # Attend to visual features
    output = weights @ value
    return output, weights
```

When processing "the red cup":
1. "red" token attends strongly to patches with red color
2. "cup" token attends to cup-shaped regions
3. Combined attention localizes the red cup

---

## Core Concept 2: Visual Grounding and Referring Expressions

### The Grounding Problem

Visual grounding resolves natural language references to specific scene objects:

| Expression | Challenge |
|------------|-----------|
| "the cup" | Multiple cups may exist |
| "the red one" | Requires attribute matching |
| "the cup on the left" | Requires spatial reasoning |
| "that one" | Requires dialogue context |

### Referring Expression Structure

Referring expressions have compositional structure:

```text
"the large red cup on the table"
 │    │     │    │   │    │
 │    │     │    │   │    └── Anchor noun
 │    │     │    │   └── Spatial preposition
 │    │     │    └── Head noun (target class)
 │    │     └── Color attribute
 │    └── Size attribute
 └── Determiner
```

Parsing this structure enables systematic grounding:
1. Find objects of class "cup"
2. Filter by color = red
3. Filter by size = large (relative to other cups)
4. Verify spatial relation: ON(target, table)

### The Grounding Pipeline

Visual grounding proceeds through stages:

```text
1. CANDIDATE GENERATION
   └── Find all objects matching head noun class

2. ATTRIBUTE FILTERING
   ├── Color: detected_color == "red"
   └── Size: volume > median(cups)

3. SPATIAL VERIFICATION
   └── Check ON(candidate, anchor)

4. CONFIDENCE SCORING
   └── Combined score = Π(individual_confidences)

5. SELECTION
   ├── If unique high-confidence match → return
   └── If ambiguous → request clarification
```

### Handling Ambiguity

When multiple objects match, the system must handle ambiguity gracefully:

```python
def handle_ambiguity(candidates: list, confidence_threshold: float):
    """Handle ambiguous grounding results."""
    if len(candidates) == 0:
        return ask_clarification("I don't see any matching objects.")

    if len(candidates) == 1 and candidates[0].confidence > confidence_threshold:
        return candidates[0]  # Unique confident match

    # Multiple candidates - need disambiguation
    if candidates[0].confidence - candidates[1].confidence < 0.1:
        # Close scores - ask user
        options = describe_differences(candidates[:3])
        return ask_clarification(f"Which one? {options}")

    # Clear winner despite multiple candidates
    return candidates[0]
```

See the [Visual Grounding Flow](../diagrams/module-4/visual-grounding-flow.md) for the complete grounding process.

---

## Core Concept 3: Context-Aware Decision Making

### Scene Context

Beyond individual object detection, scene context provides crucial information:

**Spatial Layout**: Object positions relative to each other
- "The cup next to the bottle" requires understanding proximity
- "The rightmost cup" requires ordering objects spatially

**Functional Context**: What objects afford
- A cup can be grasped, filled, placed
- A table supports objects, defines workspace

**Historical Context**: What was mentioned before
- "Pick up the cup. Now put it down." - "it" refers to the cup
- Recent mentions have higher salience

### Scene Graph Representation

Scene graphs capture object relationships:

```text
Scene Graph for "table with red cup and blue bottle":

        ┌─────────┐
        │  Table  │
        └────┬────┘
             │ SUPPORTS
     ┌───────┴───────┐
     │               │
┌────▼────┐    ┌────▼────┐
│ Red Cup │    │Blue Btl │
└────┬────┘    └─────────┘
     │ NEXT_TO
     └──────────────────►
```

Scene graphs enable:
- Relation-based queries ("cup on table")
- Spatial reasoning ("cup next to bottle")
- Consistency checking ("cup can't be on floor AND table")

### Contextual Disambiguation

Context resolves ambiguity without explicit clarification:

**Recent Mention**: If user just said "the red cup," then "it" or "that" refers to the red cup.

**Task Context**: In a "set the table" task, "the plate" likely means an unused plate, not one already placed.

**Robot State**: If robot is holding an object, "put it down" refers to that object.

**User Gaze/Pointing**: Gesture input can disambiguate "that one" (though requires additional sensing).

```python
class ContextManager:
    """Manages dialogue and scene context for disambiguation."""

    def __init__(self):
        self.recent_objects = []  # Stack of mentioned objects
        self.current_task = None
        self.robot_holding = None

    def resolve_pronoun(self, pronoun: str) -> Optional[Object]:
        """Resolve pronouns using context."""
        if pronoun in ["it", "that", "this"]:
            if self.recent_objects:
                return self.recent_objects[-1]
            if self.robot_holding:
                return self.robot_holding
        return None

    def update_context(self, grounded_object: Object):
        """Update context with newly grounded object."""
        self.recent_objects.append(grounded_object)
        # Keep only recent history
        self.recent_objects = self.recent_objects[-5:]
```

---

## Core Concept 4: Temporal Fusion for Dynamic Environments

### The Dynamic World Problem

Real environments change during task execution:
- Objects move (pushed, fall, picked up)
- People enter and leave
- Lighting changes
- Occlusions appear and disappear

Static grounding is insufficient—systems must track objects through time.

### Temporal Object Tracking

Multi-object tracking maintains identity over time:

```text
Frame t:    [cup_01 @ (0.5, 0.3)]  [bottle_01 @ (0.6, 0.3)]
                     │                      │
                     │ Association          │ Association
                     │                      │
Frame t+1:  [cup_01 @ (0.5, 0.31)] [bottle_01 @ (0.6, 0.3)]
```

Tracking enables:
- Consistent object references across frames
- Motion prediction for moving targets
- Re-identification after occlusion

### Fusion Across Time

Temporal fusion combines observations over time:

```python
class TemporalFusion:
    """Fuse observations across time for robust grounding."""

    def __init__(self, history_length: int = 10):
        self.observations = {}  # object_id -> list of observations
        self.history_length = history_length

    def update(self, object_id: str, observation: dict):
        """Add new observation for object."""
        if object_id not in self.observations:
            self.observations[object_id] = []

        self.observations[object_id].append(observation)

        # Keep bounded history
        if len(self.observations[object_id]) > self.history_length:
            self.observations[object_id].pop(0)

    def get_fused_estimate(self, object_id: str) -> dict:
        """Get temporally smoothed estimate."""
        history = self.observations.get(object_id, [])

        if not history:
            return None

        # Simple averaging (Kalman filter in production)
        avg_position = [
            sum(obs['position'][i] for obs in history) / len(history)
            for i in range(3)
        ]

        # Confidence increases with consistent observations
        confidence = min(len(history) / self.history_length, 1.0)

        return {
            'position': avg_position,
            'confidence': confidence * history[-1]['detection_confidence']
        }
```

### Handling Object State Changes

Objects change state during manipulation:
- **Grasped**: Attached to gripper, follows arm
- **Placed**: New position on surface
- **Moved by human**: Position update from detection

```python
class ObjectStateTracker:
    """Track object states through manipulation."""

    def __init__(self):
        self.object_states = {}  # object_id -> state

    def grasp_object(self, object_id: str):
        """Mark object as grasped by robot."""
        self.object_states[object_id] = {
            'state': 'GRASPED',
            'attached_to': 'gripper',
            'last_known_position': None  # Now follows gripper
        }

    def release_object(self, object_id: str, position: tuple):
        """Mark object as released at position."""
        self.object_states[object_id] = {
            'state': 'PLACED',
            'attached_to': None,
            'last_known_position': position
        }

    def get_object_position(self, object_id: str, gripper_pos: tuple):
        """Get current object position considering state."""
        state = self.object_states.get(object_id, {})

        if state.get('state') == 'GRASPED':
            return gripper_pos  # Object follows gripper

        return state.get('last_known_position')
```

---

## Hands-On Example

### Example: Multimodal Fusion Node

The following code demonstrates visual grounding for robot commands:

```python
#!/usr/bin/env python3
"""
Multimodal Fusion - Visual Grounding for Robot Commands
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class DetectedObject:
    """An object detected in the visual scene."""
    object_id: str
    class_name: str
    position: tuple[float, float, float]
    confidence: float
    attributes: dict


@dataclass
class GroundedReference:
    """Result of grounding a referring expression."""
    object: Optional[DetectedObject]
    confidence: float
    is_ambiguous: bool


class VisualGrounder:
    """Grounds referring expressions to scene objects."""

    def ground(
        self,
        expression: str,
        detected_objects: list[DetectedObject]
    ) -> GroundedReference:
        """Ground a referring expression to a specific object."""
        # Parse expression (simplified)
        head_noun = self._extract_noun(expression)
        color = self._extract_color(expression)

        # Filter by class
        candidates = [o for o in detected_objects if o.class_name == head_noun]

        # Filter by color
        if color:
            candidates = [o for o in candidates
                         if o.attributes.get("color") == color]

        # Handle results
        if not candidates:
            return GroundedReference(None, 0.0, False)

        if len(candidates) == 1:
            return GroundedReference(candidates[0], candidates[0].confidence, False)

        # Multiple candidates - ambiguous
        best = max(candidates, key=lambda o: o.confidence)
        return GroundedReference(best, best.confidence, True)

    def _extract_noun(self, text: str) -> str:
        nouns = ["cup", "bottle", "box", "ball", "book"]
        for noun in nouns:
            if noun in text.lower():
                return noun
        return ""

    def _extract_color(self, text: str) -> Optional[str]:
        colors = ["red", "blue", "green", "yellow"]
        for color in colors:
            if color in text.lower():
                return color
        return None


# Usage
grounder = VisualGrounder()
scene = [
    DetectedObject("obj_001", "cup", (0.5, 0.3, 0.1), 0.95, {"color": "red"}),
    DetectedObject("obj_002", "cup", (0.7, 0.3, 0.1), 0.92, {"color": "blue"}),
]

result = grounder.ground("the red cup", scene)
if result.object:
    print(f"Grounded to: {result.object.object_id} at {result.object.position}")
```

**Key Points**:
- Parsing extracts head noun and attributes from expression
- Filtering progressively narrows candidates
- Ambiguity is detected and flagged for handling
- Confidence scores enable threshold-based decisions

For the complete implementation with spatial relations and context handling, see the [Multimodal Fusion Node](../code-examples/module-4/multimodal_fusion_node.py) code example.

---

## Summary

This chapter explored how multimodal fusion enables robots to connect language and vision:

1. **Fusion Architectures**: Early fusion enables rich cross-modal learning, late fusion provides robustness and modularity, while hybrid approaches using cross-attention offer the best balance for robotics applications.

2. **Visual Grounding**: The process of resolving referring expressions proceeds through candidate generation, attribute filtering, spatial verification, and confidence scoring to map language to specific scene objects.

3. **Cross-Attention Mechanism**: Transformer attention allows language tokens to attend to relevant visual regions, learning semantic-visual alignment without explicit supervision.

4. **Context-Aware Disambiguation**: Scene graphs, dialogue history, task context, and robot state all contribute to resolving ambiguous references without requiring explicit clarification.

5. **Temporal Dynamics**: Real environments change during execution, requiring object tracking, temporal fusion of observations, and explicit state management for grasped/placed objects.

6. **Graceful Degradation**: When grounding fails or is ambiguous, systems should request clarification with specific options rather than guessing or failing silently.

**Next Steps**: Chapter 5 brings together all VLA components in a capstone system, adding safety constraints and evaluation metrics for deployment-ready autonomous humanoid operation.

---

## Self-Assessment

Test your understanding of multimodal fusion:

1. **Compare early and late fusion architectures. When would you choose each?**
   <details>
   <summary>Show Answer</summary>
   Early fusion concatenates features before joint processing, enabling rich cross-modal interactions but requiring aligned training data and all modalities present. Late fusion processes modalities independently, offering robustness to missing inputs and modularity. Choose early fusion when you have large aligned datasets and need tight cross-modal reasoning. Choose late fusion when modalities may be missing (sensor failure) or when using pre-trained unimodal models. Hybrid (recommended) uses cross-attention for vision-language and late fusion for auxiliary modalities.
   </details>

2. **Explain how cross-attention enables vision-language alignment.**
   <details>
   <summary>Show Answer</summary>
   Cross-attention uses language tokens as queries and vision patches as keys/values. Attention weights indicate which visual regions are relevant to each word. For "red cup," the "red" token attends strongly to patches with red color, "cup" attends to cup-shaped regions. The combined attention localizes the specific red cup. This learns alignment from data without explicit supervision, handling novel combinations of attributes and objects.
   </details>

3. **Describe the stages of the visual grounding pipeline.**
   <details>
   <summary>Show Answer</summary>
   Stages: (1) Candidate Generation - find objects matching the head noun class (e.g., all cups); (2) Attribute Filtering - filter by color, size, and other mentioned attributes; (3) Spatial Verification - verify spatial relations with anchor objects (e.g., "on the table"); (4) Confidence Scoring - compute combined confidence from detection and matching scores; (5) Selection - return unique match, request clarification for ambiguity, or report failure.
   </details>

4. **How do scene graphs help with referring expression understanding?**
   <details>
   <summary>Show Answer</summary>
   Scene graphs represent objects and their relationships (SUPPORTS, NEXT_TO, ON, etc.). They enable: relation-based queries ("cup on table" finds cups with ON relation to tables), spatial reasoning ("cup next to bottle" uses proximity relations), consistency checking (cup can't be on two surfaces), and efficient search (traverse relations instead of checking all pairs). They also cache relationship computation for multiple queries.
   </details>

5. **What context sources can help resolve ambiguous references without asking the user?**
   <details>
   <summary>Show Answer</summary>
   Context sources: (1) Dialogue history - recent mentions ("the red cup" then "it" = red cup); (2) Task context - in "set the table," "the plate" = unused plate; (3) Robot state - if holding something, "put it down" = held object; (4) Scene statistics - if only one object of a type, "the cup" = that cup; (5) User attention - gaze direction or pointing gesture; (6) Spatial proximity - "this" = closest matching object to robot.
   </details>

6. **Why is temporal tracking important for multimodal fusion in robotics?**
   <details>
   <summary>Show Answer</summary>
   Environments change during task execution: objects are moved, people enter/exit, lighting changes, occlusions appear. Static grounding fails because the object detected at command time may have moved. Temporal tracking maintains object identity across frames, enables motion prediction for moving targets, allows re-identification after occlusion, and provides confidence through consistent observations. Without it, "pick up the cup" might target the wrong position.
   </details>

7. **How should a robot handle the command "pick up the cup" when multiple cups are visible?**
   <details>
   <summary>Show Answer</summary>
   Steps: (1) Check for distinguishing context - recent mention, robot proximity, task relevance; (2) If context resolves it, proceed with that cup; (3) If ambiguous, request clarification with specific options: "I see 3 cups. Do you mean the red one, the blue one, or the white one on the shelf?"; (4) Offer pointing/gesture input if available; (5) Allow attribute specification: "Which color?" or "The one where?"; (6) Never guess for manipulation actions - wrong pick could damage objects or be unsafe.
   </details>

8. **Explain how object state tracking handles a grasped object's position.**
   <details>
   <summary>Show Answer</summary>
   When an object is grasped, its state changes to GRASPED with attached_to = gripper. Its position is no longer from detection but computed from gripper position plus grasp offset. When released, state changes to PLACED with the new position recorded. This handles: objects not visible in camera (in gripper), objects that moved from original detection, and consistent reference throughout manipulation. Detection updates position only for non-grasped objects.
   </details>
>>>>>>> 5cf5f6818272e652910633754d301908ff0bb236
