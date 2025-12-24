#!/usr/bin/env python3
"""
Multimodal Fusion Node - Vision-Language Integration for Robot Commands
Module 4, Chapter 4: Multimodal Fusion

This module demonstrates multimodal fusion techniques for grounding
language commands in visual perception. It shows how text references
are resolved to specific objects in the scene.

Note: This is an educational/architectural example. Production systems
would use trained vision-language models (CLIP, OWL-ViT, etc.).
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional
import math


# =============================================================================
# Data Structures
# =============================================================================

@dataclass
class BoundingBox3D:
    """3D bounding box for an object."""
    center: tuple[float, float, float]
    dimensions: tuple[float, float, float]  # width, depth, height

    @property
    def volume(self) -> float:
        return self.dimensions[0] * self.dimensions[1] * self.dimensions[2]

    def contains_point(self, point: tuple[float, float, float]) -> bool:
        """Check if a point is inside the bounding box."""
        for i in range(3):
            half_dim = self.dimensions[i] / 2
            if not (self.center[i] - half_dim <= point[i] <= self.center[i] + half_dim):
                return False
        return True


@dataclass
class DetectedObject:
    """An object detected in the visual scene."""
    object_id: str
    class_name: str
    position: tuple[float, float, float]
    bounding_box: BoundingBox3D
    confidence: float
    attributes: dict = field(default_factory=dict)

    def distance_to(self, other_pos: tuple[float, float, float]) -> float:
        """Calculate Euclidean distance to another position."""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(self.position, other_pos)))


@dataclass
class ReferringExpression:
    """Parsed structure of a referring expression."""
    head_noun: str  # Target object category
    color: Optional[str] = None
    size: Optional[str] = None  # "large", "small", etc.
    spatial_relation: Optional[str] = None  # "on", "next_to", "inside"
    anchor_noun: Optional[str] = None  # Reference object for spatial relation
    original_text: str = ""


@dataclass
class GroundedReference:
    """Result of grounding a referring expression to an object."""
    expression: ReferringExpression
    object: Optional[DetectedObject]
    confidence: float
    anchor_object: Optional[DetectedObject] = None
    is_ambiguous: bool = False
    alternative_objects: list[DetectedObject] = field(default_factory=list)


class SpatialRelation(Enum):
    """Supported spatial relations."""
    ON = auto()          # Object above and within footprint
    NEXT_TO = auto()     # Within proximity threshold
    INSIDE = auto()      # Within container bounds
    BEHIND = auto()      # Relative to viewpoint
    IN_FRONT_OF = auto()
    LEFT_OF = auto()
    RIGHT_OF = auto()
    ABOVE = auto()
    BELOW = auto()


# =============================================================================
# Referring Expression Parser
# =============================================================================

class ReferringExpressionParser:
    """
    Parses natural language referring expressions into structured form.

    In production, this would use a trained parser or LLM.
    This implementation uses pattern matching for demonstration.
    """

    # Color vocabulary
    COLORS = {
        "red", "blue", "green", "yellow", "orange", "purple",
        "pink", "black", "white", "gray", "grey", "brown"
    }

    # Size vocabulary
    SIZES = {"large", "big", "small", "tiny", "huge", "tall", "short"}

    # Spatial prepositions
    SPATIAL_PREPS = {
        "on": SpatialRelation.ON,
        "next to": SpatialRelation.NEXT_TO,
        "beside": SpatialRelation.NEXT_TO,
        "near": SpatialRelation.NEXT_TO,
        "inside": SpatialRelation.INSIDE,
        "in": SpatialRelation.INSIDE,
        "behind": SpatialRelation.BEHIND,
        "in front of": SpatialRelation.IN_FRONT_OF,
        "left of": SpatialRelation.LEFT_OF,
        "right of": SpatialRelation.RIGHT_OF,
        "above": SpatialRelation.ABOVE,
        "below": SpatialRelation.BELOW,
    }

    # Common object nouns
    OBJECT_NOUNS = {
        "cup", "mug", "glass", "bottle", "can", "box", "ball",
        "book", "plate", "bowl", "table", "shelf", "chair"
    }

    def parse(self, text: str) -> ReferringExpression:
        """
        Parse a referring expression into structured form.

        Args:
            text: Natural language referring expression

        Returns:
            Parsed ReferringExpression structure
        """
        text_lower = text.lower()
        words = text_lower.split()

        result = ReferringExpression(
            head_noun="",
            original_text=text
        )

        # Extract color
        for word in words:
            if word in self.COLORS:
                result.color = word
                break

        # Extract size
        for word in words:
            if word in self.SIZES:
                result.size = word
                break

        # Extract head noun (target object)
        for word in words:
            if word in self.OBJECT_NOUNS:
                result.head_noun = word
                break

        # Extract spatial relation and anchor
        for prep, relation in self.SPATIAL_PREPS.items():
            if prep in text_lower:
                result.spatial_relation = prep
                # Find anchor noun after preposition
                prep_idx = text_lower.find(prep)
                after_prep = text_lower[prep_idx + len(prep):]
                for word in after_prep.split():
                    word_clean = word.strip("the ")
                    if word_clean in self.OBJECT_NOUNS:
                        result.anchor_noun = word_clean
                        break
                break

        return result


# =============================================================================
# Visual Grounder
# =============================================================================

class VisualGrounder:
    """
    Grounds referring expressions to objects in the visual scene.

    Implements the multi-stage grounding pipeline:
    1. Candidate generation (filter by class)
    2. Attribute filtering (color, size)
    3. Spatial relation verification
    4. Confidence scoring
    """

    def __init__(self, confidence_threshold: float = 0.5):
        self.confidence_threshold = confidence_threshold
        self.parser = ReferringExpressionParser()

    def ground(
        self,
        expression: str,
        detected_objects: list[DetectedObject]
    ) -> GroundedReference:
        """
        Ground a referring expression to a specific object.

        Args:
            expression: Natural language referring expression
            detected_objects: List of objects detected in scene

        Returns:
            GroundedReference with matched object or ambiguity info
        """
        # Parse the expression
        parsed = self.parser.parse(expression)

        # Stage 1: Candidate generation (filter by class)
        candidates = self._filter_by_class(parsed.head_noun, detected_objects)

        if not candidates:
            return GroundedReference(
                expression=parsed,
                object=None,
                confidence=0.0
            )

        # Stage 2: Filter by color
        if parsed.color:
            candidates = self._filter_by_color(parsed.color, candidates)

        # Stage 3: Filter by size (relative comparison)
        if parsed.size and len(candidates) > 1:
            candidates = self._filter_by_size(parsed.size, candidates)

        # Stage 4: Filter by spatial relation
        anchor_object = None
        if parsed.spatial_relation and parsed.anchor_noun:
            candidates, anchor_object = self._filter_by_spatial(
                parsed.spatial_relation,
                parsed.anchor_noun,
                candidates,
                detected_objects
            )

        # Stage 5: Score and select
        return self._score_and_select(parsed, candidates, anchor_object)

    def _filter_by_class(
        self,
        class_name: str,
        objects: list[DetectedObject]
    ) -> list[DetectedObject]:
        """Filter objects by class name."""
        if not class_name:
            return objects
        return [obj for obj in objects if obj.class_name == class_name]

    def _filter_by_color(
        self,
        color: str,
        objects: list[DetectedObject]
    ) -> list[DetectedObject]:
        """Filter objects by color attribute."""
        filtered = []
        for obj in objects:
            obj_color = obj.attributes.get("color", "").lower()
            if obj_color == color:
                filtered.append(obj)
        return filtered if filtered else objects  # Return original if no match

    def _filter_by_size(
        self,
        size: str,
        objects: list[DetectedObject]
    ) -> list[DetectedObject]:
        """
        Filter objects by relative size.

        'large' = above median volume
        'small' = below median volume
        """
        if len(objects) <= 1:
            return objects

        # Calculate volumes and median
        volumes = [(obj, obj.bounding_box.volume) for obj in objects]
        volumes.sort(key=lambda x: x[1])
        median_idx = len(volumes) // 2
        median_volume = volumes[median_idx][1]

        # Filter based on size descriptor
        if size in ["large", "big", "huge", "tall"]:
            return [obj for obj, vol in volumes if vol >= median_volume]
        elif size in ["small", "tiny", "short"]:
            return [obj for obj, vol in volumes if vol <= median_volume]

        return objects

    def _filter_by_spatial(
        self,
        relation: str,
        anchor_noun: str,
        candidates: list[DetectedObject],
        all_objects: list[DetectedObject]
    ) -> tuple[list[DetectedObject], Optional[DetectedObject]]:
        """Filter candidates by spatial relation to anchor object."""
        # Find anchor object
        anchors = [obj for obj in all_objects if obj.class_name == anchor_noun]
        if not anchors:
            return candidates, None

        anchor = anchors[0]  # Use first match as anchor
        filtered = []

        for obj in candidates:
            if self._check_spatial_relation(relation, obj, anchor):
                filtered.append(obj)

        return (filtered if filtered else candidates), anchor

    def _check_spatial_relation(
        self,
        relation: str,
        target: DetectedObject,
        anchor: DetectedObject
    ) -> bool:
        """Check if target satisfies spatial relation with anchor."""
        target_pos = target.position
        anchor_pos = anchor.position
        anchor_box = anchor.bounding_box

        if relation == "on":
            # Target should be above anchor and within XY footprint
            above_surface = target_pos[2] > anchor_pos[2]
            within_height = target_pos[2] < anchor_pos[2] + 0.5  # Not too high

            # Check XY overlap with anchor footprint
            half_w = anchor_box.dimensions[0] / 2
            half_d = anchor_box.dimensions[1] / 2
            within_x = anchor_pos[0] - half_w <= target_pos[0] <= anchor_pos[0] + half_w
            within_y = anchor_pos[1] - half_d <= target_pos[1] <= anchor_pos[1] + half_d

            return above_surface and within_height and within_x and within_y

        elif relation in ["next to", "beside", "near"]:
            # Within proximity threshold
            distance = target.distance_to(anchor_pos)
            return distance < 0.5  # 50cm threshold

        elif relation in ["inside", "in"]:
            # Target center within anchor bounds
            return anchor_box.contains_point(target_pos)

        elif relation == "left of":
            # Target X less than anchor X (from robot's perspective)
            return target_pos[0] < anchor_pos[0]

        elif relation == "right of":
            return target_pos[0] > anchor_pos[0]

        elif relation == "behind":
            # Target Y greater than anchor Y (robot faces -Y)
            return target_pos[1] > anchor_pos[1]

        elif relation == "in front of":
            return target_pos[1] < anchor_pos[1]

        elif relation == "above":
            return target_pos[2] > anchor_pos[2] + anchor_box.dimensions[2]

        elif relation == "below":
            return target_pos[2] < anchor_pos[2]

        return True  # Unknown relation, don't filter

    def _score_and_select(
        self,
        parsed: ReferringExpression,
        candidates: list[DetectedObject],
        anchor: Optional[DetectedObject]
    ) -> GroundedReference:
        """Score candidates and select best match."""
        if not candidates:
            return GroundedReference(
                expression=parsed,
                object=None,
                confidence=0.0
            )

        # Score each candidate
        scored = []
        for obj in candidates:
            score = self._compute_confidence(parsed, obj)
            scored.append((obj, score))

        # Sort by score descending
        scored.sort(key=lambda x: x[1], reverse=True)
        best_obj, best_score = scored[0]

        # Check for ambiguity
        is_ambiguous = False
        alternatives = []
        if len(scored) > 1:
            second_score = scored[1][1]
            if best_score - second_score < 0.1:  # Close scores = ambiguous
                is_ambiguous = True
                alternatives = [obj for obj, _ in scored[1:]]

        return GroundedReference(
            expression=parsed,
            object=best_obj if best_score >= self.confidence_threshold else None,
            confidence=best_score,
            anchor_object=anchor,
            is_ambiguous=is_ambiguous,
            alternative_objects=alternatives
        )

    def _compute_confidence(
        self,
        parsed: ReferringExpression,
        obj: DetectedObject
    ) -> float:
        """Compute confidence score for a candidate match."""
        score = obj.confidence  # Start with detection confidence

        # Attribute match bonuses
        if parsed.color:
            if obj.attributes.get("color") == parsed.color:
                score *= 0.95  # Color match
            else:
                score *= 0.5  # Color mismatch penalty

        if parsed.size:
            if obj.attributes.get("size") == parsed.size:
                score *= 0.9  # Size match
            else:
                score *= 0.7  # Size uncertain

        return score


# =============================================================================
# Multimodal Fusion Node
# =============================================================================

class MultimodalFusionNode:
    """
    ROS 2-style node for multimodal fusion.

    Combines language input with visual perception to ground
    referring expressions to specific scene objects.
    """

    def __init__(self):
        self.grounder = VisualGrounder(confidence_threshold=0.5)
        self.scene_objects: list[DetectedObject] = []
        self.dialogue_history: list[str] = []

    def update_scene(self, objects: list[DetectedObject]) -> None:
        """Update the current scene with detected objects."""
        self.scene_objects = objects

    def process_command(
        self,
        command: str,
        target_expression: str
    ) -> dict:
        """
        Process a command with a referring expression.

        Args:
            command: Full command text
            target_expression: The referring expression to ground

        Returns:
            Dictionary with grounding results
        """
        # Ground the expression
        result = self.grounder.ground(target_expression, self.scene_objects)

        # Update dialogue history
        self.dialogue_history.append(command)

        # Build response
        response = {
            "command": command,
            "expression": target_expression,
            "grounding": {
                "success": result.object is not None,
                "object_id": result.object.object_id if result.object else None,
                "position": result.object.position if result.object else None,
                "confidence": result.confidence,
                "is_ambiguous": result.is_ambiguous
            }
        }

        # Add disambiguation info if needed
        if result.is_ambiguous:
            response["disambiguation"] = {
                "message": "Multiple objects match. Please clarify.",
                "alternatives": [
                    {
                        "object_id": obj.object_id,
                        "attributes": obj.attributes
                    }
                    for obj in result.alternative_objects
                ]
            }

        return response


# =============================================================================
# Usage Example
# =============================================================================

def main():
    """Demonstrate multimodal fusion for visual grounding."""
    print("=" * 60)
    print("Multimodal Fusion Demo")
    print("=" * 60)

    # Create sample scene
    scene_objects = [
        DetectedObject(
            object_id="obj_001",
            class_name="cup",
            position=(0.5, 0.3, 0.15),
            bounding_box=BoundingBox3D(
                center=(0.5, 0.3, 0.15),
                dimensions=(0.08, 0.08, 0.12)
            ),
            confidence=0.95,
            attributes={"color": "red", "size": "large"}
        ),
        DetectedObject(
            object_id="obj_002",
            class_name="cup",
            position=(0.7, 0.3, 0.12),
            bounding_box=BoundingBox3D(
                center=(0.7, 0.3, 0.12),
                dimensions=(0.06, 0.06, 0.10)
            ),
            confidence=0.92,
            attributes={"color": "blue", "size": "small"}
        ),
        DetectedObject(
            object_id="obj_003",
            class_name="cup",
            position=(0.3, 0.5, 0.12),
            bounding_box=BoundingBox3D(
                center=(0.3, 0.5, 0.12),
                dimensions=(0.06, 0.06, 0.10)
            ),
            confidence=0.88,
            attributes={"color": "red", "size": "small"}
        ),
        DetectedObject(
            object_id="obj_004",
            class_name="table",
            position=(0.5, 0.4, 0.0),
            bounding_box=BoundingBox3D(
                center=(0.5, 0.4, 0.0),
                dimensions=(0.8, 0.6, 0.02)
            ),
            confidence=0.97,
            attributes={"material": "wood"}
        ),
        DetectedObject(
            object_id="obj_005",
            class_name="bottle",
            position=(0.6, 0.4, 0.20),
            bounding_box=BoundingBox3D(
                center=(0.6, 0.4, 0.20),
                dimensions=(0.07, 0.07, 0.25)
            ),
            confidence=0.91,
            attributes={"color": "green"}
        ),
    ]

    # Initialize fusion node
    fusion_node = MultimodalFusionNode()
    fusion_node.update_scene(scene_objects)

    # Test expressions
    test_cases = [
        ("Pick up the red cup", "the red cup"),
        ("Pick up the large red cup on the table", "the large red cup on the table"),
        ("Get the small blue cup", "the small blue cup"),
        ("Pick up the cup", "the cup"),  # Ambiguous
        ("Get the green bottle", "the green bottle"),
    ]

    for command, expression in test_cases:
        print(f"\n--- Command: \"{command}\" ---")
        print(f"    Expression: \"{expression}\"")

        result = fusion_node.process_command(command, expression)

        grounding = result["grounding"]
        if grounding["success"]:
            print(f"    ✓ Grounded to: {grounding['object_id']}")
            print(f"      Position: {grounding['position']}")
            print(f"      Confidence: {grounding['confidence']:.2f}")
        else:
            print(f"    ✗ Grounding failed (confidence: {grounding['confidence']:.2f})")

        if grounding["is_ambiguous"]:
            print(f"    ⚠ AMBIGUOUS - Alternatives:")
            for alt in result.get("disambiguation", {}).get("alternatives", []):
                print(f"        - {alt['object_id']}: {alt['attributes']}")

    print("\n" + "=" * 60)
    print("Demo complete")
    print("=" * 60)


if __name__ == "__main__":
    main()
