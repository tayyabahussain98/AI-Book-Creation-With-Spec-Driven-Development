#!/usr/bin/env python3
"""
Voice Command Parser - Intent Extraction for Robot Commands
Module 4, Chapter 2: Voice-to-Action

This module demonstrates the pipeline for extracting structured intents
from natural language voice commands. It shows preprocessing, intent
classification, entity extraction, and slot filling.

Note: This is an educational/architectural example. Production systems
would use trained ML models for classification and NER.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional
import re


# =============================================================================
# Intent and Entity Definitions
# =============================================================================

class IntentType(Enum):
    """
    Supported robot action intents.

    Each intent represents a high-level action the robot can perform.
    The intent determines which slots are required or optional.
    """
    PICK = auto()      # Grasp an object
    PLACE = auto()     # Put object at location
    MOVE = auto()      # Navigate to position
    FIND = auto()      # Locate object or person
    GIVE = auto()      # Hand object to person
    SPEAK = auto()     # Generate verbal response
    WAIT = auto()      # Pause execution
    STOP = auto()      # Emergency halt
    UNKNOWN = auto()   # Unrecognized intent


class EntityType(Enum):
    """
    Entity types extracted from commands.

    These represent the semantic roles that words can play
    in robot commands.
    """
    OBJECT = auto()     # Physical items (cup, bottle, box)
    LOCATION = auto()   # Places (table, shelf, kitchen)
    PERSON = auto()     # Human references (me, John)
    COLOR = auto()      # Visual attributes (red, blue)
    SIZE = auto()       # Dimensional attributes (large, small)
    QUANTITY = auto()   # Numerical amounts (one, three)
    DIRECTION = auto()  # Spatial directions (left, right)


@dataclass
class Entity:
    """An extracted entity with its metadata."""
    entity_type: EntityType
    value: str
    confidence: float = 1.0
    start_idx: int = 0
    end_idx: int = 0


@dataclass
class Intent:
    """
    Structured representation of a parsed command.

    Contains the intent type, extracted entities organized by role,
    and confidence scores for validation.
    """
    intent_type: IntentType
    confidence: float
    target_object: Optional[Entity] = None
    object_color: Optional[Entity] = None
    object_size: Optional[Entity] = None
    source_location: Optional[Entity] = None
    destination: Optional[Entity] = None
    target_person: Optional[Entity] = None
    quantity: int = 1
    original_text: str = ""
    requires_disambiguation: bool = False
    disambiguation_reason: Optional[str] = None


# =============================================================================
# Intent Classification
# =============================================================================

class IntentClassifier:
    """
    Rule-based intent classifier for demonstration.

    In production, this would be replaced with a trained model
    (e.g., fine-tuned BERT, DistilBERT, or domain-specific classifier).
    """

    # Intent trigger words with associated confidence weights
    INTENT_PATTERNS = {
        IntentType.PICK: {
            "triggers": ["pick up", "pick", "grab", "grasp", "take", "get"],
            "base_confidence": 0.85
        },
        IntentType.PLACE: {
            "triggers": ["place", "put", "set", "put down", "drop"],
            "base_confidence": 0.85
        },
        IntentType.MOVE: {
            "triggers": ["move to", "go to", "navigate", "walk to", "come to"],
            "base_confidence": 0.85
        },
        IntentType.FIND: {
            "triggers": ["find", "locate", "look for", "search", "where is"],
            "base_confidence": 0.80
        },
        IntentType.GIVE: {
            "triggers": ["give", "hand", "pass", "bring to", "deliver"],
            "base_confidence": 0.85
        },
        IntentType.SPEAK: {
            "triggers": ["say", "tell", "speak", "announce", "report"],
            "base_confidence": 0.80
        },
        IntentType.WAIT: {
            "triggers": ["wait", "pause", "hold", "stay"],
            "base_confidence": 0.90
        },
        IntentType.STOP: {
            "triggers": ["stop", "halt", "abort", "emergency", "freeze"],
            "base_confidence": 0.95  # High confidence for safety commands
        }
    }

    def classify(self, text: str) -> tuple[IntentType, float]:
        """
        Classify the intent of a text command.

        Args:
            text: Preprocessed command text

        Returns:
            Tuple of (intent_type, confidence)
        """
        text_lower = text.lower()

        best_intent = IntentType.UNKNOWN
        best_confidence = 0.0

        for intent_type, pattern_info in self.INTENT_PATTERNS.items():
            for trigger in pattern_info["triggers"]:
                if trigger in text_lower:
                    # Earlier match in sentence = higher confidence
                    position_bonus = 0.1 if text_lower.startswith(trigger) else 0.0
                    confidence = pattern_info["base_confidence"] + position_bonus

                    if confidence > best_confidence:
                        best_intent = intent_type
                        best_confidence = confidence

        # If no match found, return UNKNOWN with low confidence
        if best_intent == IntentType.UNKNOWN:
            best_confidence = 0.3

        return best_intent, min(best_confidence, 1.0)


# =============================================================================
# Entity Extraction
# =============================================================================

class EntityExtractor:
    """
    Rule-based entity extractor for demonstration.

    In production, this would be replaced with a trained NER model
    (e.g., spaCy, fine-tuned BERT NER, or custom sequence labeler).
    """

    # Entity lexicons - simplified for demonstration
    OBJECT_WORDS = [
        "cup", "mug", "glass", "bottle", "can",
        "box", "package", "parcel",
        "ball", "toy", "book", "pen", "pencil",
        "plate", "bowl", "spoon", "fork", "knife",
        "phone", "remote", "tool", "screwdriver"
    ]

    LOCATION_WORDS = [
        "table", "desk", "shelf", "counter", "floor",
        "kitchen", "bedroom", "living room", "bathroom",
        "left", "right", "front", "back",
        "drawer", "cabinet", "refrigerator", "fridge"
    ]

    PERSON_WORDS = [
        "me", "myself", "john", "jane", "user", "operator",
        "him", "her", "them", "person"
    ]

    COLOR_WORDS = [
        "red", "blue", "green", "yellow", "orange",
        "purple", "pink", "black", "white", "gray", "grey",
        "brown", "transparent", "clear"
    ]

    SIZE_WORDS = [
        "large", "big", "small", "tiny", "huge",
        "tall", "short", "wide", "narrow", "thick", "thin"
    ]

    QUANTITY_PATTERNS = [
        (r"\b(one|1)\b", 1),
        (r"\b(two|2)\b", 2),
        (r"\b(three|3)\b", 3),
        (r"\b(four|4)\b", 4),
        (r"\b(five|5)\b", 5),
        (r"\ball\b", -1),  # -1 represents "all"
    ]

    def extract(self, text: str) -> list[Entity]:
        """
        Extract all entities from text.

        Args:
            text: Preprocessed command text

        Returns:
            List of extracted entities with types and positions
        """
        entities = []
        text_lower = text.lower()

        # Extract objects
        for word in self.OBJECT_WORDS:
            if word in text_lower:
                start = text_lower.find(word)
                entities.append(Entity(
                    entity_type=EntityType.OBJECT,
                    value=word,
                    confidence=0.90,
                    start_idx=start,
                    end_idx=start + len(word)
                ))

        # Extract locations
        for word in self.LOCATION_WORDS:
            if word in text_lower:
                start = text_lower.find(word)
                entities.append(Entity(
                    entity_type=EntityType.LOCATION,
                    value=word,
                    confidence=0.85,
                    start_idx=start,
                    end_idx=start + len(word)
                ))

        # Extract persons
        for word in self.PERSON_WORDS:
            pattern = rf"\b{word}\b"
            match = re.search(pattern, text_lower)
            if match:
                entities.append(Entity(
                    entity_type=EntityType.PERSON,
                    value=word,
                    confidence=0.85,
                    start_idx=match.start(),
                    end_idx=match.end()
                ))

        # Extract colors
        for word in self.COLOR_WORDS:
            if word in text_lower:
                start = text_lower.find(word)
                entities.append(Entity(
                    entity_type=EntityType.COLOR,
                    value=word,
                    confidence=0.92,
                    start_idx=start,
                    end_idx=start + len(word)
                ))

        # Extract sizes
        for word in self.SIZE_WORDS:
            if word in text_lower:
                start = text_lower.find(word)
                entities.append(Entity(
                    entity_type=EntityType.SIZE,
                    value=word,
                    confidence=0.88,
                    start_idx=start,
                    end_idx=start + len(word)
                ))

        # Extract quantities
        for pattern, value in self.QUANTITY_PATTERNS:
            match = re.search(pattern, text_lower)
            if match:
                entities.append(Entity(
                    entity_type=EntityType.QUANTITY,
                    value=str(value),
                    confidence=0.95,
                    start_idx=match.start(),
                    end_idx=match.end()
                ))

        return entities


# =============================================================================
# Slot Filling
# =============================================================================

class SlotFiller:
    """
    Fills intent slots from extracted entities.

    Maps extracted entities to the appropriate slots based on
    intent type and entity positions in the text.
    """

    # Define required and optional slots for each intent
    INTENT_SLOTS = {
        IntentType.PICK: {
            "required": ["target_object"],
            "optional": ["object_color", "object_size", "source_location", "quantity"]
        },
        IntentType.PLACE: {
            "required": ["target_object", "destination"],
            "optional": ["object_color", "object_size"]
        },
        IntentType.MOVE: {
            "required": ["destination"],
            "optional": []
        },
        IntentType.FIND: {
            "required": ["target_object"],
            "optional": ["object_color", "object_size", "source_location"]
        },
        IntentType.GIVE: {
            "required": ["target_object", "target_person"],
            "optional": ["object_color", "object_size"]
        },
        IntentType.SPEAK: {
            "required": [],
            "optional": []
        },
        IntentType.WAIT: {
            "required": [],
            "optional": ["quantity"]  # Wait duration in seconds
        },
        IntentType.STOP: {
            "required": [],
            "optional": []
        }
    }

    def fill_slots(
        self,
        intent_type: IntentType,
        intent_confidence: float,
        entities: list[Entity],
        original_text: str
    ) -> Intent:
        """
        Fill intent slots from extracted entities.

        Args:
            intent_type: Classified intent type
            intent_confidence: Confidence of classification
            entities: List of extracted entities
            original_text: Original command text

        Returns:
            Filled Intent object
        """
        intent = Intent(
            intent_type=intent_type,
            confidence=intent_confidence,
            original_text=original_text
        )

        # Group entities by type
        entities_by_type: dict[EntityType, list[Entity]] = {}
        for entity in entities:
            if entity.entity_type not in entities_by_type:
                entities_by_type[entity.entity_type] = []
            entities_by_type[entity.entity_type].append(entity)

        # Fill object-related slots
        if EntityType.OBJECT in entities_by_type:
            # Take first object mention as target
            intent.target_object = entities_by_type[EntityType.OBJECT][0]

        if EntityType.COLOR in entities_by_type:
            intent.object_color = entities_by_type[EntityType.COLOR][0]

        if EntityType.SIZE in entities_by_type:
            intent.object_size = entities_by_type[EntityType.SIZE][0]

        # Fill location slots based on prepositions
        if EntityType.LOCATION in entities_by_type:
            locations = entities_by_type[EntityType.LOCATION]
            text_lower = original_text.lower()

            for loc in locations:
                # Check preceding context for source/destination
                before_loc = text_lower[:loc.start_idx]
                if "from" in before_loc or "on" in before_loc:
                    intent.source_location = loc
                elif "to" in before_loc or "at" in before_loc:
                    intent.destination = loc
                else:
                    # Default: use context from intent
                    if intent_type == IntentType.PICK:
                        intent.source_location = loc
                    elif intent_type in [IntentType.PLACE, IntentType.MOVE]:
                        intent.destination = loc

        # Fill person slot
        if EntityType.PERSON in entities_by_type:
            intent.target_person = entities_by_type[EntityType.PERSON][0]

        # Fill quantity
        if EntityType.QUANTITY in entities_by_type:
            qty_entity = entities_by_type[EntityType.QUANTITY][0]
            intent.quantity = int(qty_entity.value) if qty_entity.value != "-1" else -1

        # Check for missing required slots
        self._check_required_slots(intent)

        return intent

    def _check_required_slots(self, intent: Intent) -> None:
        """Check if required slots are filled, mark for disambiguation if not."""
        if intent.intent_type not in self.INTENT_SLOTS:
            return

        required = self.INTENT_SLOTS[intent.intent_type]["required"]
        missing = []

        if "target_object" in required and intent.target_object is None:
            missing.append("object")
        if "destination" in required and intent.destination is None:
            missing.append("destination")
        if "target_person" in required and intent.target_person is None:
            missing.append("person")

        if missing:
            intent.requires_disambiguation = True
            intent.disambiguation_reason = f"Missing required: {', '.join(missing)}"


# =============================================================================
# Voice Command Parser (Main Pipeline)
# =============================================================================

class VoiceCommandParser:
    """
    Complete voice command parsing pipeline.

    Combines preprocessing, intent classification, entity extraction,
    and slot filling into a unified interface.
    """

    def __init__(self):
        self.classifier = IntentClassifier()
        self.extractor = EntityExtractor()
        self.slot_filler = SlotFiller()

    def preprocess(self, text: str) -> str:
        """
        Preprocess text for parsing.

        Args:
            text: Raw transcript text

        Returns:
            Cleaned and normalized text
        """
        # Remove extra whitespace
        text = " ".join(text.split())

        # Remove common filler words at start
        fillers = ["please", "could you", "can you", "would you", "i want you to"]
        text_lower = text.lower()
        for filler in fillers:
            if text_lower.startswith(filler):
                text = text[len(filler):].strip()
                text_lower = text.lower()

        return text

    def parse(self, transcript: str) -> Intent:
        """
        Parse a voice command transcript into structured intent.

        Args:
            transcript: Text from speech recognition

        Returns:
            Structured Intent object
        """
        # Step 1: Preprocess
        cleaned_text = self.preprocess(transcript)

        # Step 2: Classify intent
        intent_type, intent_confidence = self.classifier.classify(cleaned_text)

        # Step 3: Extract entities
        entities = self.extractor.extract(cleaned_text)

        # Step 4: Fill slots
        intent = self.slot_filler.fill_slots(
            intent_type=intent_type,
            intent_confidence=intent_confidence,
            entities=entities,
            original_text=transcript
        )

        return intent

    def parse_with_details(self, transcript: str) -> dict:
        """
        Parse command and return detailed results for debugging.

        Args:
            transcript: Text from speech recognition

        Returns:
            Dictionary with all intermediate results
        """
        cleaned_text = self.preprocess(transcript)
        intent_type, intent_confidence = self.classifier.classify(cleaned_text)
        entities = self.extractor.extract(cleaned_text)
        intent = self.slot_filler.fill_slots(
            intent_type, intent_confidence, entities, transcript
        )

        return {
            "original": transcript,
            "preprocessed": cleaned_text,
            "intent_type": intent_type.name,
            "intent_confidence": intent_confidence,
            "entities": [
                {
                    "type": e.entity_type.name,
                    "value": e.value,
                    "confidence": e.confidence
                }
                for e in entities
            ],
            "filled_intent": intent
        }


# =============================================================================
# Usage Example
# =============================================================================

def main():
    """Demonstrate voice command parsing pipeline."""
    print("=" * 60)
    print("Voice Command Parser Demo")
    print("=" * 60)

    parser = VoiceCommandParser()

    # Test commands
    test_commands = [
        "Please pick up the large red cup from the table",
        "Put the bottle on the shelf",
        "Find my phone",
        "Give the book to John",
        "Move to the kitchen",
        "Stop",
        "Wait",
        "Pick up something",  # Missing object - should trigger disambiguation
    ]

    for command in test_commands:
        print(f"\n--- Command: \"{command}\" ---\n")

        result = parser.parse_with_details(command)

        print(f"  Intent: {result['intent_type']} (conf: {result['intent_confidence']:.2f})")
        print(f"  Entities extracted:")
        for entity in result['entities']:
            print(f"    - {entity['type']}: \"{entity['value']}\" ({entity['confidence']:.2f})")

        intent = result['filled_intent']
        print(f"  Filled slots:")
        if intent.target_object:
            print(f"    - Object: {intent.target_object.value}")
        if intent.object_color:
            print(f"    - Color: {intent.object_color.value}")
        if intent.object_size:
            print(f"    - Size: {intent.object_size.value}")
        if intent.source_location:
            print(f"    - Source: {intent.source_location.value}")
        if intent.destination:
            print(f"    - Destination: {intent.destination.value}")
        if intent.target_person:
            print(f"    - Person: {intent.target_person.value}")

        if intent.requires_disambiguation:
            print(f"  ⚠ DISAMBIGUATION NEEDED: {intent.disambiguation_reason}")
        else:
            print("  ✓ Command ready for execution")

    print("\n" + "=" * 60)
    print("Demo complete")
    print("=" * 60)


if __name__ == "__main__":
    main()
