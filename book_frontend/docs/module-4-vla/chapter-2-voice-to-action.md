---
sidebar_position: 2
title: Chapter 2 - Voice-to-Action
---

# Chapter 2: Voice-to-Action Pipeline

## Introduction

When you say "pick up the red cup," your words embark on a complex journey before the robot's gripper ever moves. This chapter explores the voice-to-action pipeline—the sophisticated system that transforms spoken commands into structured robot instructions. Understanding this pipeline is essential for building robots that respond naturally to human speech.

The voice-to-action pipeline bridges the gap between human communication patterns and robot capabilities. Unlike typing commands into a terminal, voice interaction is messy: people speak with varying accents, use incomplete sentences, add filler words, and refer to objects indirectly. A robust pipeline must handle this variability while producing precise, unambiguous instructions for the robot.

Building on the VLA foundations from Chapter 1, we now dive into the technical details of speech recognition, intent extraction, entity parsing, and the critical process of resolving ambiguous references against the robot's visual perception of its environment.

### Prerequisites

Before starting this chapter, ensure you have:
- Completed Chapter 1: VLA Foundations
- Understanding of basic NLP concepts (tokens, classification)
- Familiarity with ROS 2 message types from Module 1
- Basic knowledge of neural network architectures

### Learning Objectives

By the end of this chapter, you will be able to:
1. Explain the stages of speech recognition and their role in robot voice interfaces
2. Implement intent classification and entity extraction for robot commands
3. Design slot-filling systems that map natural language to robot action parameters
4. Handle ambiguous references through context resolution and clarification

---

## Core Concept 1: Speech Recognition Fundamentals

### From Sound Waves to Text

Speech recognition (Automatic Speech Recognition, ASR) converts audio signals into text. For robotics, this is the essential first step in voice-commanded operation. The process involves several stages:

**Audio Capture**: Microphones convert sound pressure waves into electrical signals. Robot microphone arrays often use multiple microphones for:
- Beamforming: Focus on sound from a specific direction
- Noise cancellation: Suppress motor and environmental noise
- Speaker localization: Determine where the speaker is

**Feature Extraction**: Raw audio is converted to features that capture speech characteristics:
- Mel-frequency cepstral coefficients (MFCCs): Traditional features mimicking human auditory perception
- Mel spectrograms: Time-frequency representations used by modern neural networks
- Log filterbank energies: Simplified frequency representations

**Acoustic Modeling**: Neural networks map audio features to phonemes or characters:
- Connectionist Temporal Classification (CTC): Handles variable-length alignment
- Attention mechanisms: Learn which audio frames correspond to which output tokens
- Transformer architectures: Enable parallel processing with self-attention

### Modern ASR: The Whisper Model

OpenAI's Whisper has emerged as a leading ASR model for robotics applications:

**Architecture**: Encoder-decoder transformer trained on 680,000 hours of multilingual audio
- Encoder: Processes mel spectrogram features
- Decoder: Generates text tokens autoregressively

**Strengths for Robotics**:
- Robust to background noise (trained on diverse audio)
- Handles accents and speaking styles
- Supports 99 languages
- Available in multiple sizes (tiny to large)

**Latency Considerations**:
- Whisper processes complete utterances, not streaming
- Typical latency: 200-500ms for short commands
- For faster response, use Voice Activity Detection (VAD) to detect speech end

### ROS 2 Integration Pattern

```python
# Conceptual ROS 2 node for speech recognition
class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition')

        # Subscribe to audio stream
        self.audio_sub = self.create_subscription(
            AudioData, '/microphone/audio', self.audio_callback, 10)

        # Publish transcriptions
        self.transcript_pub = self.create_publisher(
            String, '/speech/transcript', 10)

        # VAD to detect speech boundaries
        self.vad = VoiceActivityDetector()
        self.audio_buffer = []

    def audio_callback(self, msg):
        # Accumulate audio during speech
        if self.vad.is_speech(msg.data):
            self.audio_buffer.extend(msg.data)
        elif self.audio_buffer:
            # Speech ended, transcribe
            transcript = self.transcribe(self.audio_buffer)
            self.transcript_pub.publish(String(data=transcript))
            self.audio_buffer = []
```

See the [Voice Pipeline](../diagrams/module-4/voice-pipeline.md) diagram for the complete audio processing flow.

---

## Core Concept 2: Command Parsing and Intent Extraction

### From Text to Structure

Once we have text, we must extract its meaning. Intent extraction converts natural language into structured representations with two key components:

**Intent**: The action type the user wants (PICK, PLACE, MOVE, FIND)
**Entities**: Parameters for the action (object, location, attributes)

For example:
- Input: "Pick up the large red cup from the table"
- Intent: PICK
- Entities:
  ```python
  {"object": "cup", "color": "red", "size": "large", "source": "table"}
  ```

### Intent Classification Approaches

**Rule-Based Systems**:
Simple pattern matching for constrained vocabularies
```python
if "pick" in text or "grab" in text:
    intent = IntentType.PICK
elif "put" in text or "place" in text:
    intent = IntentType.PLACE
```
- Pros: Fast, interpretable, no training data needed
- Cons: Brittle, doesn't handle variations

**Machine Learning Classifiers**:
Trained models that learn intent from examples
- Traditional: SVM, Random Forest on TF-IDF features
- Neural: CNN, LSTM on word embeddings
- Transformer: BERT, RoBERTa fine-tuned for classification

**Production Approach**:
Most systems use transformer-based models:
```text
Input: "Could you please grab the blue bottle"

[CLS] could you please grab the blue bottle [SEP]
         ↓
    BERT Encoder
         ↓
    Classification Head
         ↓
    Intent: PICK (0.91)
```

### Confidence and Thresholds

Intent classification produces probabilities, not certainties. Handling low confidence is critical for safety:

```python
def classify_with_threshold(text, threshold=0.7):
    intent, confidence = classifier.predict(text)

    if confidence >= threshold:
        return intent, "confirmed"
    elif confidence >= 0.5:
        return intent, "needs_confirmation"
    else:
        return None, "ask_clarification"
```

For robot safety, conservative thresholds are preferred:
- High-risk actions (PICK, PLACE): threshold = 0.8
- Navigation (MOVE): threshold = 0.7
- Information queries (FIND): threshold = 0.6

---

## Core Concept 3: Entity Recognition for Robotics

### Domain-Specific Entity Types

Generic Named Entity Recognition (NER) identifies persons, organizations, and locations. Robot commands require specialized entity types:

| Entity Type | Examples | Role in Commands |
|-------------|----------|------------------|
| OBJECT | cup, bottle, box, ball | Target of manipulation |
| LOCATION | table, shelf, kitchen | Source or destination |
| PERSON | me, John, the operator | Recipient of handover |
| COLOR | red, blue, transparent | Visual identifier |
| SIZE | large, small, tall | Physical attribute |
| QUANTITY | one, three, all | Count for multiple objects |
| DIRECTION | left, right, behind | Spatial reference |

### BIO Tagging for Entity Extraction

Entity extraction uses sequence labeling with BIO (Begin, Inside, Outside) tags:

```text
Token:    pick  up   the  large  red   cup   from  the  table
BIO Tag:  O     O    O    B-SIZE B-CLR B-OBJ O     O    B-LOC

Extracted:
- SIZE: "large" (span 3-4)
- COLOR: "red" (span 4-5)
- OBJECT: "cup" (span 5-6)
- LOCATION: "table" (span 8-9)
```

**B** (Begin): First token of an entity
**I** (Inside): Continuation of an entity (for multi-word entities)
**O** (Outside): Not part of any entity

### Handling Compound References

Natural language often uses compound references that must be correctly parsed:

"The large red cup" → Single object with attributes
- OBJECT: cup
- COLOR: red
- SIZE: large

"The cup and the bottle" → Two separate objects
- OBJECT_1: cup
- OBJECT_2: bottle

"Three red cups" → Multiple instances
- OBJECT: cup
- COLOR: red
- QUANTITY: 3

Correct parsing requires understanding syntactic structure and domain knowledge about physical objects.

---

## Core Concept 4: Context Resolution for Ambiguous References

### The Reference Resolution Challenge

Commands often contain ambiguous references that require visual grounding:

- "Pick up **the cup**" → Which cup, if multiple visible?
- "Put it **there**" → Where exactly is "there"?
- "Give me **that one**" → Which one was indicated?
- "The **other** bottle" → Requires dialogue history

### Resolution Strategies

**Attribute Matching**:
Use extracted attributes to filter candidates
```python
def resolve_by_attributes(description, detected_objects):
    candidates = detected_objects

    if description.color:
        candidates = [o for o in candidates
                     if o.color == description.color]

    if description.size:
        candidates = [o for o in candidates
                     if matches_size(o, description.size)]

    return candidates
```

**Spatial Grounding**:
Use location references to narrow candidates
```python
def resolve_by_location(description, detected_objects, locations):
    if description.location:
        target_loc = find_location(description.location, locations)
        candidates = [o for o in detected_objects
                     if is_on_or_near(o, target_loc)]
    return candidates
```

**Dialogue History**:
Track recently mentioned objects for pronouns
```python
class DialogueContext:
    def __init__(self):
        self.recent_objects = []  # Stack of mentioned objects

    def resolve_pronoun(self, pronoun):
        if pronoun in ["it", "that", "this"]:
            return self.recent_objects[-1] if self.recent_objects else None
        return None
```

### Clarification Dialogue

When resolution fails, the robot should ask clarifying questions:

```text
User: "Pick up the cup"
Robot: "I see 3 cups. Which one do you mean?"
       [Shows options: red cup, blue cup, white cup]
User: "The red one"
Robot: "Got it. Picking up the red cup."
```

Effective clarification:
- Presents specific options (not open-ended questions)
- Uses visual feedback when possible
- Limits to 2-3 choices
- Allows pointing or gestural input

See the [Intent Extraction Flow](../diagrams/module-4/intent-extraction-flow.md) for the complete parsing pipeline.

---

## Hands-On Example

### Example: Voice Command Parser

The following code demonstrates a complete voice command parsing pipeline:

```python
#!/usr/bin/env python3
"""
Voice Command Parser - Intent and Entity Extraction
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class IntentType(Enum):
    PICK = auto()
    PLACE = auto()
    MOVE = auto()
    FIND = auto()
    GIVE = auto()


@dataclass
class Entity:
    entity_type: str
    value: str
    confidence: float


@dataclass
class Intent:
    intent_type: IntentType
    confidence: float
    target_object: Optional[Entity] = None
    object_color: Optional[Entity] = None
    source_location: Optional[Entity] = None
    destination: Optional[Entity] = None
    requires_disambiguation: bool = False


class VoiceCommandParser:
    """Parse voice commands into structured intents."""

    INTENT_TRIGGERS = {
        IntentType.PICK: ["pick up", "pick", "grab", "take", "get"],
        IntentType.PLACE: ["place", "put", "set down", "drop"],
        IntentType.MOVE: ["move to", "go to", "navigate to"],
        IntentType.FIND: ["find", "locate", "where is"],
        IntentType.GIVE: ["give", "hand", "pass to"],
    }

    OBJECTS = ["cup", "bottle", "box", "ball", "book", "phone"]
    COLORS = ["red", "blue", "green", "yellow", "white", "black"]
    LOCATIONS = ["table", "shelf", "floor", "counter", "desk"]

    def parse(self, transcript: str) -> Intent:
        """Parse transcript into structured intent."""
        text = transcript.lower()

        # 1. Classify intent
        intent_type, confidence = self._classify_intent(text)

        # 2. Extract entities
        obj = self._extract_entity(text, self.OBJECTS, "object")
        color = self._extract_entity(text, self.COLORS, "color")
        location = self._extract_entity(text, self.LOCATIONS, "location")

        # 3. Build intent
        intent = Intent(
            intent_type=intent_type,
            confidence=confidence,
            target_object=obj,
            object_color=color,
        )

        # 4. Assign location based on context
        if location:
            if "from" in text:
                intent.source_location = location
            else:
                intent.destination = location

        # 5. Check if disambiguation needed
        if intent_type == IntentType.PICK and not obj:
            intent.requires_disambiguation = True

        return intent

    def _classify_intent(self, text: str) -> tuple[IntentType, float]:
        for intent_type, triggers in self.INTENT_TRIGGERS.items():
            for trigger in triggers:
                if trigger in text:
                    return intent_type, 0.85
        return IntentType.FIND, 0.5  # Default fallback

    def _extract_entity(
        self, text: str, vocabulary: list, entity_type: str
    ) -> Optional[Entity]:
        for word in vocabulary:
            if word in text:
                return Entity(entity_type, word, 0.9)
        return None


# Usage example
parser = VoiceCommandParser()
result = parser.parse("Please pick up the red cup from the table")

print(f"Intent: {result.intent_type.name}")
print(f"Object: {result.target_object.value if result.target_object else 'None'}")
print(f"Color: {result.object_color.value if result.object_color else 'None'}")
print(f"Source: {result.source_location.value if result.source_location else 'None'}")
```

**Key Points**:
- Intent classification uses trigger word matching (production would use ML models)
- Entity extraction searches for domain-specific vocabulary
- Location assignment depends on preposition context ("from" vs default)
- Missing required entities trigger disambiguation
- Confidence scores enable graceful degradation

For the complete implementation with all entity types and slot filling, see the [Voice Command Parser](../code-examples/module-4/voice_command_parser.py) code example.

---

## Summary

This chapter explored the complete voice-to-action pipeline that transforms spoken commands into structured robot instructions:

1. **Speech Recognition**: ASR systems like Whisper convert audio to text, with microphone arrays handling noise and beamforming for robot environments, while Voice Activity Detection manages speech boundaries.

2. **Intent Classification**: Commands are classified into action types (PICK, PLACE, MOVE) using pattern matching or trained ML models, with confidence thresholds determining when clarification is needed.

3. **Entity Extraction**: Domain-specific NER identifies objects, locations, colors, sizes, and quantities using BIO tagging schemes adapted for robotics vocabulary and compound references.

4. **Slot Filling**: Extracted entities are mapped to intent-specific slots (target_object, destination, etc.) with rules handling defaults, inference, and required vs optional parameters.

5. **Context Resolution**: Ambiguous references are resolved through attribute matching, spatial grounding, and dialogue history, with clarification dialogues when automatic resolution fails.

6. **Production Considerations**: Robust voice interfaces require conservative confidence thresholds for safety, graceful degradation to clarification, and integration with visual perception for grounding.

**Next Steps**: Chapter 3 explores how Large Language Models decompose high-level intents into executable action sequences, building on the structured commands produced by the voice pipeline.

---

## Self-Assessment

Test your understanding of voice-to-action pipelines:

1. **What are the main stages of the speech recognition pipeline, from audio capture to text output?**
   <details>
   <summary>Show Answer</summary>
   The stages are: (1) Audio Capture - microphones convert sound waves to electrical signals, often using arrays for beamforming and noise cancellation; (2) Feature Extraction - converting audio to mel spectrograms or MFCCs; (3) Acoustic Modeling - neural networks map features to phonemes/characters using CTC or attention mechanisms; (4) Decoding - generating final text output, typically with language model integration for context.
   </details>

2. **Why is Voice Activity Detection (VAD) important for robotics applications?**
   <details>
   <summary>Show Answer</summary>
   VAD is important because: (1) It detects speech boundaries so the system knows when to process audio vs when to ignore silence or noise; (2) It prevents wasting computation on non-speech audio; (3) Many ASR models like Whisper process complete utterances, so VAD determines when speech has ended; (4) It enables turn-taking in dialogue by detecting when the user has finished speaking.
   </details>

3. **Explain the difference between intent classification and entity extraction, using "Put the blue bottle on the shelf" as an example.**
   <details>
   <summary>Show Answer</summary>
   Intent classification identifies the action type—here PLACE (put). Entity extraction identifies the parameters: OBJECT="bottle", COLOR="blue", LOCATION="shelf". The intent tells WHAT to do, entities tell WITH WHAT and WHERE. Both are needed to form a complete actionable command.
   </details>

4. **What does BIO tagging mean in entity extraction, and how would you tag "the large red cup"?**
   <details>
   <summary>Show Answer</summary>
   BIO stands for Begin, Inside, Outside. B marks the first token of an entity, I marks continuation tokens, O marks non-entity tokens. "the large red cup" would be tagged as: "the"=O, "large"=B-SIZE, "red"=B-COLOR, "cup"=B-OBJECT. Each attribute starts with B since they're single-token entities in this case.
   </details>

5. **Why are confidence thresholds important for intent classification in robotics, and how should they vary by action type?**
   <details>
   <summary>Show Answer</summary>
   Confidence thresholds are critical because misclassified intents lead to wrong physical actions that may be dangerous or irreversible. High-risk actions (PICK, PLACE involving manipulation) should use higher thresholds (0.8+) while lower-risk queries (FIND, information requests) can use lower thresholds (0.6). When confidence is below threshold, the system should request clarification rather than guess.
   </details>

6. **Describe three strategies for resolving ambiguous object references like "the cup" when multiple cups are visible.**
   <details>
   <summary>Show Answer</summary>
   Three strategies: (1) Attribute Matching - use extracted attributes (color, size) to filter candidates, e.g., "red cup" narrows to cups with red color; (2) Spatial Grounding - use location references like "on the table" to filter by position; (3) Dialogue History - track recently mentioned objects so "it" or "that one" refers to the last discussed object. If these fail, request clarification with specific options.
   </details>

7. **What makes entity types for robotics different from generic NER systems?**
   <details>
   <summary>Show Answer</summary>
   Robotics NER requires domain-specific types not found in generic systems: OBJECT (manipulable items), SIZE (physical dimensions), COLOR (visual attributes), DIRECTION (spatial references like left/right), QUANTITY (for multiple objects). Generic NER focuses on PERSON, ORGANIZATION, LOCATION (places/countries) which are less relevant for physical manipulation commands.
   </details>

8. **Design a clarification dialogue for when the user says "Pick up that one" but the robot doesn't know which object is meant.**
   <details>
   <summary>Show Answer</summary>
   Good clarification: Robot: "I'm not sure which object you mean. Could you describe it? For example, 'the red cup' or 'the bottle on the left'." Or with visual options: "I see several objects. Are you referring to: (1) the red cup, (2) the blue bottle, or (3) the green ball?" Key principles: offer specific choices (not open-ended), limit to 2-3 options, allow pointing/gestures if supported, repeat back understanding for confirmation.
   </details>
