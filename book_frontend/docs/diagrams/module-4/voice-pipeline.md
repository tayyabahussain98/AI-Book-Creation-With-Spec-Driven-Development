---
title: Voice-to-Action Pipeline
module: 4
chapter: 2
type: flow
---

# Voice-to-Action Pipeline

## Overview

This diagram illustrates the complete pipeline for converting spoken natural language commands into robot-executable actions. The pipeline spans from audio capture through speech recognition, natural language understanding, and action generation.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                      VOICE-TO-ACTION PIPELINE                               │
└─────────────────────────────────────────────────────────────────────────────┘

┌──────────────┐
│  User Voice  │   "Pick up the red cup on the table"
└──────┬───────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 1: AUDIO CAPTURE                                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│  │ Microphone  │───►│   Voice     │───►│   Noise     │───►│   Audio     │  │
│  │   Array     │    │  Activity   │    │ Suppression │    │   Buffer    │  │
│  │             │    │  Detection  │    │   (VAD)     │    │  (16kHz)    │  │
│  └─────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘  │
│                                                                   │         │
└───────────────────────────────────────────────────────────────────┼─────────┘
                                                                    │
       ┌────────────────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 2: SPEECH RECOGNITION (ASR)                                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                        Whisper Model                                 │   │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐  │   │
│  │  │   Audio     │───►│  Encoder    │───►│       Decoder           │  │   │
│  │  │  Features   │    │ (Transformer│    │  (Autoregressive)       │  │   │
│  │  │ (Mel Spec)  │    │   Layers)   │    │                         │  │   │
│  │  └─────────────┘    └─────────────┘    └───────────┬─────────────┘  │   │
│  └────────────────────────────────────────────────────┼────────────────┘   │
│                                                        │                    │
│  Output: "pick up the red cup on the table"     conf: 0.94                 │
│                                                        │                    │
└────────────────────────────────────────────────────────┼────────────────────┘
                                                         │
       ┌─────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 3: NATURAL LANGUAGE UNDERSTANDING (NLU)                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      Intent Classification                           │   │
│  │                                                                      │   │
│  │   Input: "pick up the red cup on the table"                         │   │
│  │                                                                      │   │
│  │   ┌──────────────────────────────────────────────────────────────┐  │   │
│  │   │  Candidate Intents:                                          │  │   │
│  │   │    PICK      ████████████████████████████  0.89              │  │   │
│  │   │    MOVE      ███                           0.05              │  │   │
│  │   │    PLACE     ██                            0.03              │  │   │
│  │   │    FIND      █                             0.02              │  │   │
│  │   │    OTHER     █                             0.01              │  │   │
│  │   └──────────────────────────────────────────────────────────────┘  │   │
│  │                                                                      │   │
│  │   Selected Intent: PICK (confidence: 0.89)                          │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      Entity Extraction                               │   │
│  │                                                                      │   │
│  │   "pick up the [red]_ATTR [cup]_OBJ on the [table]_LOC"            │   │
│  │                                                                      │   │
│  │   ┌────────────────────────────────────────────────────────────┐    │   │
│  │   │  Extracted Entities:                                       │    │   │
│  │   │    OBJECT:    "cup"      (confidence: 0.95)               │    │   │
│  │   │    ATTRIBUTE: "red"      (confidence: 0.92)               │    │   │
│  │   │    LOCATION:  "table"    (confidence: 0.88)               │    │   │
│  │   └────────────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 4: CONTEXT RESOLUTION                                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────────┐         ┌──────────────────────────────────────┐ │
│  │   Scene State Graph  │         │        Resolution Process            │ │
│  │                      │         │                                      │ │
│  │  Objects:            │         │  Query: "red cup on table"           │ │
│  │   • cup_001 (red)    │◄───────►│                                      │ │
│  │   • cup_002 (blue)   │         │  Candidates:                         │ │
│  │   • cup_003 (red)    │         │   • cup_001: red, ON table_001 ✓     │ │
│  │                      │         │   • cup_003: red, NEAR table_001     │ │
│  │  Locations:          │         │                                      │ │
│  │   • table_001        │         │  Selected: cup_001 (score: 0.91)     │ │
│  │   • counter_001      │         │                                      │ │
│  └──────────────────────┘         └──────────────────────────────────────┘ │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  Ambiguity Handling                                                  │   │
│  │                                                                      │   │
│  │  If multiple matches:                                                │   │
│  │   • Score by attribute match (color, size)                          │   │
│  │   • Score by spatial relationship (ON vs NEAR)                      │   │
│  │   • Score by recency (recently mentioned objects)                   │   │
│  │   • If still ambiguous → ASK for clarification                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 5: OUTPUT GENERATION                                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      Structured Command                              │   │
│  │                                                                      │   │
│  │  {                                                                   │   │
│  │    "intent": "PICK",                                                │   │
│  │    "confidence": 0.89,                                              │   │
│  │    "entities": {                                                    │   │
│  │      "object": {                                                    │   │
│  │        "id": "cup_001",                                             │   │
│  │        "type": "cup",                                               │   │
│  │        "attributes": {"color": "red"},                              │   │
│  │        "position": [0.5, 0.3, 0.8]                                  │   │
│  │      },                                                             │   │
│  │      "reference_location": {                                        │   │
│  │        "id": "table_001",                                           │   │
│  │        "relation": "ON"                                             │   │
│  │      }                                                              │   │
│  │    },                                                               │   │
│  │    "original_text": "pick up the red cup on the table"              │   │
│  │  }                                                                   │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
       │
       ▼
┌──────────────────┐
│  To: Planning    │
│      Layer       │
└──────────────────┘
```

---

## Component Descriptions

### Stage 1: Audio Capture

**Microphone Array**
- **Purpose**: Capture high-quality audio with spatial information
- **Inputs**: Acoustic waves
- **Outputs**: Multi-channel audio streams
- **Key Details**: Far-field microphone arrays enable beamforming for noise rejection

**Voice Activity Detection (VAD)**
- **Purpose**: Detect when user is speaking vs silence
- **Inputs**: Audio stream
- **Outputs**: Speech segments with timestamps
- **Key Details**: Prevents processing of background noise, enables turn-taking

**Noise Suppression**
- **Purpose**: Remove background noise from speech signal
- **Inputs**: Raw audio with noise
- **Outputs**: Clean audio signal
- **Key Details**: Critical for robotics environments with motor noise

**Audio Buffer**
- **Purpose**: Accumulate speech segments for batch processing
- **Inputs**: Clean audio segments
- **Outputs**: Complete utterance at 16kHz sample rate
- **Key Details**: Whisper expects 16kHz mono audio

### Stage 2: Speech Recognition (ASR)

**Whisper Model**
- **Purpose**: Convert audio to text with high accuracy
- **Inputs**: Audio features (mel spectrogram)
- **Outputs**: Transcribed text with confidence scores
- **Key Details**: Supports 99 languages, robust to noise and accents

**Audio Features (Mel Spectrogram)**
- **Purpose**: Convert time-domain audio to frequency representation
- **Inputs**: Raw audio samples
- **Outputs**: 80-dimensional mel spectrogram
- **Key Details**: Standard preprocessing for transformer-based ASR

**Encoder (Transformer Layers)**
- **Purpose**: Extract acoustic features from spectrogram
- **Inputs**: Mel spectrogram
- **Outputs**: Encoded audio representations
- **Key Details**: 12-32 transformer layers depending on model size

**Decoder (Autoregressive)**
- **Purpose**: Generate text tokens from encoded audio
- **Inputs**: Encoder outputs + previous tokens
- **Outputs**: Token sequence with probabilities
- **Key Details**: Autoregressive generation with beam search

### Stage 3: Natural Language Understanding (NLU)

**Intent Classification**
- **Purpose**: Determine what action the user wants
- **Inputs**: Transcribed text
- **Outputs**: Intent label with confidence
- **Key Details**: Multi-class classification into PICK, PLACE, MOVE, FIND, etc.

**Entity Extraction**
- **Purpose**: Extract relevant parameters from command
- **Inputs**: Transcribed text + intent
- **Outputs**: Labeled entities (objects, locations, attributes)
- **Key Details**: Named entity recognition adapted for robotics domain

### Stage 4: Context Resolution

**Scene State Graph**
- **Purpose**: Provide visual grounding for entities
- **Inputs**: Detected objects, spatial relationships
- **Outputs**: Object instances with positions and attributes
- **Key Details**: Continuously updated from vision system

**Resolution Process**
- **Purpose**: Map textual references to specific objects
- **Inputs**: Entity descriptions, scene graph
- **Outputs**: Resolved object IDs with confidence
- **Key Details**: Handles ambiguity through attribute and spatial matching

**Ambiguity Handling**
- **Purpose**: Resolve cases with multiple matching objects
- **Inputs**: Multiple candidate objects
- **Outputs**: Best match or clarification request
- **Key Details**: Scoring by attributes, spatial relations, and dialogue history

### Stage 5: Output Generation

**Structured Command**
- **Purpose**: Package all information for planning layer
- **Inputs**: Intent, resolved entities, confidence scores
- **Outputs**: JSON structure with complete command specification
- **Key Details**: Standardized format for downstream processing

---

## Data Flow

1. **Audio Capture**: Microphone array captures user speech with spatial information
2. **VAD**: Voice activity detection isolates speech from silence
3. **Preprocessing**: Noise suppression cleans signal, buffer accumulates utterance
4. **ASR**: Whisper transcribes audio to text with confidence
5. **Intent Classification**: Determine action type (PICK, MOVE, etc.)
6. **Entity Extraction**: Identify objects, locations, attributes from text
7. **Context Resolution**: Match entities to detected objects in scene
8. **Ambiguity Check**: Resolve or request clarification if multiple matches
9. **Output**: Generate structured command for planning layer

---

## Key Insights

### End-to-End vs Modular Design

Modern systems balance end-to-end learning with modular pipelines:
- **End-to-end**: Single model maps audio directly to actions (simpler, but less interpretable)
- **Modular**: Separate stages allow debugging, domain adaptation (more complex, but explainable)

Most production systems use modular design for interpretability and safety.

### Confidence Propagation

Confidence scores propagate through the pipeline:
- ASR confidence affects NLU confidence
- Entity extraction confidence affects resolution
- Overall command confidence is the product of stage confidences

Low confidence at any stage should trigger clarification rather than guessing.

### Domain Adaptation

Robotics voice interfaces require domain-specific adaptation:
- Custom wake words ("Hey Robot")
- Robotics vocabulary ("gripper", "end-effector", "workspace")
- Spatial language understanding ("to my left", "behind the box")

---

## Related Concepts

- **From Module 1**: ROS 2 message types for audio and text data
- **From Module 2**: Simulated audio sources for testing voice pipelines
- **From Module 3**: GPU acceleration for Whisper inference
- **In This Module**: Intent Extraction Flow, LLM Planning Pipeline

---

## Real-World Application

Voice interfaces are increasingly deployed in industrial and service robotics:

**Amazon Astro** uses a similar pipeline for home robot commands, with Alexa providing the speech recognition and NLU, while robot-specific context resolution handles grounding to physical objects.

**Universal Robots** cobots support voice commands for industrial tasks, using constrained vocabularies and confirmation dialogues for safety-critical operations.

**Embodied Moxie** (social robot) processes natural conversation with children, requiring robust ASR for child speech patterns and context-aware dialogue management.

The key challenge in production is maintaining low latency (under 500ms from speech end to action start) while ensuring high accuracy, especially in noisy environments.
