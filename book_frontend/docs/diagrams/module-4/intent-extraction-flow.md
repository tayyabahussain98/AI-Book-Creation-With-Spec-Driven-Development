---
title: Intent Extraction Flow
module: 4
chapter: 2
type: flow
---

# Intent Extraction Flow

## Overview

This diagram illustrates the detailed process of extracting structured intents from natural language commands. It shows the pipeline from raw text through tokenization, intent classification, entity extraction, and slot filling to produce a complete command representation.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                        INTENT EXTRACTION FLOW                               │
└─────────────────────────────────────────────────────────────────────────────┘

┌──────────────────────┐
│  Input Transcript    │   "Please pick up the large red cup from the table"
└──────────┬───────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 1: PREPROCESSING                                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐        │
│  │  Normalization  │───►│  Tokenization   │───►│  Stop Word      │        │
│  │                 │    │                 │    │  Filtering      │        │
│  │ • Lowercase     │    │ • Word split    │    │ • Remove "the"  │        │
│  │ • Fix spelling  │    │ • Punctuation   │    │ • Remove "a"    │        │
│  │ • Expand abbrev │    │ • Subword units │    │ • Keep "from"   │        │
│  └─────────────────┘    └─────────────────┘    └────────┬────────┘        │
│                                                          │                  │
│  Input:  "Please pick up the large red cup from the table"                 │
│  Output: ["please", "pick", "up", "large", "red", "cup", "from", "table"]  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 2: INTENT CLASSIFICATION                                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Intent Classifier Model                           │   │
│  │                                                                      │   │
│  │   Input tokens → [Embedding] → [Encoder] → [Classification Head]    │   │
│  │                                                                      │   │
│  │   ┌──────────────────────────────────────────────────────────────┐  │   │
│  │   │  Intent Probabilities:                                       │  │   │
│  │   │                                                              │  │   │
│  │   │    PICK    ████████████████████████████████  0.87            │  │   │
│  │   │    PLACE   ████                              0.06            │  │   │
│  │   │    MOVE    ███                               0.04            │  │   │
│  │   │    FIND    █                                 0.02            │  │   │
│  │   │    SPEAK   █                                 0.01            │  │   │
│  │   └──────────────────────────────────────────────────────────────┘  │   │
│  │                                                                      │   │
│  │   Decision: Intent = PICK (confidence: 0.87)                        │   │
│  │                                                                      │   │
│  │   Confidence Threshold Check:                                       │   │
│  │   • 0.87 > 0.7 (threshold) → ACCEPT                                │   │
│  │   • If < 0.7 → Request clarification                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 3: ENTITY EXTRACTION                                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │              Named Entity Recognition (NER) with BIO Tags            │   │
│  │                                                                      │   │
│  │   Token    : please pick  up   large    red     cup     from  table │   │
│  │   BIO Tag  : O      O     O    B-SIZE   B-COLOR B-OBJ   O     B-LOC │   │
│  │                                                                      │   │
│  │   Entity Spans Extracted:                                           │   │
│  │   ┌────────────────────────────────────────────────────────────┐    │   │
│  │   │  SIZE:   "large"   span=[3,4)    confidence=0.91           │    │   │
│  │   │  COLOR:  "red"     span=[4,5)    confidence=0.94           │    │   │
│  │   │  OBJECT: "cup"     span=[5,6)    confidence=0.96           │    │   │
│  │   │  LOCATION: "table" span=[7,8)    confidence=0.89           │    │   │
│  │   └────────────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │              Entity Type Definitions for Robotics                    │   │
│  │                                                                      │   │
│  │   OBJECT    : Physical items (cup, bottle, box, ball, book)         │   │
│  │   LOCATION  : Places or surfaces (table, shelf, floor, kitchen)     │   │
│  │   PERSON    : Human references (me, John, the operator)             │   │
│  │   COLOR     : Visual attributes (red, blue, green, transparent)     │   │
│  │   SIZE      : Dimensional attributes (large, small, tall, wide)     │   │
│  │   QUANTITY  : Numerical amounts (one, two, three, all, some)        │   │
│  │   DIRECTION : Spatial directions (left, right, up, down, forward)   │   │
│  │   CONTAINER : Holding objects (in the box, inside the drawer)       │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 4: SLOT FILLING                                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Intent Frame Template                             │   │
│  │                                                                      │   │
│  │   PICK Intent Slots:                                                │   │
│  │   ┌────────────────────────────────────────────────────────────┐    │   │
│  │   │  Slot Name     │ Required │ Filled Value    │ Status       │    │   │
│  │   ├────────────────┼──────────┼─────────────────┼──────────────┤    │   │
│  │   │  object        │    ✓     │ "cup"           │ ✓ Filled     │    │   │
│  │   │  object_color  │          │ "red"           │ ✓ Filled     │    │   │
│  │   │  object_size   │          │ "large"         │ ✓ Filled     │    │   │
│  │   │  source_loc    │          │ "table"         │ ✓ Filled     │    │   │
│  │   │  destination   │          │ (not specified) │ ○ Empty      │    │   │
│  │   │  quantity      │          │ 1 (default)     │ ○ Default    │    │   │
│  │   └────────────────────────────────────────────────────────────┘    │   │
│  │                                                                      │   │
│  │   Slot Filling Status: 4/6 filled (all required slots filled ✓)     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Slot Inference Rules                              │   │
│  │                                                                      │   │
│  │   Rule 1: If no quantity specified → default to 1                   │   │
│  │   Rule 2: If PICK with no destination → hold in gripper             │   │
│  │   Rule 3: If color without object → apply to nearest OBJECT entity  │   │
│  │   Rule 4: Combine adjacent attributes → "large red cup" = one object│   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ STAGE 5: VALIDATION & DISAMBIGUATION                                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Semantic Validation Checks                        │   │
│  │                                                                      │   │
│  │   ✓ Intent-Entity Compatibility                                     │   │
│  │     PICK requires: object (✓), optional: source, destination        │   │
│  │                                                                      │   │
│  │   ✓ Entity Consistency                                              │   │
│  │     No conflicting attributes (e.g., "large small cup")             │   │
│  │                                                                      │   │
│  │   ✓ Physical Plausibility                                           │   │
│  │     Object type can be picked (cup ✓, not "floor" ✗)                │   │
│  │                                                                      │   │
│  │   ✓ Reference Resolution Possible                                   │   │
│  │     Description specific enough for grounding                       │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    Disambiguation Triggers                           │   │
│  │                                                                      │   │
│  │   Trigger Conditions:                                               │   │
│  │   • Multiple objects match description → "Which cup? I see 3 cups"  │   │
│  │   • Low confidence on required slot → "Did you say cup or cap?"     │   │
│  │   • Missing required slot → "Where should I place it?"              │   │
│  │   • Conflicting constraints → "Do you mean the large or small one?" │   │
│  │                                                                      │   │
│  │   Current Status: No disambiguation needed ✓                        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ OUTPUT: STRUCTURED INTENT                                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  {                                                                          │
│    "intent": "PICK",                                                       │
│    "confidence": 0.87,                                                     │
│    "entities": {                                                           │
│      "object": {                                                           │
│        "value": "cup",                                                     │
│        "attributes": {                                                     │
│          "color": "red",                                                   │
│          "size": "large"                                                   │
│        },                                                                  │
│        "confidence": 0.96                                                  │
│      },                                                                    │
│      "source_location": {                                                  │
│        "value": "table",                                                   │
│        "confidence": 0.89                                                  │
│      }                                                                     │
│    },                                                                      │
│    "original_text": "Please pick up the large red cup from the table",    │
│    "requires_disambiguation": false                                        │
│  }                                                                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌──────────────────┐
│  To: Context     │
│      Resolution  │
└──────────────────┘
```

---

## Component Descriptions

### Stage 1: Preprocessing

**Normalization**
- **Purpose**: Standardize input text for consistent processing
- **Inputs**: Raw transcript text
- **Outputs**: Normalized text
- **Key Details**: Lowercasing, spelling correction, abbreviation expansion

**Tokenization**
- **Purpose**: Split text into processing units
- **Inputs**: Normalized text
- **Outputs**: Token sequence
- **Key Details**: Word-level or subword tokenization (BPE, WordPiece)

**Stop Word Filtering**
- **Purpose**: Remove low-information words selectively
- **Inputs**: Token sequence
- **Outputs**: Filtered tokens
- **Key Details**: Preserve spatial prepositions like "from", "on", "under"

### Stage 2: Intent Classification

**Intent Classifier Model**
- **Purpose**: Determine the action type requested
- **Inputs**: Token embeddings
- **Outputs**: Intent label with probability
- **Key Details**: Fine-tuned transformer or specialized classifier

**Confidence Thresholding**
- **Purpose**: Gate uncertain classifications for clarification
- **Inputs**: Intent probabilities
- **Outputs**: Accept/reject decision
- **Key Details**: Threshold typically 0.7-0.8 for robotics safety

### Stage 3: Entity Extraction

**Named Entity Recognition**
- **Purpose**: Identify and label entity spans in text
- **Inputs**: Tokens with intent context
- **Outputs**: BIO-tagged sequence with entity types
- **Key Details**: Joint intent-entity models improve accuracy

**Entity Type Definitions**
- **Purpose**: Define domain-specific entity categories
- **Inputs**: Task requirements
- **Outputs**: Entity taxonomy
- **Key Details**: Robotics requires spatial and physical attributes

### Stage 4: Slot Filling

**Intent Frame Template**
- **Purpose**: Define expected slots for each intent type
- **Inputs**: Intent type
- **Outputs**: Slot template with requirements
- **Key Details**: Required vs optional slots, default values

**Slot Inference Rules**
- **Purpose**: Fill slots from entities with domain knowledge
- **Inputs**: Extracted entities, slot template
- **Outputs**: Filled slot values
- **Key Details**: Handles implicit defaults and entity grouping

### Stage 5: Validation & Disambiguation

**Semantic Validation**
- **Purpose**: Check logical consistency of filled intent
- **Inputs**: Complete intent frame
- **Outputs**: Validation result
- **Key Details**: Intent-entity compatibility, physical plausibility

**Disambiguation Triggers**
- **Purpose**: Identify when clarification is needed
- **Inputs**: Validation result, slot confidence
- **Outputs**: Clarification questions if needed
- **Key Details**: Better to ask than execute wrong action

---

## Data Flow

1. **Text Input**: Receive transcript from speech recognition
2. **Preprocessing**: Normalize, tokenize, filter input
3. **Intent Classification**: Determine action type with confidence
4. **Entity Extraction**: Identify objects, locations, attributes
5. **Slot Filling**: Map entities to intent frame slots
6. **Validation**: Check semantic consistency and completeness
7. **Disambiguation**: Generate clarification if needed
8. **Output**: Produce structured intent for context resolution

---

## Key Insights

### Joint vs Pipeline Models

Two approaches exist for intent extraction:

**Pipeline**: Separate models for intent and entities
- Easier to debug and maintain
- Each component can be updated independently
- Error propagation between stages

**Joint**: Single model for both
- Better performance (shared representations)
- Handles intent-entity dependencies
- Harder to interpret failures

Most production systems use joint models with pipeline fallbacks.

### Domain Adaptation is Critical

Generic NLU models fail on robotics commands:
- "Pick" vs "PICK UP" (action)
- "Table" vs "TABLE 3" (specific vs category)
- Spatial language unique to robotics

Fine-tuning on robotics corpora significantly improves accuracy.

### Graceful Degradation

When extraction fails, systems should:
1. Request clarification rather than guess
2. Offer specific options ("Did you mean X or Y?")
3. Allow partial commands ("Pick up... which object?")
4. Support correction ("No, the other one")

---

## Related Concepts

- **From Module 1**: ROS 2 message types for command representation
- **From Module 2**: Simulated dialogue for training data generation
- **From Module 3**: GPU acceleration for transformer inference
- **In This Module**: Voice Pipeline, Context Resolution

---

## Real-World Application

Intent extraction powers voice interfaces across robotics:

**Amazon Alexa Skills** for smart home devices use similar slot-filling approaches, with custom slot types for device names and room locations.

**Industrial Voice Interfaces** (e.g., Honeywell Vocollect) use constrained grammars for reliability, trading flexibility for accuracy in noisy warehouse environments.

**Healthcare Robots** (e.g., Moxi) implement multi-turn dialogue for intent clarification, ensuring critical commands are correctly understood before execution.

The key production insight is that robotics intent extraction must be more conservative than consumer applications—the cost of misunderstanding is physical action, not just wrong information.
