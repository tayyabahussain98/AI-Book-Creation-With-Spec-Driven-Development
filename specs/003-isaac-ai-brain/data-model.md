# Data Model: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Feature**: 003-isaac-ai-brain
**Date**: 2025-12-23
**Purpose**: Define entities, relationships, and validation rules for Module 3 educational content

## Overview

Module 3 consists of educational content entities (chapters, diagrams, code examples) that teach NVIDIA Isaac platform concepts for AI-powered robotics. This data model describes the structure and relationships of these content elements without implementation details.

---

## Core Entities

### 1. Chapter

**Description**: A standalone educational content unit teaching specific Isaac/Nav2/RL concepts.

**Attributes**:
- `chapter_id`: Integer (1-5), maps to `sidebar_position`
- `title`: String, descriptive chapter name
- `slug`: String, URL-friendly identifier (e.g., `chapter-1-isaac-overview`)
- `file_path`: String, absolute path to markdown file
- `word_count`: Integer, target 2,000-2,500 words
- `prerequisites`: List[String], concepts from prior modules/chapters
- `learning_objectives`: List[String], 3-4 measurable outcomes
- `core_concepts`: List[CoreConcept], 4-5 main sections
- `diagram_references`: List[String], diagram slugs referenced
- `code_example_references`: List[String], example file names (if any)
- `summary`: List[String], 6 bullet points
- `self_assessment`: List[String], 6 reflection questions

**Relationships**:
- Belongs to: Module 3 (one-to-many from Module to Chapter)
- References: Diagrams (many-to-many), Code Examples (many-to-many)
- Depends on: Prerequisites from Modules 1-2

**Validation Rules**:
- `chapter_id` must be unique within module (1-5)
- `word_count` must be between 1,800-2,800 words (±20% tolerance)
- `learning_objectives` must have exactly 3-4 items
- `summary` must have exactly 6 bullets
- `self_assessment` must have exactly 6 items
- All `diagram_references` must exist in diagrams/module-3/
- All `code_example_references` must exist in code-examples/module-3/

**State Transitions**:
- Draft → In Review → Approved → Published
- Draft: Initial writing, incomplete
- In Review: Content complete, validation checks pending
- Approved: All validation rules pass, ready for build
- Published: Deployed to GitHub Pages

---

### 2. Diagram Specification

**Description**: Text-based specification describing a visual concept for Module 3 content.

**Attributes**:
- `diagram_id`: String, unique identifier (slug)
- `title`: String, descriptive diagram name
- `file_path`: String, absolute path to diagram markdown file
- `diagram_type`: Enum (Architecture, DataFlow, DecisionTree, ComparisonTable, ProcessWorkflow)
- `purpose`: String, why this diagram exists
- `ascii_art`: String (optional), text-based visual representation
- `detailed_explanation`: String, 300-500 word description
- `usage_context`: String, which chapters reference this diagram
- `related_diagrams`: List[String], diagram slugs with conceptual relationships

**Relationships**:
- Belongs to: Module 3 (one-to-many from Module to Diagram)
- Referenced by: Chapters (many-to-many)

**Validation Rules**:
- `diagram_id` must be unique within module
- `diagram_type` must be valid enum value
- `purpose` must be 50-150 words
- `detailed_explanation` must be 300-500 words
- `usage_context` must reference at least one chapter
- All `related_diagrams` must exist in diagrams/module-3/

**Diagram Type Descriptions**:
- **Architecture**: Component relationships, system structure (e.g., Isaac ROS nodes, Nav2 stack)
- **DataFlow**: Information pipelines, data transformations (e.g., synthetic data pipeline)
- **DecisionTree**: When to use X vs Y, conditional logic (e.g., Isaac Sim vs Gazebo)
- **ComparisonTable**: Feature matrices, side-by-side comparisons (e.g., CPU vs GPU acceleration)
- **ProcessWorkflow**: Step-by-step procedures (e.g., domain randomization workflow)

---

### 3. Code Example

**Description**: Minimal conceptual code snippet illustrating Isaac ROS, Nav2, or RL concepts.

**Attributes**:
- `example_id`: String, unique identifier (filename without extension)
- `filename`: String, with extension (e.g., `isaac_ros_node_structure.py`)
- `file_path`: String, absolute path to example file
- `language`: Enum (Python, YAML, XML)
- `purpose`: String, what concept this illustrates
- `line_count`: Integer, target 20-50 lines (minimal)
- `concepts_demonstrated`: List[String], key concepts shown
- `referenced_by_chapters`: List[Integer], chapter IDs that embed this example
- `is_runnable`: Boolean, whether example is syntax-valid (False for pseudocode)
- `annotations`: String, inline comments explaining key concepts

**Relationships**:
- Belongs to: Module 3 (one-to-many from Module to Code Example)
- Referenced by: Chapters (many-to-many)

**Validation Rules**:
- `example_id` must be unique within module
- `line_count` must be between 15-70 lines (±30% tolerance for clarity)
- `concepts_demonstrated` must have at least 2 items
- `referenced_by_chapters` must have at least 1 item
- If `is_runnable` is True, syntax must be valid for `language`
- `annotations` must appear every 5-10 lines for clarity

**Code Example Types**:
1. **Conceptual Structure**: Isaac ROS node skeleton (Python)
2. **Configuration**: Nav2 params for humanoid (YAML)
3. **Process Illustration**: Domain randomization config (Python pseudocode)

---

### 4. Learning Path

**Description**: Sequential progression through Module 3 chapters building AI robotics knowledge.

**Attributes**:
- `module_id`: Integer (3 for this module)
- `module_title`: String ("Module 3 - The AI-Robot Brain (NVIDIA Isaac)")
- `chapter_sequence`: List[Integer], ordered chapter IDs [1, 2, 3, 4, 5]
- `estimated_duration`: String, time to complete ("5 weeks" at 1 chapter/week)
- `prerequisites`: List[String], required prior knowledge
- `learning_outcomes`: List[String], module-level outcomes
- `sidebar_position`: Integer, position in Docusaurus sidebar (3, after Module 2)

**Relationships**:
- Contains: Chapters (one-to-many)
- Follows: Module 2 (prerequisite)
- Prepares for: Module 4+ (VLA, humanoid autonomy)

**Validation Rules**:
- `chapter_sequence` must include all chapter IDs without duplicates
- `prerequisites` must reference existing modules (Modules 1-2)
- `learning_outcomes` must align with chapter-level learning objectives
- `sidebar_position` must not conflict with other modules

---

### 5. Self-Assessment Item

**Description**: Reflection question for learner self-evaluation.

**Attributes**:
- `item_id`: String, unique identifier within chapter
- `chapter_id`: Integer, parent chapter
- `question_text`: String, reflection prompt
- `question_type`: Enum (Explanation, DecisionScenario, Recall, Application)
- `expected_response_format`: String, how learner should answer
- `concepts_assessed`: List[String], concepts tested by this question

**Relationships**:
- Belongs to: Chapter (many-to-one, 6 items per chapter)

**Validation Rules**:
- Each chapter must have exactly 6 self-assessment items
- `question_text` must end with "?" (interrogative)
- `question_type` must be valid enum value
- `concepts_assessed` must have at least 1 item
- No graded answers (reflection only, no correct/incorrect)

**Question Type Descriptions**:
- **Explanation**: "Can you explain..." (understanding)
- **DecisionScenario**: "Can you identify when..." (application)
- **Recall**: "Can you list 3+ examples of..." (knowledge)
- **Application**: "How would you apply..." (transfer)

---

## Entity Relationships

```
Module 3 (1)
  ├──> Chapters (5)
  │     ├──> Core Concepts (4-5 each)
  │     ├──> References Diagrams (many-to-many)
  │     ├──> References Code Examples (many-to-many)
  │     └──> Self-Assessment Items (6 each)
  ├──> Diagrams (10-12)
  │     └──> Referenced by Chapters (many-to-many)
  └──> Code Examples (0-3)
        └──> Referenced by Chapters (many-to-many)

Prerequisites:
  Module 1 (ROS 2) ──> Module 3
  Module 2 (Digital Twin) ──> Module 3

Prepares for:
  Module 3 ──> Module 4+ (VLA, Humanoid Autonomy)
```

---

## Chapter-Specific Entities

### Chapter 1: NVIDIA Isaac Platform Overview

**Core Concepts**:
1. Isaac SDK vs Isaac Sim vs Isaac ROS distinctions
2. Omniverse role in robotics AI (USD, RTX rendering)
3. Tool selection criteria (Isaac Sim vs Gazebo vs Unity)
4. ROS 2 integration architecture

**Diagrams Referenced**:
- `isaac-platform-ecosystem.md` (Architecture)
- `isaac-ros-architecture.md` (Architecture)

**Code Examples**: None (conceptual chapter)

---

### Chapter 2: Isaac Sim & Synthetic Data

**Core Concepts**:
1. Photorealistic simulation and RTX rendering
2. Synthetic data generation workflow (scene setup → rendering → annotation)
3. Domain randomization techniques (lighting, textures, physics, camera)
4. Perception dataset formats (COCO, KITTI, custom)

**Diagrams Referenced**:
- `synthetic-data-pipeline.md` (ProcessWorkflow)
- `domain-randomization-params.md` (ComparisonTable)

**Code Examples**: Potentially `domain_randomization_config.py` (if needed)

---

### Chapter 3: Isaac ROS & Accelerated Perception

**Core Concepts**:
1. Hardware-accelerated ROS nodes (image_proc, dnn_inference, visual_slam)
2. GPU acceleration benefits and use cases
3. Visual SLAM architecture (cuVSLAM)
4. Sensor fusion strategies
5. CPU vs GPU trade-offs

**Diagrams Referenced**:
- `isaac-ros-architecture.md` (Architecture)
- `gpu-acceleration-flow.md` (DataFlow)
- `visual-slam-pipeline.md` (ProcessWorkflow)

**Code Examples**: `isaac_ros_node_structure.py` (conceptual node example)

---

### Chapter 4: Navigation with Nav2

**Core Concepts**:
1. Nav2 architecture (SLAM, localization, planners, costmaps)
2. Humanoid-specific navigation (ZMP stability, footstep planning)
3. Dynamic obstacle avoidance and replanning
4. Costmap configuration for bipedal robots

**Diagrams Referenced**:
- `nav2-architecture.md` (Architecture)
- `costmap-layers.md` (DataFlow)
- `humanoid-footstep-planning.md` (ProcessWorkflow)

**Code Examples**: `nav2_params_humanoid.yaml` (Nav2 config snippet)

---

### Chapter 5: Training & Sim-to-Real Transfer

**Core Concepts**:
1. Reinforcement learning fundamentals (state, action, reward, policy)
2. Domain randomization for sim-to-real transfer
3. Reality gap: sources and mitigation strategies
4. Safe deployment principles (action limits, emergency stops, validation)

**Diagrams Referenced**:
- `rl-training-loop.md` (ProcessWorkflow)
- `domain-randomization-params.md` (ComparisonTable, reused from Chapter 2)
- `sim-to-real-workflow.md` (ProcessWorkflow)
- `reality-gap-mitigation.md` (DecisionTree)

**Code Examples**: None (conceptual RL, no implementation)

---

## Diagram Specifications Summary

| Diagram ID | Type | Purpose | Referenced By |
|------------|------|---------|---------------|
| `isaac-platform-ecosystem` | Architecture | Show Isaac SDK/Sim/ROS/Omniverse relationships | Chapter 1 |
| `synthetic-data-pipeline` | ProcessWorkflow | Illustrate scene → randomization → annotation → export | Chapter 2 |
| `isaac-ros-architecture` | Architecture | Depict Isaac ROS packages and GPU acceleration | Chapters 1, 3 |
| `gpu-acceleration-flow` | DataFlow | Show data flow through GPU-accelerated nodes | Chapter 3 |
| `visual-slam-pipeline` | ProcessWorkflow | Explain cuVSLAM feature tracking and mapping | Chapter 3 |
| `nav2-architecture` | Architecture | Nav2 components (SLAM, localization, planners, costmaps) | Chapter 4 |
| `costmap-layers` | DataFlow | Costmap layer composition (static, obstacle, inflation) | Chapter 4 |
| `humanoid-footstep-planning` | ProcessWorkflow | Footstep sequence generation with ZMP constraints | Chapter 4 |
| `rl-training-loop` | ProcessWorkflow | RL training cycle (collect → update → repeat) | Chapter 5 |
| `domain-randomization-params` | ComparisonTable | DR parameter categories (object, env, sensor, physics) | Chapters 2, 5 |
| `sim-to-real-workflow` | ProcessWorkflow | Sim training → validation → real deployment | Chapter 5 |
| `reality-gap-mitigation` | DecisionTree | When to use DR vs sim tuning vs safety margins | Chapter 5 |

**Total Diagrams**: 12

---

## Code Examples Summary

| Example ID | Filename | Language | Purpose | Referenced By |
|------------|----------|----------|---------|---------------|
| `isaac_ros_node_structure` | `isaac_ros_node_structure.py` | Python | Illustrate Isaac ROS node structure (conceptual) | Chapter 3 |
| `nav2_params_humanoid` | `nav2_params_humanoid.yaml` | YAML | Show Nav2 config for humanoid robot (footprint, speeds) | Chapter 4 |
| `domain_randomization_config` | `domain_randomization_config.py` | Python | Example DR config (if needed, optional) | Chapter 2 |

**Total Code Examples**: 2-3 (minimal, conceptual)

---

## Validation Checklist

Before implementation (/sp.tasks), verify:

- [ ] All 5 chapters defined with complete attributes
- [ ] All 12 diagrams specified with clear purposes
- [ ] All 2-3 code examples defined (if used)
- [ ] Chapter prerequisites clearly stated
- [ ] Learning objectives align with success criteria from spec.md
- [ ] Diagram references in chapters match diagram IDs
- [ ] Self-assessment items are reflection-based (no grading)
- [ ] No implementation details in entity descriptions
- [ ] All relationships between entities documented
- [ ] Entity counts match plan.md estimates

---

## Notes

- **Concept-first approach**: Code examples are illustrative, not executable tutorials
- **Diagram specifications**: Text-based descriptions with ASCII art, not rendered images
- **Self-assessments**: Reflection prompts for learner self-evaluation, not tests
- **Prerequisites**: Modules 1-2 knowledge assumed, explicitly referenced where needed
- **External resources**: NVIDIA docs, Nav2 docs, RL tutorials linked but not embedded
- **Consistency**: Entity structure mirrors Module 2 data model for UX consistency
