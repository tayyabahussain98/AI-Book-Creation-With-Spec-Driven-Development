# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-module
**Date**: 2025-12-22
**Purpose**: Define structure of educational content entities (chapters, diagrams, examples) for Module 2

## Entity Overview

This module does not involve application data (no database, no user accounts). Instead, the "entities" are educational content artifacts that must be created and maintained.

## Content Entities

### 1. Chapter

**Purpose**: Educational markdown file teaching a specific aspect of digital twin simulation

**Attributes**:
- **chapter_number**: Integer (1-5)
- **title**: String (e.g., "Digital Twins & Simulation Foundations")
- **sidebar_position**: Integer (determines navigation order)
- **file_path**: String (e.g., "book_frontend/docs/module-2-digital-twin/chapter-1-foundations.md")
- **prerequisites**: List of prerequisite chapters (e.g., ["Module 1 Chapter 5"])
- **learning_objectives**: List of 3-5 objectives (e.g., "Understand digital twin concept")
- **word_count**: Integer (target: 3000-4000 words, ~600-800 lines)

**Structure** (consistent across all chapters):
1. Front matter (YAML: sidebar_position, title)
2. Introduction (~200 words: prerequisites, objectives, motivation)
3. Core Concepts (3-5 sections, ~500-800 words each)
4. Hands-On Example (~400-600 words with embedded code)
5. Diagrams section (references to 2-3 diagram files)
6. Summary (6 bullet points, ~150 words)
7. Self-Assessment Checklist (6 items)

**Validation Rules**:
- Must have front matter with sidebar_position and title
- Must include at least 3 core concepts
- Must reference at least 1 diagram
- Must end with 6-bullet summary and 6-item checklist
- Code examples must be annotated (no raw code dumps)

**Relationships**:
- **References** 2-3 Diagram entities
- **Includes** 0-2 Code Example entities (embedded)
- **Follows** previous Chapter (sequential navigation)
- **Prerequisites** Module 1 chapters (dependency chain)

---

### 2. Diagram Specification

**Purpose**: Text-based diagram specification (ASCII art, tables, workflow descriptions) that illustrates technical concepts

**Attributes**:
- **diagram_id**: String (e.g., "digital-twin-lifecycle")
- **file_path**: String (e.g., "book_frontend/docs/diagrams/module-2/digital-twin-lifecycle.md")
- **diagram_type**: Enum (workflow, architecture, comparison_table, data_flow, state_machine)
- **referenced_by**: List of chapter file paths (which chapters link to this diagram)
- **visual_format**: Enum (ascii_art, markdown_table, text_specification)

**Structure**:
```markdown
# [Diagram Title] Specification

**Purpose**: [What this diagram illustrates]
**Diagram Type**: [workflow/architecture/comparison/etc.]

## [Visual Representation]

[ASCII art, table, or text-based diagram]

## [Explanatory Sections]

[Detailed explanation of diagram components]

## Usage in Book

- **Referenced in**: [Chapter X, Section Y]
- **Purpose**: [Why learners need this visualization]
- **Learning Goal**: [What learners should extract from diagram]
```

**Validation Rules**:
- Must have clear purpose statement
- Must be referenced by at least one chapter
- Visual representation must be text-only (no image files)
- Must include explanatory sections with context

**Relationships**:
- **Referenced by** 1-N Chapters
- **Illustrates** concepts from Functional Requirements

---

### 3. Code Example

**Purpose**: Runnable configuration file (SDF, world, sensor config) demonstrating simulation concepts

**Attributes**:
- **example_id**: String (e.g., "simple_world")
- **file_path**: String (e.g., "book_frontend/docs/code-examples/module-2/simple_world.sdf")
- **file_type**: Enum (sdf_model, sdf_world, sensor_config, unity_config)
- **purpose**: String (e.g., "Demonstrates basic Gazebo world structure")
- **embedded_in**: List of chapter file paths (which chapters embed this example)
- **runnable**: Boolean (true = can be executed as-is, false = snippet only)
- **lines_of_code**: Integer (target: 50-200 lines for readability)

**Structure**:
```xml
<?xml version="1.0"?>
<!--
  [Example Name]

  Purpose: [What this example demonstrates]

  Usage:
  gz sim [filename]

  Key Concepts:
  - [Concept 1]
  - [Concept 2]
-->

<sdf version="1.9">
  <!-- Heavily annotated SDF/XML content -->
  <world name="example">
    <!-- Each element explained with comments -->
  </world>
</sdf>
```

**Validation Rules**:
- Must be syntactically valid XML/SDF
- Must include header comment with purpose and usage
- Must have inline annotations explaining key elements
- If runnable=true, must execute without errors in Gazebo Harmonic

**Relationships**:
- **Embedded in** 1-N Chapters (inline code blocks)
- **Demonstrates** concepts from Functional Requirements

---

### 4. Module

**Purpose**: High-level organizational unit grouping related chapters

**Attributes**:
- **module_number**: Integer (2 for this feature)
- **title**: String ("The Digital Twin (Gazebo & Unity)")
- **chapter_count**: Integer (5)
- **sidebar_label**: String (used in sidebars.ts configuration)
- **completion_time**: String ("5 weeks, 1 chapter/week")

**Relationships**:
- **Contains** 5 Chapter entities
- **Follows** Module 1 (prerequisite dependency)
- **Listed in** Sidebar configuration

---

## Entity Relationships

```
Module 2
├── Chapter 1
│   ├── References: digital-twin-lifecycle.md
│   ├── References: gazebo-architecture.md
│   └── Includes: (no code examples)
├── Chapter 2
│   ├── References: sdf-world-structure.md
│   ├── References: gazebo-physics-engine.md
│   └── Includes: simple_world.sdf (embedded)
├── Chapter 3
│   ├── References: urdf-vs-sdf-comparison.md
│   ├── References: sdf-conversion-workflow.md
│   └── Includes: humanoid_stable.sdf (embedded)
├── Chapter 4
│   ├── References: sensor-plugin-flow.md
│   ├── References: lidar-raycasting.md
│   ├── References: depth-camera-rendering.md
│   ├── Includes: lidar_robot.sdf (embedded)
│   └── Includes: depth_camera_robot.sdf (embedded)
└── Chapter 5
    ├── References: unity-ros-bridge.md
    ├── References: hri-interaction-patterns.md
    └── Includes: unity_bridge_config.xml (embedded)
```

## Content Templates

### Chapter Template Structure

```markdown
---
sidebar_position: [1-5]
title: Chapter [X] - [Title]
---

# Chapter [X]: [Title]

## Introduction
[Prerequisites, learning objectives, motivation]

## Core Concept 1: [Topic]
[Conceptual explanation, terminology, examples]

## Core Concept 2: [Topic]
[Mechanism, workflow, best practices]

## Core Concept 3: [Topic]
[Advanced topic, integration, considerations]

## Hands-On Example: [Example Name]
[Embedded code with heavy annotation, running instructions]

## Diagrams
### [Diagram 1 Name]
See the [Diagram Link](/docs/diagrams/module-2/filename.md) for...

## Summary
1. [Key point 1]
2. [Key point 2]
...
6. [Key point 6]

## Self-Assessment Checklist
- [ ] I can explain [concept 1]
- [ ] I can demonstrate [skill 2]
...
- [ ] I can distinguish [concept 6]
```

### Diagram Template Structure

```markdown
# [Diagram Title] Specification

**Purpose**: [One-sentence description]
**Diagram Type**: [workflow/architecture/comparison/data_flow]

## [Visual Section]
[ASCII art, table, or structured text representation]

## [Explanation Section]
[Detailed breakdown of diagram components]

## Usage in Book
- **Referenced in**: Chapter [X] (Section Y)
- **Purpose**: [Why learners need this]
- **Learning Goal**: [What learners extract]
```

## Validation Criteria

### Chapter Validation

Each chapter must pass:
- [ ] Markdown syntax valid (builds without errors)
- [ ] Front matter present with sidebar_position and title
- [ ] At least 3 core concepts present
- [ ] At least 1 diagram referenced
- [ ] 6-bullet summary present
- [ ] 6-item self-assessment checklist present
- [ ] No broken internal links
- [ ] Code examples (if present) are runnable or clearly marked as snippets

### Diagram Validation

Each diagram must pass:
- [ ] Purpose statement present
- [ ] Referenced by at least one chapter
- [ ] Visual representation clear and text-based
- [ ] Explanatory content provided
- [ ] No image file dependencies

### Example Validation

Each code example must pass:
- [ ] XML/SDF syntax valid
- [ ] Header comment with purpose and usage
- [ ] Inline annotations explaining key elements
- [ ] Referenced or embedded in at least one chapter

## Content Inventory (To Be Created)

### Chapters (5 total)
1. chapter-1-foundations.md (~700 lines)
2. chapter-2-gazebo-physics.md (~750 lines)
3. chapter-3-robot-models.md (~650 lines)
4. chapter-4-sensor-simulation.md (~800 lines)
5. chapter-5-unity-hri.md (~600 lines)

**Total**: ~3,500 lines of educational content

### Diagrams (10 total)
1. digital-twin-lifecycle.md
2. gazebo-architecture.md
3. urdf-vs-sdf-comparison.md
4. sdf-world-structure.md
5. sensor-plugin-flow.md
6. lidar-raycasting.md
7. depth-camera-rendering.md
8. imu-noise-model.md
9. unity-ros-bridge.md
10. hri-interaction-patterns.md

### Code Examples (6 total)
1. simple_world.sdf (~100 lines)
2. humanoid_stable.sdf (~200 lines)
3. obstacles_world.sdf (~150 lines)
4. lidar_robot.sdf (~180 lines)
5. depth_camera_robot.sdf (~170 lines)
6. imu_config.sdf (~120 lines)

**Total**: ~920 lines of example code

### Configuration Updates (2 files)
1. book_frontend/sidebars.ts (add Module 2 category)
2. book_frontend/docs/intro.md (update with Module 2 description - optional)

## Dependencies Between Entities

**Creation Order** (for implementation):
1. Diagram specifications first (referenced by chapters)
2. Code examples second (embedded in chapters)
3. Chapters third (consume diagrams and examples)
4. Sidebar configuration last (links to chapters)

**Linking Rules**:
- Chapters reference diagrams: `/docs/diagrams/module-2/filename.md`
- Chapters embed code examples: markdown code blocks with file content
- Sidebar references chapters: `module-2-digital-twin/chapter-X-slug`
