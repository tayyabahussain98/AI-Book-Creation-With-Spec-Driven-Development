# Data Model: Module 1 Content Structure

**Date**: 2025-12-21
**Feature**: 001-ros2-module
**Phase**: 1 (Design)

## Overview

This document defines the structure of Module 1 educational content, including chapter organization, code example schemas, and diagram specifications. All content stored as Markdown files in `book_frontend/docs/module-1-ros2/`.

---

## Entity: Chapter

Represents one week of instruction (5 total for Module 1).

### Attributes

| Attribute | Type | Description | Required |
|-----------|------|-------------|----------|
| `chapter_number` | Integer | Week number (1-5) | Yes |
| `title` | String | Chapter title | Yes |
| `learning_goals` | List[String] | 3-5 outcomes learner should achieve | Yes |
| `introduction` | Markdown | 1-2 paragraph overview | Yes |
| `core_concepts` | List[ConceptSection] | 3-5 conceptual sections | Yes |
| `diagrams` | List[DiagramSpec] | 1-2 architecture/flow diagrams | Yes |
| `code_examples` | List[CodeExample] | 0-3 runnable examples (Ch3-5 only) | Conditional |
| `summary` | List[String] | Bullet list of key takeaways | Yes |
| `checklist` | List[ChecklistItem] | 5-7 self-assessment items | Yes |

### Relationships

- Chapter `contains` multiple ConceptSections
- Chapter `references` multiple CodeExamples
- Chapter `includes` multiple DiagramSpecs
- Chapters are `ordered` sequentially (1 → 2 → 3 → 4 → 5)

### Validation Rules

1. Chapter numbers must be unique and sequential (1-5)
2. Each chapter must have at least 3 core concepts
3. Chapters 3-5 must include at least 1 code example
4. All ROS 2 terminology must be defined before first use (FR-003)
5. Checklist must have 5-7 items (per user story acceptance criteria)

### File Naming Convention

```
chapter-{number}-{slug}.md
```

**Examples**:
- `chapter-1-intro-physical-ai.md`
- `chapter-2-ros2-architecture.md`
- `chapter-3-python-nodes.md`

---

## Entity: ConceptSection

Represents a single conceptual topic within a chapter.

### Attributes

| Attribute | Type | Description | Required |
|-----------|------|-------------|----------|
| `title` | String | Concept name (e.g., "Nodes and Executors") | Yes |
| `explanation` | Markdown | 2-4 paragraphs explaining concept | Yes |
| `analogies` | List[String] | Real-world comparisons (optional) | No |
| `terminology` | Dict[String, String] | Key terms and definitions | Yes |
| `common_mistakes` | List[String] | Pitfalls to avoid (optional) | No |

### Example

```markdown
## Concept: ROS 2 Topics

Topics are named channels for asynchronous message passing. Think of topics like radio frequencies: publishers broadcast messages on a frequency, and subscribers tune in to listen. Multiple nodes can publish or subscribe to the same topic without knowing about each other.

**Key Terminology**:
- **Publisher**: Node that sends messages to a topic
- **Subscriber**: Node that receives messages from a topic
- **Message Type**: Data structure format (e.g., `std_msgs/String`, `sensor_msgs/JointState`)

**Common Mistake**: Forgetting that topics are many-to-many (multiple publishers + multiple subscribers OK).
```

---

## Entity: CodeExample

Represents a runnable Python rclpy code snippet.

### Attributes

| Attribute | Type | Description | Required |
|-----------|------|-------------|----------|
| `filename` | String | Example file name (e.g., `publisher_example.py`) | Yes |
| `description` | String | One-sentence purpose | Yes |
| `ros2_concept` | String | What ROS 2 feature it demonstrates | Yes |
| `target_version` | String | ROS 2 distribution (Humble or Iron) | Yes |
| `code` | String | Full Python code with comments | Yes |
| `run_command` | String | Bash command to execute | Yes |
| `expected_output` | String | What learner should see | Yes |
| `prerequisites` | List[String] | Required ROS 2 packages/tools | No |

### Schema

```python
{
  "filename": "publisher_example.py",
  "description": "Publishes mock joint angles at 10Hz",
  "ros2_concept": "Publishers and Topics",
  "target_version": "Humble/Iron",
  "code": "#!/usr/bin/env python3\n...",
  "run_command": "python3 publisher_example.py",
  "expected_output": "Publishing angle: 0.00\nPublishing angle: 0.10\n...",
  "prerequisites": ["rclpy", "std_msgs"]
}
```

### Validation Rules

1. All examples must be executable without modification (FR-004, SC-008)
2. Must include shebang (`#!/usr/bin/env python3`)
3. Must have docstring with purpose and target version
4. Must include inline comments for non-obvious code
5. Must demonstrate exactly ONE ROS 2 concept per example

### File Location

```
book_frontend/docs/code-examples/module-1/{filename}
```

---

## Entity: DiagramSpec

Represents a textual description of an architecture or flow diagram.

### Attributes

| Attribute | Type | Description | Required |
|-----------|------|-------------|----------|
| `diagram_id` | String | Unique identifier (e.g., `ros2-signal-flow`) | Yes |
| `title` | String | Diagram title | Yes |
| `purpose` | String | What the diagram illustrates | Yes |
| `components` | List[Component] | Boxes, nodes, entities in diagram | Yes |
| `connections` | List[Connection] | Arrows, flows between components | Yes |
| `labels` | Dict[String, String] | Legend explaining visual elements | Yes |
| `notes` | List[String] | Additional context or caveats | No |

### Component Schema

```python
{
  "name": "AI Agent Node",
  "type": "software_node",  # software_node, hardware, middleware, data
  "description": "Decides robot actions based on sensor input",
  "shape": "rectangle"  # rectangle, rounded_rectangle, ellipse, cylinder
}
```

### Connection Schema

```python
{
  "from": "AI Agent Node",
  "to": "Motor Controller Node",
  "type": "data_flow",  # data_flow, control_flow, dependency
  "label": "Publish(topic=/arm_command)",
  "style": "solid_arrow"  # solid_arrow, dashed_arrow, bidirectional
}
```

### File Location

```
book_frontend/docs/diagrams/module-1/{diagram_id}.md
```

---

## Entity: ChecklistItem

Represents a self-assessment item for learner verification.

### Attributes

| Attribute | Type | Description | Required |
|-----------|------|-------------|----------|
| `statement` | String | "I can..." statement | Yes |
| `assessment_type` | String | How learner verifies (explain, demonstrate, identify) | Yes |
| `difficulty` | String | Basic, intermediate, advanced | Yes |

### Example

```markdown
- [ ] **I can explain** why robots need middleware like ROS 2 (explain to a peer or write 3-5 sentences)
- [ ] **I can identify** the three communication patterns in ROS 2 (topics, services, actions)
- [ ] **I can demonstrate** creating a Python publisher node that emits messages at 10Hz
```

### Validation Rules

1. Must use "I can..." format (promotes self-efficacy)
2. Must include assessment method in parentheses
3. 5-7 items per chapter (SC-007 requirement)

---

## Module 1 Chapter Outline

### Chapter 1: Introduction to Physical AI & ROS 2

**Learning Goals**:
- Define physical AI and embodied intelligence
- Explain why humanoid robots require middleware
- Describe ROS 2's role in the robotics ecosystem
- Compare ROS 2 to traditional software architectures

**Core Concepts**:
1. Physical AI and Embodied Intelligence
2. Middleware for Robot Systems
3. ROS 2 Architecture Overview
4. ROS 2 vs. Traditional Software

**Diagrams**: 1 (Physical AI system components)

**Code Examples**: 0 (conceptual only)

**Checklist Items**: 5

---

### Chapter 2: ROS 2 Architecture Fundamentals

**Learning Goals**:
- Describe the ROS 2 node lifecycle
- Differentiate topics, services, and actions
- Explain DDS middleware role
- Trace signal flow in humanoid control systems

**Core Concepts**:
1. Nodes and Executors
2. Topics (Publish-Subscribe)
3. Services (Request-Response)
4. Actions (Goal-Oriented)
5. DDS and Communication Lifecycle

**Diagrams**: 2 (Node lifecycle, Signal flow)

**Code Examples**: 0 (pseudo-code snippets for illustration)

**Checklist Items**: 7

---

### Chapter 3: Building ROS 2 Nodes with Python

**Learning Goals**:
- Create and run Python ROS 2 nodes
- Implement publishers and subscribers
- Use parameters and logging
- Debug common node issues

**Core Concepts**:
1. rclpy Fundamentals
2. Publisher Pattern
3. Subscriber Pattern
4. Parameters and Configuration
5. Logging and Debugging

**Diagrams**: 1 (Publisher-Subscriber interaction)

**Code Examples**: 3
- `publisher_example.py`: Basic publisher
- `subscriber_example.py`: Basic subscriber
- `param_logger_example.py`: Parameters + logging

**Checklist Items**: 6

---

### Chapter 4: ROS 2 Packages, Launch, and Integration

**Learning Goals**:
- Understand ROS 2 package structure
- Create launch files for multi-node systems
- Integrate Python AI agents with ROS nodes
- Manage node lifecycle and parameters

**Core Concepts**:
1. ROS 2 Package Anatomy
2. Launch Files and Configuration
3. Multi-Node Coordination
4. Bridging AI Agents to ROS

**Diagrams**: 2 (Package structure, AI-ROS integration)

**Code Examples**: 2
- `simple_launch.py`: Launch file example
- `ai_agent_node.py`: AI decision logic as ROS node

**Checklist Items**: 6

---

### Chapter 5: Humanoid Robot Description with URDF

**Learning Goals**:
- Parse URDF files for robot structure
- Identify links, joints, coordinate frames
- Distinguish visual vs. collision models
- Prepare URDF for simulation tools

**Core Concepts**:
1. URDF Purpose and Structure
2. Links and Joints
3. Coordinate Frames and Transformations
4. Visual vs. Collision Geometry
5. URDF for Simulation

**Diagrams**: 2 (URDF tree structure, Joint types)

**Code Examples**: 1
- `simple_arm.urdf`: Annotated 3-joint arm URDF

**Checklist Items**: 6

---

## Content Organization

### Directory Structure

```
book_frontend/docs/
├── module-1-ros2/
│   ├── chapter-1-intro-physical-ai.md
│   ├── chapter-2-ros2-architecture.md
│   ├── chapter-3-python-nodes.md
│   ├── chapter-4-packages-launch.md
│   └── chapter-5-urdf.md
├── code-examples/
│   └── module-1/
│       ├── publisher_example.py
│       ├── subscriber_example.py
│       ├── param_logger_example.py
│       ├── simple_launch.py
│       ├── ai_agent_node.py
│       └── simple_arm.urdf
└── diagrams/
    └── module-1/
        ├── physical-ai-components.md
        ├── node-lifecycle.md
        ├── signal-flow.md
        ├── pubsub-interaction.md
        ├── package-structure.md
        ├── ai-ros-integration.md
        ├── urdf-tree.md
        └── joint-types.md
```

### Navigation Flow

Chapter 1 → Chapter 2 → Chapter 3 → Chapter 4 → Chapter 5

Each chapter links:
- **Previous**: Link to prior chapter (if not Ch1)
- **Next**: Link to next chapter (if not Ch5)
- **Code Examples**: Links to relevant code files
- **Diagrams**: Inline references to diagram specs

---

## Validation Checklist

Before content considered complete:

- [ ] All 5 chapters created with required sections
- [ ] Each chapter has 3-5 core concepts
- [ ] All ROS 2 terminology defined before use (FR-003)
- [ ] Chapters 3-5 include runnable code examples
- [ ] All code examples target Humble/Iron (FR-004)
- [ ] Each chapter has 5-7 checklist items
- [ ] All diagrams have textual specifications
- [ ] Learning progression is logical (Ch1 → Ch5)
- [ ] Summary sections capture key takeaways
- [ ] Sidebars.js correctly references all chapter files

---

## Future Modules (Out of Scope)

Module 2+: Simulation (Gazebo/Isaac Sim), Advanced Control, Hardware Integration - not covered in this data model.
