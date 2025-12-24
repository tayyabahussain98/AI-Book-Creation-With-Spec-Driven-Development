<<<<<<< HEAD
# Diagram Template: Module 4 - Vision-Language-Action

**Purpose**: Ensure consistent diagram specifications across Module 4
**Based On**: Modules 1-3 diagram patterns

---

## Required Structure

```markdown
---
title: {Diagram Title}
module: 4
chapter: {1-5}
type: {architecture|flow|comparison|state|entity}
---

# {Diagram Title}

## Overview

{1-2 sentences describing what this diagram illustrates}

---

## Diagram

```text
{ASCII art diagram}
{Use box characters: ┌ ┐ └ ┘ │ ─ ├ ┤ ┬ ┴ ┼}
{Use arrows: → ← ↑ ↓ ↔ ⟶ ⟵}
{Keep width under 80 characters for readability}
```

---

## Component Descriptions

### {Component 1}

**Purpose**: {What this component does}
**Inputs**: {What it receives}
**Outputs**: {What it produces}
**Key Details**: {Important implementation notes}

### {Component 2}

{Same structure}

{... repeat for all major components}

---

## Data Flow

1. **Step 1**: {Description of first step}
2. **Step 2**: {Description of second step}
{... continue for complete flow}

---

## Key Insights

### {Insight 1 Title}
{Explanation of important architectural decision or pattern}

### {Insight 2 Title}
{Explanation of important architectural decision or pattern}

---

## Related Concepts

- **From Module 1**: {Relevant ROS 2 concept}
- **From Module 2**: {Relevant simulation concept}
- **From Module 3**: {Relevant Isaac concept}
- **In This Module**: {Related Module 4 diagram/chapter}

---

## Real-World Application

{1-2 paragraphs explaining how this architecture/pattern is used in actual robotic systems}
{Reference specific robots or research projects where appropriate}
```

---

## Diagram Types and Templates

### 1. Architecture Diagram

For system overviews and component relationships.

```text
┌─────────────────────────────────────────────────────────────┐
│                    SYSTEM NAME                               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐     ┌──────────┐     ┌──────────┐            │
│  │  Input   │────►│ Process  │────►│  Output  │            │
│  │  Layer   │     │  Layer   │     │  Layer   │            │
│  └──────────┘     └──────────┘     └──────────┘            │
│       │                │                │                   │
│       └────────────────┼────────────────┘                   │
│                        ▼                                    │
│               ┌──────────────┐                              │
│               │ Shared State │                              │
│               └──────────────┘                              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 2. Flow Diagram

For process sequences and pipelines.

```text
┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐
│ Step 1  │────►│ Step 2  │────►│ Step 3  │────►│ Step 4  │
└─────────┘     └────┬────┘     └─────────┘     └─────────┘
                     │
                     ▼ (on error)
                ┌─────────┐
                │ Handler │
                └─────────┘
```

### 3. Comparison Diagram

For contrasting approaches or options.

```text
┌─────────────────────┬─────────────────────┬─────────────────────┐
│     APPROACH A      │     APPROACH B      │     APPROACH C      │
├─────────────────────┼─────────────────────┼─────────────────────┤
│ ✓ Advantage 1       │ ✓ Advantage 1       │ ✓ Advantage 1       │
│ ✓ Advantage 2       │ ✗ Disadvantage 1    │ ✓ Advantage 2       │
│ ✗ Disadvantage 1    │ ✓ Advantage 2       │ ✗ Disadvantage 1    │
├─────────────────────┼─────────────────────┼─────────────────────┤
│ Best for: {use}     │ Best for: {use}     │ Best for: {use}     │
└─────────────────────┴─────────────────────┴─────────────────────┘
```

### 4. State Diagram

For entity lifecycles and state machines.

```text
                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │ start
                         ▼
┌─────────┐        ┌─────────┐        ┌─────────┐
│ FAILED  │◄───────│EXECUTING│───────►│COMPLETED│
└────┬────┘  error └────┬────┘ done   └─────────┘
     │                  │
     │    retry         │ pause
     └──────────────────┤
                        ▼
                   ┌─────────┐
                   │ PAUSED  │
                   └─────────┘
```

### 5. Entity Relationship Diagram

For data models and relationships.

```text
┌─────────────┐         ┌─────────────┐
│   Entity A  │─────────│   Entity B  │
│  - field1   │   1:N   │  - field1   │
│  - field2   │         │  - field2   │
└──────┬──────┘         └─────────────┘
       │
       │ 1:1
       │
┌──────▼──────┐
│   Entity C  │
│  - field1   │
└─────────────┘
```

---

## Module 4 Diagram Inventory

| Diagram | Type | Chapter | File Name |
|---------|------|---------|-----------|
| VLA Architecture Overview | architecture | 1 | vla-architecture-overview.md |
| Embodied Cognition Layers | architecture | 1 | embodied-cognition-layers.md |
| Voice Pipeline | flow | 2 | voice-pipeline.md |
| Intent Extraction Flow | flow | 2 | intent-extraction-flow.md |
| LLM Planning Pipeline | flow | 3 | llm-planning-pipeline.md |
| Action Graph Structure | entity | 3 | action-graph-structure.md |
| Multimodal Fusion Architecture | architecture | 4 | multimodal-fusion-architecture.md |
| Visual Grounding Flow | flow | 4 | visual-grounding-flow.md |
| Capstone System Architecture | architecture | 5 | capstone-system-architecture.md |
| Safety Constraints | architecture | 5 | safety-constraints-diagram.md |

---

## Validation Checklist

Before submitting a diagram, verify:

- [ ] Front matter includes title, module, chapter, type
- [ ] Overview explains diagram purpose
- [ ] ASCII art is clear and under 80 chars wide
- [ ] All components have descriptions
- [ ] Data flow is documented step-by-step
- [ ] Key insights explain architectural decisions
- [ ] Related concepts link to other modules
- [ ] Real-world application provides context
- [ ] No special characters that break MDX
=======
# Diagram Template: Module 4 - Vision-Language-Action

**Purpose**: Ensure consistent diagram specifications across Module 4
**Based On**: Modules 1-3 diagram patterns

---

## Required Structure

```markdown
---
title: {Diagram Title}
module: 4
chapter: {1-5}
type: {architecture|flow|comparison|state|entity}
---

# {Diagram Title}

## Overview

{1-2 sentences describing what this diagram illustrates}

---

## Diagram

```text
{ASCII art diagram}
{Use box characters: ┌ ┐ └ ┘ │ ─ ├ ┤ ┬ ┴ ┼}
{Use arrows: → ← ↑ ↓ ↔ ⟶ ⟵}
{Keep width under 80 characters for readability}
```

---

## Component Descriptions

### {Component 1}

**Purpose**: {What this component does}
**Inputs**: {What it receives}
**Outputs**: {What it produces}
**Key Details**: {Important implementation notes}

### {Component 2}

{Same structure}

{... repeat for all major components}

---

## Data Flow

1. **Step 1**: {Description of first step}
2. **Step 2**: {Description of second step}
{... continue for complete flow}

---

## Key Insights

### {Insight 1 Title}
{Explanation of important architectural decision or pattern}

### {Insight 2 Title}
{Explanation of important architectural decision or pattern}

---

## Related Concepts

- **From Module 1**: {Relevant ROS 2 concept}
- **From Module 2**: {Relevant simulation concept}
- **From Module 3**: {Relevant Isaac concept}
- **In This Module**: {Related Module 4 diagram/chapter}

---

## Real-World Application

{1-2 paragraphs explaining how this architecture/pattern is used in actual robotic systems}
{Reference specific robots or research projects where appropriate}
```

---

## Diagram Types and Templates

### 1. Architecture Diagram

For system overviews and component relationships.

```text
┌─────────────────────────────────────────────────────────────┐
│                    SYSTEM NAME                               │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────┐     ┌──────────┐     ┌──────────┐            │
│  │  Input   │────►│ Process  │────►│  Output  │            │
│  │  Layer   │     │  Layer   │     │  Layer   │            │
│  └──────────┘     └──────────┘     └──────────┘            │
│       │                │                │                   │
│       └────────────────┼────────────────┘                   │
│                        ▼                                    │
│               ┌──────────────┐                              │
│               │ Shared State │                              │
│               └──────────────┘                              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 2. Flow Diagram

For process sequences and pipelines.

```text
┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐
│ Step 1  │────►│ Step 2  │────►│ Step 3  │────►│ Step 4  │
└─────────┘     └────┬────┘     └─────────┘     └─────────┘
                     │
                     ▼ (on error)
                ┌─────────┐
                │ Handler │
                └─────────┘
```

### 3. Comparison Diagram

For contrasting approaches or options.

```text
┌─────────────────────┬─────────────────────┬─────────────────────┐
│     APPROACH A      │     APPROACH B      │     APPROACH C      │
├─────────────────────┼─────────────────────┼─────────────────────┤
│ ✓ Advantage 1       │ ✓ Advantage 1       │ ✓ Advantage 1       │
│ ✓ Advantage 2       │ ✗ Disadvantage 1    │ ✓ Advantage 2       │
│ ✗ Disadvantage 1    │ ✓ Advantage 2       │ ✗ Disadvantage 1    │
├─────────────────────┼─────────────────────┼─────────────────────┤
│ Best for: {use}     │ Best for: {use}     │ Best for: {use}     │
└─────────────────────┴─────────────────────┴─────────────────────┘
```

### 4. State Diagram

For entity lifecycles and state machines.

```text
                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │ start
                         ▼
┌─────────┐        ┌─────────┐        ┌─────────┐
│ FAILED  │◄───────│EXECUTING│───────►│COMPLETED│
└────┬────┘  error └────┬────┘ done   └─────────┘
     │                  │
     │    retry         │ pause
     └──────────────────┤
                        ▼
                   ┌─────────┐
                   │ PAUSED  │
                   └─────────┘
```

### 5. Entity Relationship Diagram

For data models and relationships.

```text
┌─────────────┐         ┌─────────────┐
│   Entity A  │─────────│   Entity B  │
│  - field1   │   1:N   │  - field1   │
│  - field2   │         │  - field2   │
└──────┬──────┘         └─────────────┘
       │
       │ 1:1
       │
┌──────▼──────┐
│   Entity C  │
│  - field1   │
└─────────────┘
```

---

## Module 4 Diagram Inventory

| Diagram | Type | Chapter | File Name |
|---------|------|---------|-----------|
| VLA Architecture Overview | architecture | 1 | vla-architecture-overview.md |
| Embodied Cognition Layers | architecture | 1 | embodied-cognition-layers.md |
| Voice Pipeline | flow | 2 | voice-pipeline.md |
| Intent Extraction Flow | flow | 2 | intent-extraction-flow.md |
| LLM Planning Pipeline | flow | 3 | llm-planning-pipeline.md |
| Action Graph Structure | entity | 3 | action-graph-structure.md |
| Multimodal Fusion Architecture | architecture | 4 | multimodal-fusion-architecture.md |
| Visual Grounding Flow | flow | 4 | visual-grounding-flow.md |
| Capstone System Architecture | architecture | 5 | capstone-system-architecture.md |
| Safety Constraints | architecture | 5 | safety-constraints-diagram.md |

---

## Validation Checklist

Before submitting a diagram, verify:

- [ ] Front matter includes title, module, chapter, type
- [ ] Overview explains diagram purpose
- [ ] ASCII art is clear and under 80 chars wide
- [ ] All components have descriptions
- [ ] Data flow is documented step-by-step
- [ ] Key insights explain architectural decisions
- [ ] Related concepts link to other modules
- [ ] Real-world application provides context
- [ ] No special characters that break MDX
>>>>>>> 5cf5f6818272e652910633754d301908ff0bb236
