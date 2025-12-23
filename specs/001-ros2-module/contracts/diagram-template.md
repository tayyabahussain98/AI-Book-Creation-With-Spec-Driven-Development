# Diagram Template: Module 1 - ROS 2

**Version**: 1.0
**Date**: 2025-12-21

This template defines the structure for diagram specifications. Diagrams are described textually for later rendering as Mermaid diagrams, illustrations, or hand-drawn sketches.

---

## Diagram Specification Template

```markdown
# Diagram: {DIAGRAM_TITLE}

**ID**: `{diagram-id-slug}`
**Chapter**: Chapter {N} - {Chapter Title}
**Date**: {YYYY-MM-DD}

## Purpose

{2-3 sentences explaining what this diagram illustrates and why it's valuable for learners}

## Components

{List all elements that appear in the diagram}

1. **{COMPONENT_1_NAME}** ({type: software_node | hardware | middleware | data})
   - Description: {What this represents}
   - Shape: {rectangle | rounded_rectangle | ellipse | cylinder | diamond}
   - Color/Style: {Optional: visual hints}

2. **{COMPONENT_2_NAME}** ({type})
   - Description: {What this represents}
   - Shape: {shape}

{Continue for all components...}

## Connections

{List all arrows, flows, relationships between components}

| From | To | Type | Label | Style |
|------|----|----|-------|-------|
| {Component A} | {Component B} | data_flow | {e.g., "Publish(topic=/cmd)"} | solid_arrow |
| {Component C} | {Component D} | control_flow | {e.g., "Start/Stop"} | dashed_arrow |
| {Component E} | {Component F} | dependency | {e.g., "requires"} | dotted_line |

**Flow Types**:
- `data_flow`: Information/messages moving between components
- `control_flow`: Commands or lifecycle management
- `dependency`: "Requires" or "uses" relationship

**Style Guide**:
- `solid_arrow`: Primary data/control flow
- `dashed_arrow`: Asynchronous or optional flow
- `dotted_line`: Weak dependency or configuration
- `bidirectional`: Two-way communication

## Diagram Flow (Textual Representation)

{ASCII-style or indented flow showing the sequence/structure}

```
{COMPONENT_A}
    ↓
{COMPONENT_B} → {COMPONENT_C}
                       ↓
               {COMPONENT_D}
```

Or for more complex flows:

```
[Start] → {Process 1} → [Decision Point]
                              ↓
                        Yes ← + → No
                         ↓           ↓
                   {Process 2}  {Process 3}
                         ↓           ↓
                         └─────┬─────┘
                               ↓
                            [End]
```

## Labels & Legend

**Symbol Legend**:
- Solid box: {What it represents}
- Rounded box: {What it represents}
- Dashed box: {What it represents}
- Arrow direction: {What it represents}
- Line thickness: {What it represents}

**Color Coding** (if applicable):
- Blue: {Category, e.g., "Software"}
- Green: {Category, e.g., "Hardware"}
- Orange: {Category, e.g., "Configuration"}

## Annotations & Notes

{Important details or context that don't fit in components/connections}

- **Note 1**: {Clarification about diagram element or flow}
- **Note 2**: {Edge case or special behavior shown}
- **Note 3**: {Real-world analogy for the diagram}

## Example Instantiation

{Optional: Concrete example using the abstract diagram structure}

**Scenario**: {Specific use case}

- {Component A} = {Real example, e.g., "AI Decision Node"}
- {Component B} = {Real example, e.g., "Motor Controller"}
- {Flow X} = {Real message/data, e.g., "MoveArm(angle=45°)"}

## Rendering Suggestions

**Mermaid Syntax Hint**:

```mermaid
graph TD
    A[{Component A}] --> B[{Component B}]
    B --> C[{Component C}]
    C --> D[{Component D}]
```

{Adjust based on actual diagram complexity}

**Alternative Tools**:
- draw.io / diagrams.net (for complex visual layouts)
- PlantUML (for UML-style diagrams)
- Hand-drawn + scanned (for informal teaching style)

## Validation Checklist

Before considering diagram spec complete:

- [ ] Purpose clearly states learning value
- [ ] All components listed with types and shapes
- [ ] All connections documented in table
- [ ] Flow direction is unambiguous
- [ ] Legend explains all symbols
- [ ] Notes clarify non-obvious elements
- [ ] Example instantiation provided (if abstract)
- [ ] Rendering suggestion included
- [ ] No placeholders remain (all {CAPS} replaced)
```

---

## Example Diagram Specification

See `/specs/001-ros2-module/contracts/example-signal-flow-diagram.md` for a complete filled example.
