# Diagram Specification Template - Module 2

**Purpose**: Standard structure for all Module 2 diagram specifications

## Required Sections

### 1. Header

```markdown
# [Diagram Title] Specification

**Purpose**: [One-sentence description of what this diagram illustrates]

**Diagram Type**: [workflow | architecture | comparison | data_flow | state_machine | component]
```

### 2. Visual Representation

Choose format based on diagram type:

#### For Workflows and Data Flow

```markdown
## [Workflow/Process Name]

```text
Step 1: [Action]
   ↓
Step 2: [Action]
   ↓
Step 3: [Action]
```

Or ASCII art:

```text
┌──────────┐      ┌──────────┐      ┌──────────┐
│ Component│─────→│ Component│─────→│ Component│
│    A     │      │    B     │      │    C     │
└──────────┘      └──────────┘      └──────────┘
```
```

#### For Comparisons

```markdown
## Comparison: [Thing A] vs [Thing B]

| Aspect | [Thing A] | [Thing B] |
|--------|-----------|-----------|
| **[Dimension 1]** | [Description] | [Description] |
| **[Dimension 2]** | [Description] | [Description] |
| **[Dimension 3]** | [Description] | [Description] |
```

#### For Architecture

```markdown
## [System Name] Architecture

```text
┌─────────────────────────────────┐
│      [Top Layer Component]      │
├─────────────────────────────────┤
│   [Middle Layer Component 1]    │
│   [Middle Layer Component 2]    │
├─────────────────────────────────┤
│     [Bottom Layer Component]    │
└─────────────────────────────────┘
```

**Components**:
- **[Component 1]**: [Role and responsibilities]
- **[Component 2]**: [Role and responsibilities]
```

### 3. Detailed Explanation

```markdown
## [Section Title Based on Diagram Type]

[Detailed breakdown of diagram elements]

### [Component/Step 1]
[Explanation of this part of the diagram]

### [Component/Step 2]
[Explanation with context and relationships]

[Continue for all major components]
```

### 4. Usage in Book

```markdown
## Usage in Book

- **Referenced in**: Chapter [X] ([Section Name])
- **Purpose**: [Why learners need this visualization at this point in their learning]
- **Learning Goal**: [What specific insight or understanding learners should gain from this diagram]

## Key Takeaways

1. [Main insight 1]
2. [Main insight 2]
3. [Main insight 3]
```

## Diagram Type Guidelines

### Workflow Diagrams
**Use for**: Step-by-step processes, development workflows, simulation pipelines
**Format**: Sequential arrows with text descriptions
**Example**: Digital twin lifecycle (design → simulate → test → deploy)

### Architecture Diagrams
**Use for**: System components and their relationships
**Format**: Layered boxes or component diagrams
**Example**: Gazebo architecture (physics engine, rendering, plugins, ROS bridge)

### Comparison Tables
**Use for**: Contrasting two or more concepts or tools
**Format**: Markdown table with multiple dimensions
**Example**: URDF vs SDF feature comparison

### Data Flow Diagrams
**Use for**: How data moves through a system
**Format**: Boxes with arrows showing data paths
**Example**: Sensor plugin → ROS 2 topic → Perception node

### State Machine Diagrams
**Use for**: States and transitions
**Format**: Circles/boxes for states, arrows for transitions
**Example**: Robot simulation states (paused, running, step-by-step)

## Content Guidelines

### ASCII Art Best Practices
- Use box-drawing characters: ┌ ─ ┐ │ └ ┘ ├ ┤ ┬ ┴ ┼
- Keep diagrams under 80 characters wide for readability
- Use arrows: → ← ↑ ↓ ↔ ↕
- Use symbols sparingly: ✓ ✗ ⚠ ⟲ ⟳

### Table Formatting
- Align pipes for readability
- Use **bold** for row/column headers
- Keep cell content concise (1-2 sentences max)
- Use 3+ dashes in separator row: `|--------|`

### Annotations
- Every diagram must have explanatory text
- Define unfamiliar terms inline
- Provide context for why diagram structure was chosen
- Link diagram elements to book concepts

## Validation Checklist

Before marking diagram complete, verify:

- [ ] Purpose statement present
- [ ] Diagram type specified
- [ ] Visual representation is text-only (no images)
- [ ] Detailed explanation section present
- [ ] Usage in book section present
- [ ] Referenced by at least one chapter
- [ ] Markdown syntax valid
- [ ] ASCII art renders correctly in monospace font
- [ ] Tables formatted properly
- [ ] No broken links
