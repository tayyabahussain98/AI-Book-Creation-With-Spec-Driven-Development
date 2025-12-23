# Diagram Template: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Purpose**: Standard structure for diagram specifications to ensure clarity and consistency.

**Format**: Text-based markdown with ASCII art or structured text representation

---

## Diagram Specification: [DIAGRAM_NAME]

**Type**: [Architecture | DataFlow | DecisionTree | ComparisonTable | ProcessWorkflow]

**Purpose**: [One-sentence description of why this diagram exists and what it clarifies]

**Referenced By**: Chapter [X], Chapter [Y] (list all chapters that reference this diagram)

---

## Visual Representation

### [Option 1: ASCII Art Diagram]

```
[ASCII art representation of the diagram]
[Use boxes, arrows, and text to illustrate relationships]
[Example:]

┌─────────────────┐
│  Component A    │
│  (Description)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐       ┌─────────────────┐
│  Component B    │──────>│  Component C    │
│  (Description)  │       │  (Description)  │
└─────────────────┘       └─────────────────┘
```

### [Option 2: Structured Text Representation]

**Components**:
1. **[Component/Stage 1]**: [Description]
2. **[Component/Stage 2]**: [Description]
3. **[Component/Stage 3]**: [Description]

**Relationships**:
- [Component 1] → [Component 2]: [Explanation of connection]
- [Component 2] → [Component 3]: [Explanation of connection]

### [Option 3: Comparison Table]

| Dimension | [Option A] | [Option B] | [Option C] |
|-----------|------------|------------|------------|
| [Criterion 1] | [Value/Description] | [Value/Description] | [Value/Description] |
| [Criterion 2] | [Value/Description] | [Value/Description] | [Value/Description] |
| [Criterion 3] | [Value/Description] | [Value/Description] | [Value/Description] |
| [Criterion 4] | [Value/Description] | [Value/Description] | [Value/Description] |

### [Option 4: Decision Tree]

```
Start: [Question/Decision Point]
  ├─ If [Condition A]:
  │   └─> Use [Option 1]
  │       Reason: [Explanation]
  ├─ If [Condition B]:
  │   └─> Use [Option 2]
  │       Reason: [Explanation]
  └─ If [Condition C]:
      └─> Use [Option 3]
          Reason: [Explanation]
```

---

## Detailed Explanation (300-500 words)

[Provide comprehensive written explanation of the diagram]

### [Subsection 1: Component Details]

[Explain each component, stage, or element in detail]

### [Subsection 2: Relationships and Flow]

[Describe how components interact, data flows, or decisions are made]

### [Subsection 3: Key Insights]

[Highlight important takeaways, design principles, or trade-offs illustrated by this diagram]

---

## Usage in Book

This diagram appears in:
- **Chapter [X], Section [Y]**: [How it supports that content]
- **Chapter [Z], Section [W]**: [How it supports that content]

**Context**: [Explain when readers encounter this diagram and what they should understand from it]

---

## Related Diagrams

- [related-diagram-1.md](related-diagram-1.md): [Relationship explanation]
- [related-diagram-2.md](related-diagram-2.md): [Relationship explanation]

---

## Notes for Implementation

- **Diagram type** determines visual style (architecture = boxes/arrows, workflow = sequential steps, etc.)
- **ASCII art** preferred for simplicity, but structured text acceptable for complex concepts
- **Comparison tables** best for decision-making criteria (Gazebo vs Isaac Sim, CPU vs GPU)
- **Decision trees** clarify "when to use X" questions
- **300-500 word explanation** ensures diagram is self-contained if visual is unclear

---

## Template Notes (Remove in actual diagram)

- **Purpose**: Always state why this diagram exists (what confusion it resolves)
- **Visual representation**: Choose format that best communicates concept
- **Explanation**: Provide context readers need to interpret diagram correctly
- **Related diagrams**: Link conceptually similar diagrams to build understanding
- **Usage tracking**: List all chapters referencing this diagram for maintenance
