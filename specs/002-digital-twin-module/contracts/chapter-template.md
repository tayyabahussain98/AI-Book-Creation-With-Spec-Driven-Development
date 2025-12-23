# Chapter Template - Module 2

**Purpose**: Standard structure for all Module 2 chapters

## Front Matter

```yaml
---
sidebar_position: [1-5]
title: Chapter [X] - [Title]
---
```

## Required Sections

### 1. Title and Introduction

```markdown
# Chapter [X]: [Title]

## Introduction

[Prerequisites: Which chapters or modules must be completed first]

[Learning objectives: 3-5 bullet points of what learners will achieve]

[Motivation: Why this topic matters for humanoid robotics]
```

### 2. Core Concepts (3-5 sections)

```markdown
## Core Concept 1: [Topic Name]

### [Subtopic if needed]

[Conceptual explanation with terminology definitions]

[Examples or analogies for clarity]

[Best practices or common pitfalls]
```

### 3. Hands-On Example

```markdown
## Hands-On Example: [Example Name]

[Introduction to the example and what it demonstrates]

**File**: `[filename]` ([brief description])

```[language]
[Embedded code with inline comments]
```

### Key Elements Explained

1. **[Element 1]**: [What it does and why it's important]
2. **[Element 2]**: [Configuration options and typical values]
3. **[Element 3]**: [How it integrates with ROS 2 or other components]

### Running the Example

```bash
# [Command to execute the example]
gz sim [filename]
```

**Expected Output**: [What learner should see when running the example]
```

### 4. Diagrams Section

```markdown
## Diagrams

### [Diagram 1 Title]

See the [Diagram Name](/docs/diagrams/module-2/filename.md) for:
- [Key point 1]
- [Key point 2]
- [Key point 3]

**Key Takeaway**: [One-sentence summary of diagram insight]

### [Diagram 2 Title]

See the [Diagram Name](/docs/diagrams/module-2/filename2.md) for:
- [Key point 1]
- [Key point 2]

**Key Takeaway**: [One-sentence summary]
```

### 5. Summary

```markdown
## Summary

This chapter covered [topic]:

1. **[Key Point 1]**: [One-sentence summary]
2. **[Key Point 2]**: [One-sentence summary]
3. **[Key Point 3]**: [One-sentence summary]
4. **[Key Point 4]**: [One-sentence summary]
5. **[Key Point 5]**: [One-sentence summary]
6. **[Key Point 6]**: [One-sentence summary with forward reference to next chapter]
```

### 6. Self-Assessment Checklist

```markdown
## Self-Assessment Checklist

After completing this chapter, you should be able to:

- [ ] **I can explain** [fundamental concept from Core Concept 1]
- [ ] **I can identify** [key component or pattern from Core Concept 2]
- [ ] **I can create** [practical skill from hands-on example]
- [ ] **I can distinguish** [comparison or contrast from Core Concept 3]
- [ ] **I can apply** [integration skill from Core Concept 4]
- [ ] **I can troubleshoot** [debugging skill or common issue]

**Next Steps**: [Preview of next chapter and how it builds on this one]
```

## Content Guidelines

### Writing Style
- **Tone**: Educational, encouraging, beginner-friendly
- **Technical Level**: Assumes Module 1 completion (ROS 2, URDF knowledge)
- **Concept-First**: Explain "why" before "how", theory before practice
- **Practical**: Every concept tied to hands-on example or real-world application

### Code Annotation Standards
- Every code block must have introductory context
- XML/SDF examples must have inline comments (<!-- Comment -->) explaining non-obvious elements
- Commands must show expected output or describe what learner will see
- No code dumps without explanation

### Terminology
- Define technical terms on first use (e.g., "ZMP (Zero-Moment Point)")
- Use consistent terminology across chapters
- Link back to Module 1 concepts when building on previous knowledge

### Length Targets
- **Introduction**: 150-250 words
- **Each Core Concept**: 400-600 words
- **Hands-On Example**: 300-500 words + code
- **Summary**: 100-150 words
- **Total**: 3000-4000 words (~600-800 lines with code)

## Validation Checklist

Before marking chapter complete, verify:

- [ ] Front matter present and correct
- [ ] Introduction states prerequisites and objectives
- [ ] At least 3 core concepts with clear explanations
- [ ] At least 1 hands-on example with annotated code
- [ ] At least 1 diagram reference (preferably 2-3)
- [ ] 6-bullet summary recaps key points
- [ ] 6-item self-assessment checklist with "I can..." format
- [ ] No broken links to diagrams or examples
- [ ] Markdown builds without errors in Docusaurus
- [ ] Technical accuracy verified (no hallucinated Gazebo/Unity APIs)
