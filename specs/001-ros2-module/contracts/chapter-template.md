# Chapter Template: Module 1 - ROS 2

**Version**: 1.0
**Date**: 2025-12-21

This template defines the structure for all Module 1 chapters. Copy and fill in placeholders for each chapter.

---

## Front Matter

```markdown
---
sidebar_position: {CHAPTER_NUMBER}
---
```

## Chapter Header

```markdown
# Chapter {CHAPTER_NUMBER}: {CHAPTER_TITLE}

**Week {CHAPTER_NUMBER}** | **Estimated Time**: 4-6 hours

## Learning Goals

By the end of this chapter, you will be able to:

- {LEARNING_GOAL_1}
- {LEARNING_GOAL_2}
- {LEARNING_GOAL_3}
- {LEARNING_GOAL_4}
- {LEARNING_GOAL_5}
```

---

## Introduction Section

```markdown
## Introduction

{1-2 paragraph overview explaining:
- Why this chapter matters
- How it connects to previous chapters
- What learners will build/understand
}

**Prerequisites**: {List what learners should know from prior chapters, or "None" for Chapter 1}

**Key Concepts**: {List 3-5 main concepts covered}
```

---

## Core Concepts Section (3-5 concepts)

```markdown
## Core Concepts

### {CONCEPT_1_TITLE}

{2-4 paragraphs explaining the concept. Use analogies, examples, and clear definitions.}

**Key Terminology**:
- **{TERM_1}**: {Definition}
- **{TERM_2}**: {Definition}
- **{TERM_3}**: {Definition}

{Optional: Common Mistakes section}
**Common Mistakes**:
- {Pitfall and how to avoid it}

---

### {CONCEPT_2_TITLE}

{Repeat pattern above}

---

{Continue for all concepts}
```

---

## Diagrams Section

```markdown
## Architecture & Flow Diagrams

### {DIAGRAM_1_TITLE}

{1 paragraph explaining what the diagram shows and why it's important}

See the detailed diagram specification: [{Diagram Title}](../../diagrams/module-1/{diagram-id}.md)

**Key Takeaways from Diagram**:
- {Insight 1}
- {Insight 2}
- {Insight 3}

---

{Repeat for additional diagrams if applicable}
```

---

## Code Examples Section (Chapters 3-5 only)

```markdown
## Hands-On Code Examples

{1 paragraph intro explaining what learners will build}

### Example {N}: {EXAMPLE_TITLE}

**Purpose**: {One sentence describing what this example demonstrates}

**ROS 2 Concepts**: {List concepts used: e.g., Publishers, Topics, rclpy}

**Code**:

```python title="{filename}.py"
{FULL_CODE_HERE}
```

**How to Run**:

```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash  # or iron

# Run the example
python3 {filename}.py
```

**Expected Output**:

```
{PASTE_EXPECTED_TERMINAL_OUTPUT}
```

**Code Walkthrough**:

1. **Lines X-Y**: {Explanation of this code section}
2. **Lines A-B**: {Explanation of this code section}
3. **{Function/Method Name}**: {What it does and why}

**Try It Yourself**:
- Modify {parameter} to see {expected behavior change}
- Add {feature} to practice {concept}

---

{Repeat for additional examples}
```

---

## Summary Section

```markdown
## Summary

In this chapter, you learned:

- {KEY_TAKEAWAY_1}
- {KEY_TAKEAWAY_2}
- {KEY_TAKEAWAY_3}
- {KEY_TAKEAWAY_4}
- {KEY_TAKEAWAY_5}

**Connection to Next Chapter**: {1 sentence teasing what's next}
```

---

## Self-Assessment Checklist

```markdown
## Self-Assessment Checklist

Complete these items to verify your understanding before moving to the next chapter:

- [ ] **I can explain** {concept} (in 3-5 sentences or to a peer)
- [ ] **I can identify** {elements/components} in {context}
- [ ] **I can demonstrate** {skill} (by {specific action})
- [ ] **I can compare** {concept A} and {concept B} (listing 3 differences)
- [ ] **I can trace** {process flow} from {start} to {end}
- [ ] **I can create** {artifact} that {meets criteria}

{Note: 5-7 items total, use varied assessment types}
```

---

## Additional Resources (Optional)

```markdown
## Additional Resources

**Official Documentation**:
- [{Resource Title}]({URL}) - {Brief description}

**Community Resources**:
- [{Tutorial/Blog Title}]({URL}) - {Brief description}

**Troubleshooting**:
- {Common Issue 1}: {Solution}
- {Common Issue 2}: {Solution}
```

---

## Template Checklist

Before considering chapter complete:

- [ ] Front matter includes sidebar_position
- [ ] Learning goals listed (3-5 items)
- [ ] Introduction explains why + prerequisites
- [ ] 3-5 core concepts with definitions
- [ ] All ROS 2 terms defined before first use
- [ ] Diagrams referenced with links to specs
- [ ] Code examples (if Ch3-5) are runnable
- [ ] Code examples include run commands + expected output
- [ ] Summary captures key takeaways
- [ ] Self-assessment checklist has 5-7 items
- [ ] No placeholders remain (all {CAPS} replaced)
