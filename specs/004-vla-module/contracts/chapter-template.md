# Chapter Template: Module 4 - Vision-Language-Action

**Purpose**: Ensure consistent structure across all Module 4 chapters
**Based On**: Modules 1-3 chapter patterns

---

## Required Structure

```markdown
---
sidebar_position: {1-5}
title: Chapter {N} - {Title}
---

# Chapter {N}: {Full Title}

## Introduction

{2-3 paragraphs introducing the chapter topic}
{Connect to previous chapters and overall module goals}
{Preview what learners will accomplish}

### Prerequisites

Before starting this chapter, ensure you have:
- {Prerequisite 1 from earlier modules/chapters}
- {Prerequisite 2}
- {Prerequisite 3}

### Learning Objectives

By the end of this chapter, you will be able to:
1. {Measurable objective 1}
2. {Measurable objective 2}
3. {Measurable objective 3}
4. {Measurable objective 4}

---

## Core Concept 1: {Topic}

### {Subtopic 1.1}

{Explanation with examples}

### {Subtopic 1.2}

{Explanation with examples}

{Include diagram reference where appropriate}
See [{Diagram Name}](../diagrams/module-4/{diagram-file}.md) for detailed visualization.

---

## Core Concept 2: {Topic}

{Same structure as Core Concept 1}

---

## Core Concept 3: {Topic}

{Same structure as Core Concept 1}

---

## Core Concept 4: {Topic}

{Same structure as Core Concept 1}

---

## Hands-On Example

{For chapters with code examples}

### Example: {Descriptive Name}

{Brief description of what the example demonstrates}

```python
# {filename}.py
{Code example - conceptual, well-commented}
```

**Key Points**:
- {Explanation of key code element 1}
- {Explanation of key code element 2}
- {Explanation of key code element 3}

---

## Summary

{Introductory sentence about what was covered}

1. **{Point 1 title}**: {Brief explanation}
2. **{Point 2 title}**: {Brief explanation}
3. **{Point 3 title}**: {Brief explanation}
4. **{Point 4 title}**: {Brief explanation}
5. **{Point 5 title}**: {Brief explanation}
6. **{Point 6 title}**: {Brief explanation}

**Next Steps**: {Preview of next chapter and how it builds on this one}

---

## Self-Assessment

Test your understanding of {chapter topic}:

1. **{Question 1}**
   <details>
   <summary>Show Answer</summary>
   {Detailed answer}
   </details>

2. **{Question 2}**
   <details>
   <summary>Show Answer</summary>
   {Detailed answer}
   </details>

{... repeat for 8 total questions}

8. **{Question 8}**
   <details>
   <summary>Show Answer</summary>
   {Detailed answer}
   </details>
```

---

## Content Guidelines

### Word Count
- Target: 2,000-3,000 words per chapter
- Core concepts: ~400-600 words each
- Summary: ~100-150 words
- Self-assessment: ~400 words total

### Diagrams
- Minimum 2 diagrams per chapter
- Referenced as relative links to diagrams/module-4/
- ASCII art format with detailed explanations

### Code Examples
- Minimum 1 per chapter
- Python 3.10+ syntax
- Conceptual/architectural focus
- Well-commented
- No external API dependencies

### Cross-References
- Link to relevant Module 1-3 content
- Link to other Module 4 chapters where appropriate
- Use relative paths for all links

### Terminology
- Define technical terms on first use
- Consistent with Modules 1-3 terminology
- ROS 2 concepts use official naming

---

## Validation Checklist

Before submitting a chapter, verify:

- [ ] Front matter includes sidebar_position and title
- [ ] Introduction connects to module goals
- [ ] Prerequisites listed
- [ ] 4-5 learning objectives (measurable)
- [ ] 4-5 core concepts with subheadings
- [ ] At least 2 diagram references
- [ ] At least 1 code example (where applicable)
- [ ] Exactly 6 summary bullet points
- [ ] Exactly 8 self-assessment questions with answers
- [ ] All internal links resolve
- [ ] No angle brackets that would break MDX (use &lt; &gt;)
- [ ] Word count in 2,000-3,000 range
