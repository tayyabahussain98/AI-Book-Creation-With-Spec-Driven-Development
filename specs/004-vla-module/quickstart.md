# Quickstart: Module 4 - Vision-Language-Action (VLA)

**Feature**: 004-vla-module
**Purpose**: Quick reference for implementation team

---

## Overview

Module 4 adds Vision-Language-Action content to the Physical AI book, covering how robots convert natural language commands into physical actions through multimodal perception and LLM-based planning.

---

## Directory Structure

```text
book_frontend/
├── docs/
│   ├── module-4-vla/           # 5 chapter files
│   ├── diagrams/module-4/      # 10 diagram specs
│   └── code-examples/module-4/ # 5 Python examples
└── sidebars.ts                 # Add Module 4 navigation

specs/004-vla-module/
├── spec.md          # Requirements
├── plan.md          # Implementation plan
├── research.md      # Technical decisions
├── data-model.md    # Entity definitions
├── quickstart.md    # This file
├── contracts/       # Templates
└── tasks.md         # Task breakdown (after /sp.tasks)
```

---

## Implementation Checklist

### Phase 1: Setup
- [ ] Create `book_frontend/docs/module-4-vla/` directory
- [ ] Create `book_frontend/docs/diagrams/module-4/` directory
- [ ] Create `book_frontend/docs/code-examples/module-4/` directory
- [ ] Update `sidebars.ts` with Module 4 category

### Phase 2: Foundational Diagrams
- [ ] `vla-architecture-overview.md`
- [ ] `voice-pipeline.md`
- [ ] `llm-planning-pipeline.md`

### Phase 3-7: Chapters
| Chapter | File | Key Diagrams | Code Example |
|---------|------|--------------|--------------|
| 1 | chapter-1-vla-foundations.md | vla-architecture, embodied-cognition | vla_system_skeleton.py |
| 2 | chapter-2-voice-to-action.md | voice-pipeline, intent-extraction | voice_command_parser.py |
| 3 | chapter-3-llm-planning.md | llm-planning, action-graph | llm_task_planner.py |
| 4 | chapter-4-multimodal-fusion.md | multimodal-fusion, visual-grounding | multimodal_fusion_node.py |
| 5 | chapter-5-capstone.md | capstone-architecture, safety-constraints | safety_monitor.py |

### Phase 8: Validation
- [ ] `npm run build` succeeds
- [ ] All internal links resolve
- [ ] All diagram references valid
- [ ] Code syntax checked
- [ ] Each chapter has 6 summary points
- [ ] Each chapter has 8 self-assessment questions

---

## Key Constraints

1. **Markdown only** - No custom React components
2. **Simulation-first** - No hardware requirements
3. **Free-tier** - No paid cloud APIs
4. **ROS 2-aligned** - Use standard ROS 2 patterns
5. **LLM-agnostic** - Focus on principles, not specific models

---

## Sidebar Configuration

Add to `sidebars.ts`:

```typescript
{
  type: 'category',
  label: 'Module 4: Vision-Language-Action',
  items: [
    'module-4-vla/chapter-1-vla-foundations',
    'module-4-vla/chapter-2-voice-to-action',
    'module-4-vla/chapter-3-llm-planning',
    'module-4-vla/chapter-4-multimodal-fusion',
    'module-4-vla/chapter-5-capstone',
  ],
  collapsed: false,
},
```

---

## Content Word Counts

| Chapter | Target Words | Core Concepts |
|---------|-------------|---------------|
| 1 | 2,500 | VLA intro, embodied cognition, LLM roles |
| 2 | 2,500 | Speech recognition, intent extraction |
| 3 | 2,500 | Task decomposition, action graphs, ROS 2 |
| 4 | 2,500 | Multimodal fusion, visual grounding |
| 5 | 2,500 | End-to-end system, safety, evaluation |
| **Total** | **~12,500** | |

---

## Diagram Specifications

10 diagrams minimum, ASCII art format:
- Architecture diagrams: System overviews
- Flow diagrams: Data/process sequences
- Comparison tables: Approach trade-offs
- Entity diagrams: Data model relationships

---

## Code Example Requirements

5 Python files, each:
- Conceptual/architectural focus
- Well-commented
- No external API dependencies
- Python 3.10+ syntax
- ROS 2 node patterns where applicable

---

## Validation Commands

```bash
# Build check
cd book_frontend && npm run build

# Link validation (manual)
grep -r "](../" docs/module-4-vla/ | head -20

# Angle bracket check (MDX safety)
grep -n '<[0-9]' docs/module-4-vla/*.md

# Word count
wc -w docs/module-4-vla/*.md
```

---

## Success Criteria

| Metric | Target |
|--------|--------|
| Build errors | 0 |
| Chapters | 5 |
| Diagrams | ≥10 |
| Code examples | ≥5 |
| Summary points/chapter | 6 |
| Self-assessment/chapter | 8 |
| Total words | ~12,500 |

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown
2. Execute tasks in order (T001 → T065)
3. Validate with `npm run build` after each phase
4. Commit after each chapter completion
