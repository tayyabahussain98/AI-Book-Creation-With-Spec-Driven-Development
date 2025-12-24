# Implementation Plan: Module 4 - Vision-Language-Action (VLA)

**Branch**: `004-vla-module` | **Date**: 2025-12-24 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/004-vla-module/spec.md`

## Summary

Module 4 extends the Physical AI book with Vision-Language-Action (VLA) content, enabling learners to design systems that convert natural language commands into robot actions through multimodal fusion. The implementation creates 5 Docusaurus chapters with diagrams and code examples, following the established patterns from Modules 1-3. Content is simulation-first, LLM-driven, and ROS 2-aligned.

## Technical Context

**Language/Version**: Markdown (CommonMark), TypeScript (Docusaurus config), Python 3.10+ (conceptual examples)
**Primary Dependencies**: Docusaurus 3.9.2 (existing), Modules 1-3 content (prerequisites)
**Storage**: Static files (Markdown chapters, diagram specifications, conceptual code snippets)
**Testing**: npm run build (Docusaurus validation), markdown lint, internal link verification
**Target Platform**: GitHub Pages via Docusaurus static site generation
**Project Type**: Documentation module extending existing book structure
**Performance Goals**: Build time <60 seconds for full site, all chapters accessible
**Constraints**: Simulation-first (no hardware required), open-source model references, free-tier compatible
**Scale/Scope**: 5 chapters (~12,000 words), 10+ diagrams, 5+ code examples

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence |
|-----------|--------|----------|
| I. Spec-Driven Development | ✅ PASS | Spec completed and validated before plan |
| II. Source-Grounded Accuracy | ✅ PASS | Content references established VLA research, no hallucinated APIs |
| III. Verifiable and Runnable Code | ✅ PASS | Code examples are conceptual/architectural, clearly documented |
| IV. Clear Separation of Concerns | ✅ PASS | Chapters in docs/, specs in specs/, diagrams in diagrams/ |
| V. AI-Native Architecture | ✅ PASS | Module teaches LLM-driven robotics patterns |
| VI. Free-Tier and Security | ✅ PASS | No cloud API requirements, open-source model alternatives |
| VII. Smallest Viable Changes | ✅ PASS | Incremental chapter implementation, follows Module 1-3 patterns |

**Gate Result**: ✅ PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/004-vla-module/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output (chapter templates)
│   ├── chapter-template.md
│   └── diagram-template.md
├── checklists/
│   └── requirements.md  # Spec validation checklist
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
book_frontend/
├── docs/
│   ├── module-4-vla/                    # NEW: Module 4 chapters
│   │   ├── chapter-1-vla-foundations.md
│   │   ├── chapter-2-voice-to-action.md
│   │   ├── chapter-3-llm-planning.md
│   │   ├── chapter-4-multimodal-fusion.md
│   │   └── chapter-5-capstone.md
│   ├── diagrams/
│   │   └── module-4/                    # NEW: Module 4 diagrams
│   │       ├── vla-architecture-overview.md
│   │       ├── voice-pipeline.md
│   │       ├── intent-extraction-flow.md
│   │       ├── llm-planning-pipeline.md
│   │       ├── action-graph-structure.md
│   │       ├── multimodal-fusion-architecture.md
│   │       ├── visual-grounding-flow.md
│   │       ├── capstone-system-architecture.md
│   │       ├── safety-constraints-diagram.md
│   │       └── execution-monitoring.md
│   └── code-examples/
│       └── module-4/                    # NEW: Module 4 code examples
│           ├── voice_command_parser.py
│           ├── intent_extractor.py
│           ├── llm_task_planner.py
│           ├── multimodal_fusion_node.py
│           └── safety_monitor.py
├── sidebars.ts                          # MODIFY: Add Module 4 navigation
└── docusaurus.config.ts                 # NO CHANGE: Config unchanged
```

**Structure Decision**: Extends existing Docusaurus book structure with new module-4-vla directory, following the established pattern from Modules 1-3.

## Phase 0: Research Findings

### VLA Architecture Patterns

**Decision**: Three-layer VLA architecture (Perception → Planning → Execution)
**Rationale**: Standard pattern in robotics literature (RT-1, PaLM-E, SayCan), well-documented, maps cleanly to ROS 2 node structure
**Alternatives Considered**:
- End-to-end neural (rejected: harder to explain, less interpretable)
- Behavior trees only (rejected: limited LLM integration, less flexible)

### Speech Recognition for Robotics

**Decision**: OpenAI Whisper as primary example, with local alternatives mentioned
**Rationale**: Open-source, well-documented, good accuracy, available for local deployment
**Alternatives Considered**:
- Google Speech-to-Text (rejected: cloud-only, API costs)
- Vosk (mentioned as alternative for offline use)
- DeepSpeech (mentioned but deprecated)

### LLM for Task Planning

**Decision**: Conceptual coverage of prompting patterns, not specific model
**Rationale**: Rapidly evolving field, concepts transfer across models (GPT-4, Claude, LLaMA, Mistral)
**Alternatives Considered**:
- Single model focus (rejected: would become outdated quickly)
- Training custom model (rejected: out of scope per spec)

### Multimodal Fusion Strategies

**Decision**: Late fusion with confidence weighting as primary approach
**Rationale**: Easier to explain, debug, and implement; widely used in practice
**Alternatives Considered**:
- Early fusion (mentioned for comparison)
- Attention-based fusion (mentioned as advanced topic)

### ROS 2 Integration Patterns

**Decision**: Action servers for long-running tasks, topics for streaming data
**Rationale**: Consistent with Modules 1-3, standard ROS 2 patterns
**Alternatives Considered**:
- Services only (rejected: inappropriate for long-running actions)
- Custom protocols (rejected: reinventing wheel)

## Phase 1: Design Artifacts

### Data Model

See [data-model.md](data-model.md) for complete entity definitions.

**Core Entities**:
1. **VoiceCommand** - Raw and processed natural language input
2. **Intent** - Parsed action type with confidence score
3. **Entity** - Extracted parameters (object, location, attribute)
4. **ActionGraph** - DAG of primitive actions with dependencies
5. **PrimitiveAction** - Atomic robot capability with parameters
6. **SceneState** - Visual understanding of environment
7. **ExecutionState** - Runtime status and history
8. **SafetyConstraint** - Rules limiting robot behavior

### Chapter Contracts

See [contracts/](contracts/) for chapter and diagram templates.

**Chapter Structure** (consistent with Modules 1-3):
- Front matter (sidebar_position, title)
- Introduction with prerequisites
- Learning objectives (4-5 items)
- Core concepts (4-5 sections)
- Hands-on example (where applicable)
- Summary (exactly 6 bullet points)
- Self-assessment (8 questions with expandable answers)

### Diagram Specifications

**Per Chapter Allocation**:
- Chapter 1: 2 diagrams (VLA overview, embodied cognition layers)
- Chapter 2: 2 diagrams (voice pipeline, intent extraction flow)
- Chapter 3: 2 diagrams (LLM planning pipeline, action graph structure)
- Chapter 4: 2 diagrams (multimodal fusion, visual grounding)
- Chapter 5: 2 diagrams (capstone architecture, safety constraints)

**Total**: 10 diagrams minimum

### Code Examples

**Per Chapter Allocation**:
- Chapter 1: 1 example (VLA system skeleton)
- Chapter 2: 1 example (voice command parser)
- Chapter 3: 1 example (LLM task planner)
- Chapter 4: 1 example (multimodal fusion node)
- Chapter 5: 1 example (safety monitor)

**Total**: 5 code examples minimum

## Implementation Phases

### Phase 1: Setup (Tasks T001-T002)
- Create directory structure for Module 4
- Update sidebars.ts with Module 4 navigation

### Phase 2: Foundational Diagrams (Tasks T003-T005)
- Create VLA architecture overview diagram
- Create voice pipeline diagram
- Create LLM planning diagram

### Phase 3: Chapter 1 - VLA Foundations (Tasks T006-T015)
- Create embodied cognition diagram
- Create VLA system skeleton code example
- Write chapter content covering all FR-010 to FR-014

### Phase 4: Chapter 2 - Voice-to-Action (Tasks T016-T025)
- Create intent extraction flow diagram
- Create voice command parser code example
- Write chapter content covering all FR-020 to FR-024

### Phase 5: Chapter 3 - LLM Planning (Tasks T026-T035)
- Create action graph structure diagram
- Create LLM task planner code example
- Write chapter content covering all FR-030 to FR-034

### Phase 6: Chapter 4 - Multimodal Fusion (Tasks T036-T045)
- Create multimodal fusion architecture diagram
- Create visual grounding flow diagram
- Create multimodal fusion node code example
- Write chapter content covering all FR-040 to FR-044

### Phase 7: Chapter 5 - Capstone (Tasks T046-T055)
- Create capstone system architecture diagram
- Create safety constraints diagram
- Create execution monitoring diagram
- Create safety monitor code example
- Write chapter content covering all FR-050 to FR-054

### Phase 8: Polish & Validation (Tasks T056-T065)
- Verify build succeeds with 0 errors
- Test all internal links
- Validate all diagram references
- Check code syntax
- Verify sidebar navigation
- Confirm 6 summary points per chapter
- Validate self-assessment questions

## Risk Analysis

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| LLM concepts become outdated | Medium | High | Focus on principles, not specific models |
| Content too advanced for target audience | High | Medium | Build on Module 1-3 concepts explicitly |
| Diagram complexity | Low | Medium | Use consistent ASCII art style from Modules 1-3 |
| Build failures from MDX | Medium | Low | Escape angle brackets, validate early |

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Build success | 0 errors | npm run build |
| Chapter count | 5 | File count |
| Diagram count | ≥10 | File count |
| Code examples | ≥5 | File count |
| Self-assessment per chapter | 8 questions | Manual count |
| Summary points per chapter | 6 | Manual count |

## Post-Phase 1 Constitution Re-Check

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Spec-Driven Development | ✅ PASS | Plan follows spec requirements |
| II. Source-Grounded Accuracy | ✅ PASS | Research cites established patterns |
| III. Verifiable Code | ✅ PASS | Conceptual examples with clear documentation |
| IV. Separation of Concerns | ✅ PASS | Structure maintains clean boundaries |
| V. AI-Native Architecture | ✅ PASS | LLM patterns are core teaching content |
| VI. Free-Tier Compatible | ✅ PASS | No paid services required |
| VII. Smallest Changes | ✅ PASS | Incremental phase structure |

**Final Gate Result**: ✅ PASSED - Ready for /sp.tasks
