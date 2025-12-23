# Implementation Plan: Module 2 - The Digital Twin (Gazebo & Unity)

**Branch**: `002-digital-twin-module` | **Date**: 2025-12-22 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-digital-twin-module/spec.md` + user guidance: "Extend existing Docusaurus site, add Module 2 to sidebar, create 5 chapter .md files, simulation-focused, ROS 2-aligned"

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Module 2 extends the existing Docusaurus educational site with 5 new chapters teaching digital twin simulation for humanoid robots. Content covers Gazebo physics simulation, SDF robot modeling, sensor simulation (LiDAR, depth cameras, IMU), and Unity visualization with ROS 2 integration. Educational approach follows Module 1 pattern: concept-first explanations, hands-on examples, self-assessment checklists.

## Technical Context

**Language/Version**: Markdown (CommonMark), XML (SDF/World files for examples), TypeScript (Docusaurus config)
**Primary Dependencies**: Docusaurus 3.9.2 (existing), Module 1 content (prerequisite), ROS 2 Humble/Iron (reader environment)
**Storage**: Static files (Markdown chapters, diagram specifications, example SDF/world files)
**Testing**: Docusaurus build validation, markdown linting, internal link checking
**Target Platform**: GitHub Pages (static site hosting), cross-platform readers (Ubuntu 22.04/24.04 for Gazebo, Windows/Mac/Linux for Unity)
**Project Type**: Documentation extension (adding Module 2 to existing book_frontend/)
**Performance Goals**: Build time under 60 seconds, page load under 2 seconds, responsive navigation
**Constraints**: Markdown-only content, no video/interactive demos, concept-first minimal code, weekly pacing (5 chapters = 5 weeks), simulation-only (no hardware)
**Scale/Scope**: 5 chapters (~600-800 lines each), 8-10 diagram specifications, 3-5 example files (SDF models, world files, sensor configs), self-assessment checklists per chapter

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Spec-Driven Development ✅ PASS
- Specification exists at specs/002-digital-twin-module/spec.md
- Implementation plan follows spec requirements (16 functional requirements mapped to 5 chapters)
- No deviation from spec: all chapters align with user stories US1-US5

### II. Source-Grounded Accuracy ✅ PASS
- Educational content will be grounded in official Gazebo and Unity documentation
- No hallucinated APIs: all Gazebo plugins and Unity Robotics Hub features must be verified
- Simulation tool versions explicitly specified (Gazebo Garden/Harmonic, Unity 2022 LTS)

### III. Verifiable and Runnable Code ✅ PASS
- Minimal code examples (SDF, world files) will be tested in actual Gazebo environments
- All commands (ros2 launch, gz sim, Unity setup) must be verified before publication
- Example files (simple_world.sdf, sensor_config.sdf) will be runnable without modification

### IV. Clear Separation of Concerns ✅ PASS
- Specs: Module 2 specification separate from implementation (specs/002-digital-twin-module/)
- Documentation: Chapter markdown files in book_frontend/docs/module-2-digital-twin/
- Implementation: Docusaurus configuration updates (sidebars.ts), no backend code changes

### V. AI-Native Architecture ✅ PASS
- Educational content teaches AI-robot integration (sensor data for perception pipelines)
- Prepares learners for AI workflows in later modules (vision, manipulation, planning)
- No direct AI implementation in this module (simulation infrastructure only)

### VI. Free-Tier and Security Constraints ✅ PASS
- Docusaurus GitHub Pages deployment (free tier)
- No secrets or credentials required (static site)
- Gazebo and Unity are free tools (open-source and free personal licenses)
- No paid dependencies introduced

### VII. Smallest Viable Changes ✅ PASS
- Atomic changes: one sidebar update, 5 chapter files, diagram specs
- Precise file references: book_frontend/docs/module-2-digital-twin/chapter-X.md
- No refactoring of Module 1 content
- Incremental: Module 2 builds on existing Docusaurus infrastructure

**Overall Gate Status**: ✅ ALL PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/002-digital-twin-module/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (Gazebo/Unity best practices)
├── data-model.md        # Phase 1 output (chapter/diagram entities)
├── quickstart.md        # Phase 1 output (development workflow)
├── contracts/           # Phase 1 output (chapter/diagram templates)
│   ├── chapter-template.md
│   ├── diagram-template.md
│   └── sdf-example-template.md
├── checklists/          # Spec validation (completed)
│   └── requirements.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book_frontend/
├── docs/
│   ├── module-1-ros2/              # Existing Module 1 (completed)
│   │   ├── chapter-1-intro-physical-ai.md
│   │   ├── chapter-2-ros2-architecture.md
│   │   ├── chapter-3-python-nodes.md
│   │   ├── chapter-4-packages-launch.md
│   │   └── chapter-5-urdf.md
│   ├── module-2-digital-twin/      # NEW: Module 2 chapters
│   │   ├── chapter-1-foundations.md
│   │   ├── chapter-2-gazebo-physics.md
│   │   ├── chapter-3-robot-models.md
│   │   ├── chapter-4-sensor-simulation.md
│   │   └── chapter-5-unity-hri.md
│   ├── code-examples/
│   │   ├── module-1/               # Existing Module 1 examples
│   │   └── module-2/               # NEW: Module 2 examples
│   │       ├── simple_world.sdf
│   │       ├── humanoid_stable.sdf
│   │       ├── lidar_sensor.sdf
│   │       └── unity_bridge_config.xml
│   └── diagrams/
│       ├── module-1/               # Existing Module 1 diagrams
│       └── module-2/               # NEW: Module 2 diagrams
│           ├── digital-twin-lifecycle.md
│           ├── gazebo-architecture.md
│           ├── urdf-vs-sdf.md
│           ├── sensor-plugin-flow.md
│           ├── unity-ros-bridge.md
│           └── [5-8 more diagram specs]
├── sidebars.ts                      # UPDATE: Add Module 2 category
├── docusaurus.config.ts             # No changes needed
└── [other Docusaurus files unchanged]
```

**Structure Decision**: Extends existing Docusaurus documentation structure. Module 2 follows Module 1 pattern: chapters in `docs/module-2-digital-twin/`, code examples in `code-examples/module-2/`, diagrams in `diagrams/module-2/`. Sidebar configuration updated to add Module 2 category with 5 chapter entries. No backend or source code changes—purely documentation extension.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

*No violations - all constitution principles satisfied. Table not needed.*

## Phase 0: Research (COMPLETED)

**Status**: ✅ Complete

**Output**: research.md

**Key Decisions**:
1. **Gazebo Version**: Gazebo Harmonic (latest LTS, best ROS 2 integration)
2. **SDF vs URDF**: Teach both - URDF for structure, SDF for Gazebo-specific features
3. **Sensors**: Focus on 3 essential types (LiDAR, depth camera, IMU) for AI perception
4. **Unity Version**: Unity 2022 LTS with Unity Robotics Hub for Chapter 5
5. **Content Structure**: Mirror Module 1 pattern (introduction, core concepts, hands-on, diagrams, summary, checklist)

**Research Sources**:
- Gazebo documentation (gazebosim.org)
- Unity Robotics Hub (github.com/Unity-Technologies)
- ROS 2 Gazebo integration (ros_gz packages)
- Humanoid stability theory (ZMP, CoM concepts)

## Phase 1: Design & Contracts (COMPLETED)

**Status**: ✅ Complete

**Outputs**:
1. data-model.md - Content entity structure (Chapter, Diagram, Code Example, Module entities)
2. contracts/chapter-template.md - Standard chapter structure
3. contracts/diagram-template.md - Diagram specification format
4. contracts/sdf-example-template.md - SDF code example standards
5. quickstart.md - Development workflow guide

**Entity Definitions**:
- **Chapter**: Educational markdown file (~600-800 lines, 9 required sections)
- **Diagram**: Text-based visualization (ASCII art, tables, workflows)
- **Code Example**: Runnable SDF file (50-200 lines with annotations)
- **Module**: Organizational container for 5 chapters

**Content Inventory**:
- 5 chapters (~3,500 total lines)
- 10 diagrams (specifications)
- 6 code examples (~920 total lines)
- 1 sidebar configuration update

## File Structure (Detailed)

### New Files to Create

```text
book_frontend/
├── docs/
│   ├── module-2-digital-twin/                    # NEW DIRECTORY
│   │   ├── chapter-1-foundations.md              # ~700 lines
│   │   ├── chapter-2-gazebo-physics.md           # ~750 lines
│   │   ├── chapter-3-robot-models.md             # ~650 lines
│   │   ├── chapter-4-sensor-simulation.md        # ~800 lines
│   │   └── chapter-5-unity-hri.md                # ~600 lines
│   ├── code-examples/
│   │   └── module-2/                             # NEW DIRECTORY
│   │       ├── simple_world.sdf                  # ~100 lines
│   │       ├── humanoid_stable.sdf               # ~200 lines
│   │       ├── obstacles_world.sdf               # ~150 lines
│   │       ├── lidar_robot.sdf                   # ~180 lines
│   │       ├── depth_camera_robot.sdf            # ~170 lines
│   │       └── imu_config.sdf                    # ~120 lines
│   └── diagrams/
│       └── module-2/                             # NEW DIRECTORY
│           ├── digital-twin-lifecycle.md
│           ├── gazebo-architecture.md
│           ├── urdf-vs-sdf-comparison.md
│           ├── sdf-world-structure.md
│           ├── sensor-plugin-flow.md
│           ├── lidar-raycasting.md
│           ├── depth-camera-rendering.md
│           ├── imu-noise-model.md
│           ├── unity-ros-bridge.md
│           └── hri-interaction-patterns.md
└── sidebars.ts                                   # UPDATE (add Module 2 category)
```

### Files to Modify

**book_frontend/sidebars.ts**:
- Add Module 2 category after Module 1
- Configure 5 chapter entries
- Set collapsed: false for visibility

**Optional** (book_frontend/docs/intro.md):
- Update introduction to mention Module 2
- Add link to Module 2 start page

## Implementation Notes

### Chapter Development

**Follow Module 1 Pattern**:
- Introduction (~200 words)
- 3-5 Core Concepts (400-600 words each)
- Hands-On Example (300-500 words + code)
- 2-3 Diagram references
- 6-bullet summary
- 6-item self-assessment checklist

**Chapter-Specific Focus**:
1. **Chapter 1**: Conceptual foundation (no code, pure concepts)
2. **Chapter 2**: Gazebo world files (SDF examples, physics config)
3. **Chapter 3**: Robot models (URDF → SDF conversion, tuning)
4. **Chapter 4**: Sensor simulation (3 sensor types with noise models)
5. **Chapter 5**: Unity integration (ROS 2 bridge, visualization)

### Diagram Development

**Text-Based Visualizations**:
- ASCII art for workflows and architecture
- Markdown tables for comparisons
- Structured text for complex diagrams
- No image files or screenshots

**Diagram Coverage**:
- Each chapter references 2-3 diagrams
- Total 10 unique diagrams
- Some diagrams may be referenced by multiple chapters

### Code Example Development

**Testing Requirements**:
- Every SDF file must be tested in Gazebo Harmonic
- Run: `gz sim [filename]` and verify no errors
- Check physics behavior (no jittering, explosions, or warnings)
- Verify ROS 2 topics appear (for sensor examples)

**Annotation Requirements**:
- Header comment with purpose and usage
- Inline comments for every non-trivial element
- Explain units (m, rad, Nm, Hz, kg)
- Provide typical values with context

## Technical Decisions

### Gazebo Configuration

**Physics Engine**: DART (default for Gazebo Harmonic)
- More stable than Bullet for humanoid simulation
- Better joint constraint handling
- Active development and bug fixes

**Physics Timestep**: 0.001s (1kHz)
- Standard for humanoid robotics
- Balance between accuracy and performance
- Real-time factor should remain ~1.0

**Surface Friction**: 0.8 (indoor floor)
- Typical for tile, wood, or carpet
- Adjust to 0.3-0.5 for slippery surfaces
- Critical for humanoid stability

### Sensor Configuration

**LiDAR**:
- Update rate: 10Hz (standard for navigation)
- Horizontal: 640 samples, 360° coverage
- Vertical: 16 beams (mid-range LiDAR)
- Range: 0.1m to 30m
- Noise: Gaussian, stddev 1cm

**Depth Camera**:
- Resolution: 640×480 (VGA)
- Update rate: 30Hz (video framerate)
- FoV: 60° horizontal, 45° vertical
- Range: 0.5m to 10m
- Noise: Gaussian on depth, stddev 2mm at 1m

**IMU**:
- Update rate: 100Hz (standard for control)
- Accelerometer range: ±2g
- Gyroscope range: ±250°/s
- Noise: Gaussian, typical consumer-grade specs

### Unity Integration

**ROS-TCP-Connector**:
- TCP port: 10000 (default)
- Message serialization: JSON (simple) or binary (performance)
- Update rate: 30-60 FPS (sufficient for visualization)
- Topics: /joint_states (subscribe), /camera/image (publish)

## Risks and Mitigations

### Risk 1: Gazebo Version Mismatch
**Impact**: Examples may not work if readers use older Gazebo versions

**Mitigation**:
- Explicitly state "Gazebo Harmonic or newer required" in each chapter
- Provide installation instructions with exact commands
- Note compatibility with Gazebo Garden (previous LTS) where applicable

### Risk 2: Unity Learning Curve
**Impact**: Chapter 5 may be too complex for readers unfamiliar with Unity

**Mitigation**:
- Keep Unity content conceptual (minimal C# code)
- Focus on setup and configuration over programming
- Provide screenshots of Unity GUI steps in text descriptions
- Mark Unity chapter as optional for Gazebo-only learners

### Risk 3: Reality Gap Misunderstanding
**Impact**: Learners may assume simulation results transfer perfectly to hardware

**Mitigation**:
- Emphasize reality gap in every chapter
- Dedicate section in Chapter 1 to limitations
- Provide examples of what doesn't transfer (friction, delays, noise)
- Include edge case on debugging sim-to-real discrepancies

### Risk 4: Large Example Files
**Impact**: Long SDF files may overwhelm beginner readers

**Mitigation**:
- Keep examples under 200 lines
- Use modular includes where possible
- Provide simplified versions first, then expanded versions
- Heavy annotation makes long files readable

## Post-Implementation

### Validation Tasks
1. Build validation (0 errors)
2. Link checking (all diagrams referenced exist)
3. Code syntax validation (SDF files parse correctly)
4. Navigation testing (chapter sequencing works)

### Deployment
1. Merge 002-digital-twin-module → main
2. GitHub Pages auto-deploys from main branch
3. Verify Module 2 visible at: https://tayyabahussain98.github.io/AI-book-creation-with-spec-driven-development/

### Documentation
1. Create PHRs (Prompt History Records) for each implementation session
2. Update main README if needed
3. Mark tasks.md complete when all tasks done

## Summary

This plan extends the existing Docusaurus educational site with Module 2 content following the established Module 1 pattern. Implementation focuses on educational content creation (markdown chapters, diagrams, examples) with no application code or backend changes. All constitution principles satisfied. Ready for task breakdown via `/sp.tasks`.

