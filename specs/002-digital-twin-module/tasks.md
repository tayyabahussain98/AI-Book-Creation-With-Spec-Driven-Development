# Tasks: Module 2 - The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-digital-twin-module/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Tests**: No automated tests required (educational content, validated via build and manual review)

**Organization**: Tasks are grouped by user story (US1-US5) to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `book_frontend/docs/module-2-digital-twin/`
- **Code Examples**: `book_frontend/docs/code-examples/module-2/`
- **Diagrams**: `book_frontend/docs/diagrams/module-2/`
- **Configuration**: `book_frontend/sidebars.ts`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Extend Docusaurus site with Module 2 structure

- [X] T001 Create directory book_frontend/docs/module-2-digital-twin/
- [X] T002 Create directory book_frontend/docs/code-examples/module-2/
- [X] T003 Create directory book_frontend/docs/diagrams/module-2/
- [X] T004 Update book_frontend/sidebars.ts to add Module 2 category with 5 chapter entries (chapter-1-foundations, chapter-2-gazebo-physics, chapter-3-robot-models, chapter-4-sensor-simulation, chapter-5-unity-hri)
- [X] T005 Create placeholder files for all 5 chapters (front matter with sidebar_position and title only)
- [X] T006 Test dev server starts successfully: npm start in book_frontend/
- [X] T007 Verify Module 2 appears in sidebar navigation

**Checkpoint**: Docusaurus configuration complete - content creation can now begin

---

## Phase 2: Foundational Diagrams (Blocking Prerequisites)

**Purpose**: Create diagram specifications that will be referenced by multiple chapters

**⚠️ CRITICAL**: Diagrams must be complete before chapter writing begins (chapters reference diagrams)

- [X] T008 [P] Create book_frontend/docs/diagrams/module-2/digital-twin-lifecycle.md diagram specification (design → simulate → test → deploy → sync workflow)
- [X] T009 [P] Create book_frontend/docs/diagrams/module-2/gazebo-architecture.md diagram specification (physics engine, rendering, plugins, ROS 2 bridge layers)
- [X] T010 [P] Create book_frontend/docs/diagrams/module-2/urdf-vs-sdf-comparison.md diagram specification (comparison table with 8-10 dimensions)

**Checkpoint**: Core diagrams ready - chapter writing can reference these foundational diagrams

---

## Phase 3: User Story 1 - Understanding Digital Twin Foundations (Priority: P1) 🎯 MVP

**Goal**: Learner understands digital twin concept, tool selection (Gazebo vs Unity), and simulation limitations

**Independent Test**: Learner reads Chapter 1, completes conceptual exercises, and correctly identifies when to use Gazebo vs Unity for different use cases

### Diagrams for User Story 1

- [X] T011 [US1] Verify digital-twin-lifecycle.md and gazebo-architecture.md exist (created in Phase 2)
- [X] T012 [P] [US1] Create book_frontend/docs/diagrams/module-2/simulation-vs-reality.md diagram specification (reality gap visualization, what transfers and what doesn't)

### Chapter Content for User Story 1

- [X] T013 [US1] Create book_frontend/docs/module-2-digital-twin/chapter-1-foundations.md with front matter (sidebar_position: 1)
- [X] T014 [US1] Write Chapter 1 Introduction (prerequisites: Module 1 complete, learning objectives: digital twin concept, tool selection, limitations)
- [X] T015 [US1] Write Chapter 1 Core Concept 1: Digital Twins in Robotics (definition, distinguishing digital twin vs simulation vs emulation)
- [X] T016 [US1] Write Chapter 1 Core Concept 2: Gazebo vs Unity Use Cases (comparison table, decision criteria, when to use each tool)
- [X] T017 [US1] Write Chapter 1 Core Concept 3: Simulation-to-Reality Gap (friction models, sensor noise, motor delays, transfer limitations)
- [X] T018 [US1] Write Chapter 1 Core Concept 4: Computational Constraints (real-time factor, GPU requirements, simulation complexity trade-offs)
- [X] T019 [US1] Reference digital-twin-lifecycle, gazebo-architecture, and simulation-vs-reality diagrams in Chapter 1
- [X] T020 [US1] Write Chapter 1 Summary (6 bullet points: digital twin purpose, Gazebo focus, Unity focus, reality gap, tool selection, simulation workflow)
- [X] T021 [US1] Write Chapter 1 Self-Assessment Checklist (6 items: explain digital twin, choose tools, identify limitations)

**Checkpoint**: Chapter 1 complete - learner has conceptual foundation for simulation

---

## Phase 4: User Story 2 - Setting Up Physics-Based Environments (Priority: P2)

**Goal**: Learner creates Gazebo world files with custom physics properties and spawns robots

**Independent Test**: Learner creates a world file with custom gravity, ground plane, obstacles, spawns URDF robot, observes physics behavior

### Code Example for User Story 2

- [X] T022 [US2] Create book_frontend/docs/code-examples/module-2/simple_world.sdf (basic Gazebo world: physics config, lighting, ground plane with friction, comments explaining each element)

### Diagrams for User Story 2

- [X] T023 [P] [US2] Create book_frontend/docs/diagrams/module-2/sdf-world-structure.md diagram specification (anatomy of SDF world file: physics, models, lighting, scene graph)
- [X] T024 [P] [US2] Create book_frontend/docs/diagrams/module-2/physics-engine-comparison.md diagram specification (DART vs Bullet vs ODE trade-offs)
- [X] T025 [P] [US2] Create book_frontend/docs/diagrams/module-2/humanoid-stability-zmp.md diagram specification (Zero-Moment Point, Center of Mass, support polygon visualization)

### Chapter Content for User Story 2

- [X] T026 [US2] Create book_frontend/docs/module-2-digital-twin/chapter-2-gazebo-physics.md with front matter (sidebar_position: 2)
- [X] T027 [US2] Write Chapter 2 Introduction (prerequisites: Chapter 1, learning objectives: Gazebo world files, physics configuration, humanoid stability)
- [X] T028 [US2] Write Chapter 2 Core Concept 1: Gazebo World File Structure (SDF format, physics element, gravity vector, model inclusion)
- [X] T029 [US2] Write Chapter 2 Core Concept 2: Physics Engine Configuration (DART/Bullet/ODE selection, timestep, real-time factor, surface properties)
- [X] T030 [US2] Write Chapter 2 Core Concept 3: Humanoid Stability Fundamentals (ZMP, CoM, support polygon, static vs dynamic stability)
- [X] T031 [US2] Write Chapter 2 Core Concept 4: Environmental Design (ground plane friction, obstacles, slopes, stairs for testing)
- [X] T032 [US2] Write Chapter 2 Hands-On Example: Creating a Gazebo World (embed simple_world.sdf, annotate sections, show gz sim command)
- [X] T033 [US2] Reference sdf-world-structure, physics-engine-comparison, and humanoid-stability-zmp diagrams in Chapter 2
- [X] T034 [US2] Write Chapter 2 Summary (6 bullet points: world file structure, physics config, stability metrics, environmental design)
- [X] T035 [US2] Write Chapter 2 Self-Assessment Checklist (6 items: create world file, configure physics, explain ZMP)

**Checkpoint**: Chapter 2 complete - learner can create Gazebo environments

---

## Phase 5: User Story 3 - Spawning and Tuning Robot Models (Priority: P3)

**Goal**: Learner converts URDF to SDF, spawns robots in Gazebo, tunes parameters for stability

**Independent Test**: Learner converts Module 1 URDF to SDF, spawns in Gazebo, identifies and fixes joint/collision issues

### Code Example for User Story 3

- [X] T036 [US3] Create book_frontend/docs/code-examples/module-2/humanoid_stable.sdf (balanced bipedal robot model with proper CoM, collision geometry, joint limits, damping)

### Diagrams for User Story 3

- [X] T037 [P] [US3] Verify urdf-vs-sdf-comparison.md exists (created in Phase 2)
- [X] T038 [P] [US3] Create book_frontend/docs/diagrams/module-2/sdf-conversion-workflow.md diagram specification (URDF → SDF conversion steps, gz sdf command, common issues)
- [X] T039 [P] [US3] Create book_frontend/docs/diagrams/module-2/collision-tuning.md diagram specification (collision margin adjustment, visual vs collision geometry debugging)

### Chapter Content for User Story 3

- [X] T040 [US3] Create book_frontend/docs/module-2-digital-twin/chapter-3-robot-models.md with front matter (sidebar_position: 3)
- [X] T041 [US3] Write Chapter 3 Introduction (prerequisites: Chapter 2, Module 1 Chapter 5 (URDF), learning objectives: URDF/SDF differences, spawning, tuning)
- [X] T042 [US3] Write Chapter 3 Core Concept 1: URDF vs SDF (format differences, capabilities comparison, when to use each, conversion workflow)
- [X] T043 [US3] Write Chapter 3 Core Concept 2: Spawning Robots in Gazebo (gz sim command, initial pose, model inclusion in worlds)
- [X] T044 [US3] Write Chapter 3 Core Concept 3: Joint and Collision Tuning (joint limits, damping, friction, collision margins, debugging warnings)
- [X] T045 [US3] Write Chapter 3 Core Concept 4: Stability Testing (gravity application, balance verification, CoM diagnosis)
- [X] T046 [US3] Write Chapter 3 Hands-On Example: Stable Humanoid Model (embed humanoid_stable.sdf, annotate CoM placement, show spawning and testing)
- [X] T047 [US3] Reference urdf-vs-sdf-comparison, sdf-conversion-workflow, and collision-tuning diagrams in Chapter 3
- [X] T048 [US3] Write Chapter 3 Summary (6 bullet points: URDF/SDF differences, conversion, spawning, tuning, stability)
- [X] T049 [US3] Write Chapter 3 Self-Assessment Checklist (6 items: distinguish formats, convert files, spawn robots, tune parameters)

**Checkpoint**: Chapter 3 complete - learner can import and configure robot models

---

## Phase 6: User Story 4 - Simulating Sensors for Perception Pipelines (Priority: P4)

**Goal**: Learner adds sensors (LiDAR, depth camera, IMU) to robots, applies noise models, integrates with ROS 2

**Independent Test**: Learner adds 3 sensor types to robot, subscribes to ROS 2 topics, visualizes in RViz2, applies noise models

### Code Examples for User Story 4

- [X] T050 [P] [US4] Create book_frontend/docs/code-examples/module-2/lidar_robot.sdf (robot model with GPU LiDAR sensor plugin, ROS 2 bridge, noise model, annotations)
- [X] T051 [P] [US4] Create book_frontend/docs/code-examples/module-2/depth_camera_robot.sdf (robot with depth camera sensor, RGB-D topics, camera intrinsics)
- [X] T052 [P] [US4] Create book_frontend/docs/code-examples/module-2/imu_config.sdf (IMU sensor configuration with accelerometer, gyroscope, noise parameters)

### Diagrams for User Story 4

- [X] T053 [P] [US4] Create book_frontend/docs/diagrams/module-2/sensor-plugin-flow.md diagram specification (Gazebo sensor plugin → ROS 2 topic → perception node data flow)
- [X] T054 [P] [US4] Create book_frontend/docs/diagrams/module-2/lidar-raycasting.md diagram specification (ray-casting algorithm, point cloud generation, resolution trade-offs)
- [X] T055 [P] [US4] Create book_frontend/docs/diagrams/module-2/depth-camera-rendering.md diagram specification (depth image generation, depth encoding formats, camera parameters)
- [X] T056 [P] [US4] Create book_frontend/docs/diagrams/module-2/imu-noise-model.md diagram specification (accelerometer/gyro noise characteristics, Gaussian distribution, bias drift)

### Chapter Content for User Story 4

- [X] T057 [US4] Create book_frontend/docs/module-2-digital-twin/chapter-4-sensor-simulation.md with front matter (sidebar_position: 4)
- [X] T058 [US4] Write Chapter 4 Introduction (prerequisites: Chapter 3, learning objectives: sensor types, noise models, ROS 2 integration)
- [X] T059 [US4] Write Chapter 4 Core Concept 1: LiDAR Simulation (ray-casting principles, point cloud generation, resolution/range trade-offs, ROS 2 topics)
- [X] T060 [US4] Write Chapter 4 Core Concept 2: Depth Camera Simulation (RGB-D image generation, depth encoding, camera intrinsics/extrinsics, calibration)
- [X] T061 [US4] Write Chapter 4 Core Concept 3: IMU Simulation (accelerometer, gyroscope, magnetometer, orientation estimation, noise characteristics)
- [X] T062 [US4] Write Chapter 4 Core Concept 4: Noise Models for Realism (Gaussian noise, systematic bias, sensor spec matching, why noise matters for AI)
- [X] T063 [US4] Write Chapter 4 Core Concept 5: ROS 2 Sensor Integration (ros_gz_bridge, topic naming, message types, visualization in RViz2)
- [X] T064 [US4] Write Chapter 4 Hands-On Example: Adding Sensors (embed lidar_robot.sdf and depth_camera_robot.sdf, show ROS 2 topic commands, visualize in RViz2)
- [X] T065 [US4] Reference sensor-plugin-flow, lidar-raycasting, depth-camera-rendering, and imu-noise-model diagrams in Chapter 4
- [X] T066 [US4] Write Chapter 4 Summary (6 bullet points: LiDAR, depth camera, IMU, noise models, ROS 2 integration)
- [X] T067 [US4] Write Chapter 4 Self-Assessment Checklist (6 items: configure sensors, apply noise, visualize data, integrate with ROS 2)

**Checkpoint**: Chapter 4 complete - learner can simulate sensors for AI pipelines

---

## Phase 7: User Story 5 - High-Fidelity Visualization in Unity (Priority: P5)

**Goal**: Learner sets up Unity with ROS 2 bridge, imports robot models, creates human-robot interaction scenarios

**Independent Test**: Learner sets up Unity, imports URDF, connects ROS 2 bridge, demonstrates robot responding to joint commands

### Code Example for User Story 5

- [X] T068 [US5] Create book_frontend/docs/code-examples/module-2/unity_bridge_config.xml (ROS-TCP-Connector configuration example with endpoint, topics, message types)

### Diagrams for User Story 5

- [X] T069 [P] [US5] Create book_frontend/docs/diagrams/module-2/unity-ros-bridge.md diagram specification (Unity-ROS 2 architecture: TCP connector, message serialization, joint state synchronization)
- [X] T070 [P] [US5] Create book_frontend/docs/diagrams/module-2/hri-interaction-patterns.md diagram specification (common human-robot interaction scenarios: object handoff, gesture recognition, social navigation)
- [X] T071 [P] [US5] Create book_frontend/docs/diagrams/module-2/unity-urdf-import.md diagram specification (URDF import workflow in Unity, ArticulationBody mapping, collision setup)

### Chapter Content for User Story 5

- [X] T072 [US5] Create book_frontend/docs/module-2-digital-twin/chapter-5-unity-hri.md with front matter (sidebar_position: 5)
- [X] T073 [US5] Write Chapter 5 Introduction (prerequisites: Chapter 4, learning objectives: Unity setup, ROS 2 bridge, human-robot interaction)
- [X] T074 [US5] Write Chapter 5 Core Concept 1: Unity Robotics Hub (installation, ROS-TCP-Connector, URDF Importer, ArticulationBody physics)
- [X] T075 [US5] Write Chapter 5 Core Concept 2: Importing Robot Models (URDF → Unity workflow, joint mapping, collision configuration, material setup)
- [X] T076 [US5] Write Chapter 5 Core Concept 3: ROS 2 Bridge Configuration (TCP endpoint, topic subscription/publication, message serialization, latency considerations)
- [X] T077 [US5] Write Chapter 5 Core Concept 4: Human-Robot Interaction Scenarios (NPC humans, object handoff, gesture recognition, social navigation examples)
- [X] T078 [US5] Write Chapter 5 Hands-On Example: Unity Setup (embed unity_bridge_config.xml, show URDF import steps, demonstrate joint control from ROS 2)
- [X] T079 [US5] Reference unity-ros-bridge, hri-interaction-patterns, and unity-urdf-import diagrams in Chapter 5
- [X] T080 [US5] Write Chapter 5 Summary (6 bullet points: Unity setup, URDF import, ROS 2 bridge, HRI scenarios, visualization benefits)
- [X] T081 [US5] Write Chapter 5 Self-Assessment Checklist (6 items: set up Unity, import robot, configure bridge, create HRI scenario)

**Checkpoint**: Chapter 5 complete - learner can use Unity for high-fidelity visualization

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Validate all content, ensure build succeeds, prepare for deployment

- [X] T082 [P] Verify all 5 chapters build without errors using npm run build in book_frontend/
- [X] T083 [P] Test navigation between Module 2 chapters (previous/next links working)
- [X] T084 [P] Verify all diagram references point to existing files in diagrams/module-2/
- [X] T085 [P] Check all internal links in Module 2 markdown files resolve correctly
- [X] T086 [P] Validate all SDF example files are syntactically valid (gz sdf --check for each file)
- [X] T087 Test local deployment with npm run serve (production build at localhost:3000)
- [X] T088 Verify Module 2 appears in sidebar below Module 1 with correct ordering
- [X] T089 Check all self-assessment checklists have exactly 6 items
- [X] T090 Verify all chapter summaries have exactly 6 bullet points
- [X] T091 Run production build final validation: npm run build should complete in <60 seconds with 0 errors

**Final Checkpoint**: Module 2 ready for deployment to GitHub Pages

---

## Dependencies Between User Stories

### Story Dependency Graph

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational Diagrams)
    ↓
Phase 3 (US1) ← Independent (no dependencies on other stories)
    ↓
Phase 4 (US2) ← Depends on: Phase 2 diagrams
    ↓
Phase 5 (US3) ← Depends on: Phase 2 diagrams, conceptually builds on US2
    ↓
Phase 6 (US4) ← Depends on: US3 (robot models needed for sensors)
    ↓
Phase 7 (US5) ← Depends on: US3 (robot models needed for Unity import)
    ↓
Phase 8 (Polish)
```

### Parallel Execution Opportunities

**Phase 2** (Foundational Diagrams):
- All 3 diagrams can be created in parallel: T008, T009, T010

**Phase 4** (US2 Diagrams):
- Diagrams can be created in parallel: T023, T024, T025

**Phase 6** (US4 Code Examples):
- All 3 sensor examples can be created in parallel: T050, T051, T052

**Phase 6** (US4 Diagrams):
- All 4 sensor diagrams can be created in parallel: T053, T054, T055, T056

**Phase 7** (US5 Diagrams):
- All 3 Unity diagrams can be created in parallel: T069, T070, T071

**Phase 8** (Polish):
- All validation tasks can run in parallel: T082, T083, T084, T085, T086

### Independent Testing Per Story

**US1** (Chapter 1):
- Test: Learner reads chapter, identifies Gazebo vs Unity use cases
- No dependencies: Can be tested immediately after chapter written

**US2** (Chapter 2):
- Test: Learner creates simple_world.sdf, runs gz sim simple_world.sdf
- Dependencies: Requires Gazebo Harmonic installed (reader environment)

**US3** (Chapter 3):
- Test: Learner converts Module 1 simple_arm.urdf to SDF, spawns in Gazebo
- Dependencies: Requires US2 (Gazebo world knowledge) and Module 1 URDF

**US4** (Chapter 4):
- Test: Learner adds LiDAR to robot, runs ros2 topic list, visualizes in RViz2
- Dependencies: Requires US3 (robot models to attach sensors to)

**US5** (Chapter 5):
- Test: Learner imports URDF into Unity, connects ROS 2 bridge, controls joints
- Dependencies: Requires US3 (robot models) and Unity installed

---

## Implementation Strategy

### MVP (Minimum Viable Product)

**MVP = User Story 1 (Chapter 1) Only**:
- Delivers conceptual foundation
- Learners understand digital twin value proposition
- Can make informed decisions about Gazebo vs Unity
- No code examples required (pure concepts)
- Fastest path to value (1 chapter, 2-3 diagrams)

### Incremental Delivery

**Iteration 1**: US1 (Chapter 1) - Conceptual foundation
**Iteration 2**: US1 + US2 (Chapters 1-2) - Gazebo basics
**Iteration 3**: US1 + US2 + US3 (Chapters 1-3) - Robot modeling
**Iteration 4**: US1-US4 (Chapters 1-4) - Sensor simulation
**Iteration 5**: US1-US5 (All 5 chapters) - Complete module with Unity

### Task Execution Order

**Sequential Tasks** (must complete in order):
- Setup (T001-T007) before any content
- Foundational diagrams (T008-T010) before chapters
- Chapter prerequisites before chapter content (T013 before T014-T021)

**Parallel Tasks** (can execute simultaneously):
- Multiple diagrams in same phase
- Multiple code examples in same phase
- Validation tasks in Phase 8

**File-Based Coordination**:
- Tasks modifying same file must run sequentially
- Tasks creating different files can run in parallel (marked with [P])

---

## Validation Criteria

### Per-Chapter Validation

Each chapter must have:
- [ ] Front matter (sidebar_position, title)
- [ ] Introduction (prerequisites, objectives)
- [ ] 3-5 Core Concepts (400-600 words each)
- [ ] At least 1 Hands-On Example (for Chapters 2-5)
- [ ] 2-3 Diagram references
- [ ] 6-bullet summary
- [ ] 6-item self-assessment checklist
- [ ] No broken links
- [ ] Builds without errors

### Per-Diagram Validation

Each diagram must have:
- [ ] Purpose statement
- [ ] Diagram type specified
- [ ] Text-based visual representation
- [ ] Detailed explanation section
- [ ] Usage in book section
- [ ] Referenced by at least one chapter

### Per-Example Validation

Each code example must have:
- [ ] Header comment (purpose, usage, key concepts)
- [ ] Syntactically valid XML/SDF
- [ ] Inline annotations
- [ ] Tested in Gazebo Harmonic (for SDF files)
- [ ] Embedded or referenced in at least one chapter

---

## Task Summary

**Total Tasks**: 91

### Breakdown by Phase:
- **Phase 1** (Setup): 7 tasks
- **Phase 2** (Foundational): 3 tasks
- **Phase 3** (US1): 11 tasks
- **Phase 4** (US2): 14 tasks
- **Phase 5** (US3): 14 tasks
- **Phase 6** (US4): 18 tasks
- **Phase 7** (US5): 14 tasks
- **Phase 8** (Polish): 10 tasks

### Breakdown by Type:
- **Configuration**: 2 tasks (directories, sidebar)
- **Diagrams**: 13 tasks
- **Code Examples**: 6 tasks
- **Chapter Content**: 60 tasks (5 chapters × ~12 tasks each)
- **Validation**: 10 tasks

### Parallel Opportunities:
- **Phase 2**: 3 tasks can run in parallel
- **Phase 4**: 3 diagrams in parallel
- **Phase 6**: 3 examples + 4 diagrams = 7 tasks in parallel
- **Phase 7**: 3 diagrams in parallel
- **Phase 8**: 5 validation tasks in parallel

**Total Parallelizable**: ~20 tasks (22% of total)

---

## Success Criteria

Module 2 implementation is complete when:

1. **All 91 tasks marked [X]** in this file
2. **All 5 chapters exist** and build successfully
3. **All 13 diagrams created** and referenced by chapters
4. **All 6 code examples created** and tested (SDF syntax valid)
5. **Sidebar navigation updated** with Module 2 category
6. **Production build successful**: 0 errors, <60 seconds
7. **All validation tasks pass** (T082-T091)

**Deployment Ready**: After all tasks complete, Module 2 can be merged to main and deployed to GitHub Pages.
