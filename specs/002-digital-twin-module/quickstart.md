# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: 002-digital-twin-module
**Date**: 2025-12-22
**Purpose**: Development workflow for creating Module 2 educational content

## Prerequisites

Before implementing Module 2 content:

1. **Module 1 Complete**: All 5 chapters of Module 1 (ROS 2 fundamentals) must be implemented and deployed
2. **Docusaurus Running**: book_frontend/ site must build successfully
3. **Gazebo Installed**: Gazebo Harmonic on Ubuntu 22.04/24.04 for testing examples
4. **Unity Installed** (for Chapter 5): Unity Hub with Unity 2022 LTS and Robotics Hub package
5. **ROS 2 Environment**: Humble or Iron distribution sourced and functional

## Development Workflow

### Step 1: Verify Branch

```bash
git branch --show-current
# Should output: 002-digital-twin-module
```

If not on correct branch:
```bash
git checkout 002-digital-twin-module
```

### Step 2: Review Planning Artifacts

Read in order:
1. `specs/002-digital-twin-module/spec.md` - Feature requirements
2. `specs/002-digital-twin-module/plan.md` - Implementation plan
3. `specs/002-digital-twin-module/research.md` - Technical decisions
4. `specs/002-digital-twin-module/data-model.md` - Content structure
5. `specs/002-digital-twin-module/contracts/` - Templates

### Step 3: Generate Task Breakdown

```bash
# Run from repo root
/sp.tasks
```

This creates `specs/002-digital-twin-module/tasks.md` with detailed task list.

### Step 4: Implementation Phases

Following the task breakdown:

#### Phase 1: Docusaurus Configuration
- Update `book_frontend/sidebars.ts` to add Module 2 category
- Create placeholder chapter files in `docs/module-2-digital-twin/`
- Verify dev server starts: `cd book_frontend && npm start`

#### Phase 2: Diagram Specifications
- Create 10 diagram .md files in `docs/diagrams/module-2/`
- Follow `contracts/diagram-template.md` structure
- Use ASCII art, tables, and text descriptions (no images)

#### Phase 3: Code Examples
- Create 6 SDF example files in `docs/code-examples/module-2/`
- Follow `contracts/sdf-example-template.md` structure
- Test each example in Gazebo Harmonic: `gz sim [filename]`
- Verify no physics warnings or errors

#### Phase 4: Chapter Content
- Write 5 chapters (~600-800 lines each)
- Follow `contracts/chapter-template.md` structure
- Embed code examples and reference diagrams
- Include 6-bullet summary and 6-item checklist per chapter

#### Phase 5: Validation & Polish
- Build production site: `npm run build`
- Test navigation between chapters
- Validate all internal links
- Check self-assessment checklists
- Run final build validation

### Step 5: Testing

#### Docusaurus Build Test
```bash
cd book_frontend
npm run build
# Should complete with 0 errors in <60 seconds
```

#### SDF Syntax Validation
```bash
# Test each SDF file
gz sdf --check code-examples/module-2/simple_world.sdf
```

#### Link Validation
```bash
# Check all diagram references resolve
grep -r "/docs/diagrams/module-2/" docs/module-2-digital-twin/*.md
ls docs/diagrams/module-2/
# Compare output - all references should have matching files
```

### Step 6: Deployment

```bash
# Build and serve locally
npm run build
npm run serve

# Visit http://localhost:3000 and navigate to Module 2
# Verify all chapters load correctly
```

## File Creation Order

**Optimal sequence** (respects dependencies):

1. **Diagrams first** (referenced by chapters):
   - digital-twin-lifecycle.md
   - gazebo-architecture.md
   - urdf-vs-sdf-comparison.md
   - sdf-world-structure.md
   - sensor-plugin-flow.md
   - lidar-raycasting.md
   - depth-camera-rendering.md
   - imu-noise-model.md
   - unity-ros-bridge.md
   - hri-interaction-patterns.md

2. **Code examples second** (embedded in chapters):
   - simple_world.sdf
   - humanoid_stable.sdf
   - obstacles_world.sdf
   - lidar_robot.sdf
   - depth_camera_robot.sdf
   - imu_config.sdf

3. **Chapters third** (consume diagrams and examples):
   - chapter-1-foundations.md
   - chapter-2-gazebo-physics.md
   - chapter-3-robot-models.md
   - chapter-4-sensor-simulation.md
   - chapter-5-unity-hri.md

4. **Configuration last**:
   - Update sidebars.ts (add Module 2 category)

## Common Development Tasks

### Add a New Diagram

```bash
# 1. Create diagram file
touch docs/diagrams/module-2/[diagram-name].md

# 2. Fill using contracts/diagram-template.md structure

# 3. Reference from chapter:
# See the [Diagram Name](/docs/diagrams/module-2/[diagram-name].md) for...
```

### Add a New Code Example

```bash
# 1. Create example file
touch docs/code-examples/module-2/[example-name].sdf

# 2. Fill using contracts/sdf-example-template.md structure

# 3. Test in Gazebo
gz sim docs/code-examples/module-2/[example-name].sdf

# 4. Embed in chapter using markdown code block:
```xml
[paste file contents with syntax highlighting]
```
```

### Update Sidebar Navigation

Edit `book_frontend/sidebars.ts`:

```typescript
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [ /* Module 1 chapters */ ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/chapter-1-foundations',
        'module-2-digital-twin/chapter-2-gazebo-physics',
        'module-2-digital-twin/chapter-3-robot-models',
        'module-2-digital-twin/chapter-4-sensor-simulation',
        'module-2-digital-twin/chapter-5-unity-hri',
      ],
      collapsed: false,
    },
  ],
};
```

## Quality Checklist

Before marking Module 2 complete:

### Content Quality
- [ ] All 5 chapters follow template structure
- [ ] All 10 diagrams created and referenced
- [ ] All 6 code examples tested in Gazebo
- [ ] Sidebar navigation updated and functional
- [ ] No broken internal links
- [ ] All chapters have 6-bullet summary + 6-item checklist

### Technical Accuracy
- [ ] All Gazebo commands tested in Harmonic
- [ ] All SDF examples parse without errors
- [ ] Sensor plugins verified to work with ROS 2
- [ ] Unity Robotics Hub version confirmed (2022 LTS)
- [ ] No hallucinated APIs or non-existent features

### Build & Deploy
- [ ] Production build successful (0 errors)
- [ ] All chapters render correctly in browser
- [ ] Navigation between chapters functional
- [ ] Module 2 appears in sidebar below Module 1
- [ ] Code syntax highlighting works

## Troubleshooting

### Gazebo Not Installed

If Gazebo Harmonic is not available for testing examples:
```bash
# Ubuntu 24.04
sudo apt update
sudo apt install gz-harmonic

# Verify installation
gz sim --version
```

### SDF Syntax Errors

If SDF file has errors:
```bash
# Validate syntax
gz sdf --check [filename.sdf]

# Common issues:
# - Missing closing tags
# - Invalid element nesting
# - Unsupported SDF version (use 1.9)
```

### Build Fails

If Docusaurus build fails:
```bash
# Clear cache
rm -rf book_frontend/.docusaurus book_frontend/build

# Rebuild
cd book_frontend && npm run build

# Check error messages for:
# - Broken markdown links
# - Invalid front matter
# - MDX/JSX syntax issues (avoid < > in text)
```

### Unity Setup Issues

If Unity Robotics Hub installation fails:
1. Open Unity Hub
2. Install Unity 2022 LTS (latest patch version)
3. Open project
4. Window → Package Manager
5. Click "+" → Add package from git URL
6. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

## Time Estimates

**Phase 1** (Diagrams): 2-3 hours
- 10 diagrams × 15-20 minutes each
- ASCII art creation and explanation

**Phase 2** (Code Examples): 3-4 hours
- 6 SDF files × 30-40 minutes each
- Testing in Gazebo and annotation

**Phase 3** (Chapters): 10-15 hours
- 5 chapters × 2-3 hours each
- Research, writing, editing

**Phase 4** (Polish): 2-3 hours
- Build validation, link checking, final review

**Total**: 17-25 hours (following spec-driven workflow)

## Success Criteria

Module 2 implementation is complete when:

1. All 5 chapters build without errors
2. All 10 diagrams exist and are referenced
3. All 6 code examples tested and annotated
4. Sidebar shows Module 2 with correct navigation
5. Production build successful (0 errors, <60s)
6. All self-assessment checklists present
7. PHR (Prompt History Records) created for each implementation session

## Next Commands

After implementation complete:

```bash
# 1. Commit changes
git add .
git commit -m "feat: Complete Module 2 - Digital Twin (Gazebo & Unity)"

# 2. Push to GitHub
git push origin 002-digital-twin-module

# 3. Create Pull Request
gh pr create --title "Module 2: Digital Twin (Gazebo & Unity)" --body "..."

# 4. Deploy to GitHub Pages (after merge)
cd book_frontend && npm run deploy
```
