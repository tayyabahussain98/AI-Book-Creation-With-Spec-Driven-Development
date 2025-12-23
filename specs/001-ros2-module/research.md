# Research: Module 1 - ROS 2 Educational Content & Docusaurus Setup

**Date**: 2025-12-21
**Feature**: 001-ros2-module
**Phase**: 0 (Research)

## Research Questions

1. **Docusaurus Setup**: How to initialize, configure, and deploy Docusaurus site to GitHub Pages?
2. **ROS 2 Code Examples**: What patterns ensure runnable, copy-pasteable rclpy examples for learners?
3. **Chapter Structure**: How to organize 5-week educational content with progressive complexity?
4. **Diagram Specifications**: How to describe ROS 2 architecture diagrams textually for later rendering?

---

## 1. Docusaurus Setup & GitHub Pages Deployment

### Decision
Use Docusaurus 3.x with standard create-docusaurus initialization, configure for GitHub Pages deployment via gh-pages branch.

### Rationale
- **Docusaurus 3.x**: Latest stable, excellent markdown support, built-in search, versioning
- **GitHub Pages**: Free hosting (constitution Principle VI), simple deployment workflow
- **Standard initialization**: Minimal custom configuration, proven patterns

### Implementation Steps

**A. Initialize Docusaurus**:
```bash
# In book_frontend/ directory
npx create-docusaurus@latest . classic --typescript=false
```

**B. Configure docusaurus.config.js**:
```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Building Intelligent Embodied Systems',
  url: 'https://tayyabahussain98.github.io',
  baseUrl: '/AI-book-creation-with-spec-driven-development/',
  organizationName: 'tayyabahussain98',
  projectName: 'AI-book-creation-with-spec-driven-development',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,
  // ... standard presets
};
```

**C. Configure sidebars.js** for Module 1:
```javascript
module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-1-ros2/chapter-1-intro-physical-ai',
        'module-1-ros2/chapter-2-ros2-architecture',
        'module-1-ros2/chapter-3-python-nodes',
        'module-1-ros2/chapter-4-packages-launch',
        'module-1-ros2/chapter-5-urdf',
      ],
    },
  ],
};
```

**D. GitHub Pages Deployment**:
```bash
# Add to package.json scripts
"deploy": "docusaurus deploy"

# Set Git config (already done in repo)
# Deploy command:
npm run deploy
```

### Alternatives Considered
- **MkDocs**: Python-based, good for API docs, but less interactive than Docusaurus
- **GitBook**: Nice UI but paid tiers for advanced features
- **Gatsby**: More complex setup, overkill for documentation

### References
- Docusaurus Official Docs: https://docusaurus.io/docs
- GitHub Pages Guide: https://docusaurus.io/docs/deployment#deploying-to-github-pages

---

## 2. ROS 2 Code Example Patterns

### Decision
Use minimal, self-contained Python rclpy examples with explicit imports, parameter defaults, and inline comments. Each example focuses on ONE concept.

### Rationale
- **Self-contained**: Learners can copy-paste and run immediately (constitution Principle III)
- **Explicit**: No hidden dependencies, all imports visible
- **Focused**: One concept per example reduces cognitive load
- **Commented**: Inline explanations aid learning without external docs

### Example Pattern: Publisher Node

```python
#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example
Publishes mock joint angles at 10Hz
Target: ROS 2 Humble/Iron
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')

        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(Float32, 'joint_angle', 10)

        # Create timer: period (0.1s = 10Hz), callback function
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.angle = 0.0
        self.get_logger().info('Joint angle publisher started')

    def timer_callback(self):
        """Called every 0.1 seconds by timer"""
        msg = Float32()
        msg.data = self.angle

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing angle: {self.angle:.2f}')

        # Increment angle (simulate joint movement)
        self.angle += 0.1
        if self.angle > 6.28:  # 2*pi radians
            self.angle = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = JointAnglePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run command** (included in chapter):
```bash
python3 publisher_example.py
```

### Key Patterns
1. **Shebang**: `#!/usr/bin/env python3` for executable scripts
2. **Docstring**: Purpose, target ROS 2 version
3. **Explicit imports**: Every dependency visible
4. **Class-based nodes**: Standard ROS 2 pattern
5. **Logging**: `get_logger().info()` for visibility
6. **Cleanup**: try/except/finally for graceful shutdown
7. **Main guard**: `if __name__ == '__main__'` for module reuse

### Alternatives Considered
- **Function-based nodes**: Simpler but less reusable, not idiomatic ROS 2
- **External config files**: Adds complexity, violates self-contained principle
- **C++ examples**: Out of scope (FR-009: Python focus)

### References
- ROS 2 Python Examples: https://github.com/ros2/examples/tree/humble/rclpy
- rclpy API Docs: https://docs.ros2.org/humble/api/rclpy/

---

## 3. Chapter Structure & Progressive Complexity

### Decision
5-chapter structure with consistent template: Introduction → Concepts → Examples → Diagram → Summary → Checklist. Complexity increases chapter-to-chapter (abstract → concrete).

### Chapter Template Structure

**Each chapter contains**:
1. **Introduction** (1-2 paragraphs): Learning goals, why this matters
2. **Core Concepts** (3-5 sections): Definitions, explanations, comparisons
3. **Architecture/Flow Diagrams** (1-2 diagrams): Textual descriptions
4. **Code Examples** (2-3 examples): Minimal, runnable, commented
5. **Summary** (bullet list): Key takeaways
6. **Self-Assessment Checklist** (5-7 items): Can-do statements

### Progressive Complexity Mapping

| Chapter | Complexity Level | Focus | Code Complexity |
|---------|------------------|-------|-----------------|
| 1 | Conceptual | Why ROS 2 exists | No code |
| 2 | Architectural | How ROS 2 works | Pseudo-code only |
| 3 | Introductory Code | Basic nodes | Simple pub/sub |
| 4 | Intermediate Code | Multi-node systems | Launch files, params |
| 5 | Data Modeling | Robot description | URDF parsing |

### Rationale
- **Consistent template**: Predictable structure aids learning retention
- **Progressive complexity**: Scaffolding approach (Vygotsky's ZPD)
- **No code → code transition**: Chapter 1-2 build mental models before Chapter 3 coding
- **Self-assessment**: Metacognition improves learning outcomes (SC-007)

### Alternatives Considered
- **Code-first approach**: Risks learners copying without understanding
- **Mixed theory/practice per chapter**: Context-switching reduces comprehension
- **Longer chapters**: Exceeds attention span for self-paced learning

### References
- Educational Design Patterns: Cognitive Load Theory (Sweller)
- Docusaurus Docs Structure: https://docusaurus.io/docs/docs-introduction

---

## 4. Diagram Specifications (Textual Descriptions)

### Decision
Describe diagrams using structured markdown with component lists, connections, and flow annotations. Later rendered as Mermaid diagrams or illustrations.

### Example: ROS 2 Signal Flow Diagram

**File**: `docs/diagrams/module-1/ros2-signal-flow.md`

```markdown
# Diagram: AI Decision to Robot Actuator Signal Flow

**Purpose**: Show how an AI agent's decision travels through ROS 2 to control a robot joint.

**Components**:
1. AI Agent Node (Python)
   - Decides "move arm to position X"
   - Publishes command to `/arm_command` topic

2. ROS 2 Middleware (DDS)
   - Routes message from publisher to subscribers
   - Handles discovery, serialization

3. Motor Controller Node (Python)
   - Subscribes to `/arm_command` topic
   - Translates command to motor signals

4. Robot Actuator (Hardware)
   - Receives motor signals
   - Moves physical joint

**Flow**:
```
AI Agent → [Decision Logic] → Publish(topic=/arm_command, msg=MoveCommand)
                                      ↓
                              ROS 2 DDS Middleware
                                      ↓
Motor Controller ← Subscribe(topic=/arm_command) ← [Command Handler]
         ↓
    [Signal Translation]
         ↓
    Robot Actuator → Physical Movement
```

**Labels**:
- Dashed lines: Data flow
- Solid boxes: Software nodes
- Rounded box: Hardware component
- Arrow direction: Information flow

**Notes**:
- AI Agent and Motor Controller run as separate processes
- DDS handles network communication (can be same machine or distributed)
- Motor Controller validates commands before sending to hardware
```

### Rationale
- **Textual format**: Version-controllable, searchable, AI-parseable
- **Structured markdown**: Easy to convert to Mermaid, draw.io, or hand-drawn illustrations
- **Explicit labels**: No ambiguity about diagram elements
- **Deferred rendering**: Allows flexibility in visualization tools later

### Alternatives Considered
- **Mermaid inline**: Requires learners to understand Mermaid syntax
- **Hand-drawn + embedded images**: Not version-controllable, hard to update
- **PlantUML**: Another DSL to learn, less portable

### References
- Mermaid Diagram Syntax: https://mermaid.js.org/
- Docusaurus Diagram Support: https://docusaurus.io/docs/markdown-features/diagrams

---

## Technology Stack Summary

| Component | Choice | Version | Rationale |
|-----------|--------|---------|-----------|
| Site Generator | Docusaurus | 3.x | Best-in-class for tech docs, free, easy deployment |
| Hosting | GitHub Pages | N/A | Free tier (constitution), integrated with repo |
| Content Format | Markdown | CommonMark | Universal, version-controllable, Docusaurus native |
| Code Examples | Python rclpy | 3.10+ | Spec requirement (FR-004, FR-008) |
| ROS 2 Target | Humble/Iron | LTS/Stable | Long-term support, learner-accessible |
| Diagram Format | Markdown descriptions | N/A | Textual, flexible rendering later |
| Build Tool | npm/Node.js | 18+ | Docusaurus requirement |

---

## Next Steps (Phase 1)

1. **Create data-model.md**: Define chapter content structure, code example schema
2. **Generate contracts/**: Chapter templates with placeholders for content
3. **Write quickstart.md**: Developer workflow for editing/building/deploying docs
4. **Update agent context**: Add Docusaurus, ROS 2 tooling info

**Unresolved Questions**: None - all research complete.
