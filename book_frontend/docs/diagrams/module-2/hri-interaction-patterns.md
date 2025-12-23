# Human-Robot Interaction Patterns Specification

**Purpose**: Illustrate common HRI scenarios that Unity's visual fidelity enables for training and testing

**Diagram Type**: conceptual

## HRI Scenario Categories

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                    HUMAN-ROBOT INTERACTION PATTERNS                          │
│                    (Unity Simulation Scenarios)                              │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────┐  ┌─────────────────────────┐  ┌─────────────────────┐
│   PHYSICAL INTERACTION  │  │    SOCIAL NAVIGATION    │  │   COMMUNICATION     │
│                         │  │                         │  │                     │
│  • Object Handoff       │  │  • Personal Space       │  │  • Gesture Recognition│
│  • Collaborative Lift   │  │  • Path Prediction      │  │  • Gaze Following    │
│  • Tool Exchange        │  │  • Group Navigation     │  │  • Speech Interaction│
│  • Contact Detection    │  │  • Door/Elevator        │  │  • Emotion Response  │
└─────────────────────────┘  └─────────────────────────┘  └─────────────────────┘
```

## Pattern 1: Object Handoff

```text
OBJECT HANDOFF INTERACTION
══════════════════════════

SCENARIO: Robot hands object to human; human receives and acknowledges

    Phase 1: APPROACH          Phase 2: PRESENT           Phase 3: RELEASE
    ═══════════════           ═══════════════            ═══════════════

         ┌───┐                     ┌───┐                      ┌───┐
         │ H │                     │ H │                      │ H │
         └─┬─┘                     └─┬─┘                      └─┬─┘
           │                         │ ← hand extends           │●─ object
           │                         │                          │
           │         ●               │    ●                     │
           │        ╱│╲              │   ╱│╲                    │
           │         │               │    │                     │
       ┌───┴───┐     │           ┌───┴───┐                 ┌───┴───┐
       │       │    ●┤           │       ├●                │       │
       │ Robot │    █│           │ Robot │█─ gripper open  │ Robot │
       │       │     │           │       │                 │       │
       └───────┘     │           └───────┘                 └───────┘

    Robot approaches     Robot extends arm,         Human grasps object,
    optimal handoff      offers object at           robot detects grip force,
    distance (0.5-0.8m)  comfortable height         releases gripper


UNITY SIMULATION REQUIREMENTS:
─────────────────────────────
• Human NPC with inverse kinematics (reach animation)
• Gripper force/contact sensing
• Handoff position calculation (human arm reach zone)
• Release trigger based on detected grasp


STATE MACHINE:
──────────────
┌──────────┐    human detected    ┌───────────┐    in range    ┌───────────┐
│  IDLE    ├─────────────────────►│ APPROACH  ├───────────────►│  PRESENT  │
└──────────┘                      └───────────┘                └─────┬─────┘
                                                                     │
┌──────────┐    timer expired     ┌───────────┐    grasp detected    │
│  IDLE    │◄────────────────────┤  RETRACT  │◄──────────────────────┘
└──────────┘                      └───────────┘


ROS 2 TOPICS FOR HANDOFF:
─────────────────────────
/human_pose         → geometry_msgs/PoseStamped (hand tracking)
/handoff_command    → std_msgs/String (initiate, abort)
/gripper_force      → std_msgs/Float64 (grasp detection)
/handoff_state      → std_msgs/String (approaching, presenting, complete)
```

## Pattern 2: Social Navigation

```text
SOCIAL NAVIGATION PATTERNS
══════════════════════════

SCENARIO: Robot navigates through space while respecting human comfort zones

PROXEMICS ZONES (Hall's Model):
───────────────────────────────

                    ┌─────────────────────────────────────────┐
                    │                PUBLIC                    │
                    │              (3.6m - 7.6m)               │
                    │    ┌─────────────────────────────┐      │
                    │    │         SOCIAL              │      │
                    │    │       (1.2m - 3.6m)         │      │
                    │    │   ┌─────────────────────┐   │      │
                    │    │   │     PERSONAL        │   │      │
                    │    │   │   (0.45m - 1.2m)    │   │      │
                    │    │   │  ┌─────────────┐    │   │      │
                    │    │   │  │  INTIMATE   │    │   │      │
                    │    │   │  │ (< 0.45m)   │    │   │      │
                    │    │   │  │    ┌───┐    │    │   │      │
                    │    │   │  │    │ H │    │    │   │      │
                    │    │   │  │    └───┘    │    │   │      │
                    │    │   │  └─────────────┘    │   │      │
                    │    │   └─────────────────────┘   │      │
                    │    └─────────────────────────────┘      │
                    └─────────────────────────────────────────┘


NAVIGATION BEHAVIORS:
─────────────────────

1. PASSING (Hallway)                    2. YIELDING (Doorway)

   H → →                                      ┌────┐
                                              │    │
   ════════════════                       H → │    │
                     ← Robot avoids           │    │
   ════════════════                       R ○ │    │ Robot waits
                                              │    │
   Robot detects human, moves right           └────┘


3. FOLLOWING (Guidance)                 4. GROUP NAVIGATION

         H                                  H   H
         │                                   \ /
         │ 1.5m                               ●  Group center
         │                                   / \
         R                                  H   R → maintains
                                                   formation
   Robot maintains following
   distance, matches pace


UNITY SIMULATION ELEMENTS:
──────────────────────────
• Multiple human NPCs with NavMesh navigation
• Randomized walking patterns and speeds
• Group formation behaviors
• Dynamic obstacle spawning
• Crowd density variation


ROS 2 TOPICS FOR SOCIAL NAV:
────────────────────────────
/people_tracker     → people_msgs/People (detected humans)
/social_costmap     → nav_msgs/OccupancyGrid (augmented with social costs)
/human_prediction   → nav_msgs/Path (predicted human trajectories)
/robot_intention    → geometry_msgs/PoseArray (robot's planned path)
```

## Pattern 3: Gesture Recognition

```text
GESTURE RECOGNITION INTERACTION
═══════════════════════════════

SCENARIO: Robot recognizes and responds to human gestures

COMMON GESTURE VOCABULARY:
──────────────────────────

┌────────────────┬────────────────┬────────────────┬────────────────┐
│    STOP        │    COME HERE   │    POINT       │    WAVE        │
│                │                │                │                │
│     ┌───┐      │     ┌───┐      │     ┌───┐      │     ┌───┐      │
│     │ H │      │     │ H │      │     │ H │      │     │ H │      │
│    ─┴─┬─┴─     │    ─┴─┬─┴─     │    ─┴─┬─┴─     │    ─┴─┬─┴─     │
│   ═══╪═══     │      ╲│        │      ╱│────►   │      ╱│~~~     │
│      │        │       │╲       │      │        │      │        │
│     ╱ ╲       │      ╱ ╲       │     ╱ ╲       │     ╱ ╲       │
│                │                │                │                │
│   Palm out     │   Beckoning    │   Index finger │   Side-to-side │
│   Robot stops  │   Robot approach│  Robot looks  │   Greeting     │
└────────────────┴────────────────┴────────────────┴────────────────┘

┌────────────────┬────────────────┬────────────────┬────────────────┐
│    THUMBS UP   │    THUMBS DOWN │    GRAB/TAKE   │    PUSH AWAY   │
│                │                │                │                │
│     ┌───┐      │     ┌───┐      │     ┌───┐      │     ┌───┐      │
│     │ H │      │     │ H │      │     │ H │      │     │ H │      │
│    ─┴─┬─┴─     │    ─┴─┬─┴─     │    ─┴─┬─┴─     │    ─┴─┬─┴─     │
│      │👍       │      │👎       │     ✊│        │   ═══╪═══►    │
│      │        │      │        │      │        │      │        │
│     ╱ ╲       │     ╱ ╲       │     ╱ ╲       │     ╱ ╲       │
│                │                │                │                │
│   Confirmation │   Rejection    │   Request obj  │   Back away    │
│   Continue task│   Stop/redo    │   Robot offers │   Robot retreats│
└────────────────┴────────────────┴────────────────┴────────────────┘


GESTURE RECOGNITION PIPELINE:
─────────────────────────────

┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│ Unity Camera│───►│ Image to    │───►│ Pose        │───►│ Gesture     │
│ (RGB stream)│    │ ROS 2       │    │ Estimation  │    │ Classifier  │
└─────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘
                                                                │
┌─────────────┐    ┌─────────────┐    ┌─────────────┐           │
│ Robot       │◄───│ Behavior    │◄───│ Gesture     │◄──────────┘
│ Action      │    │ Selector    │    │ Message     │
└─────────────┘    └─────────────┘    └─────────────┘


UNITY SIMULATION FEATURES:
──────────────────────────
• Human avatars with gesture animations (Mixamo/custom)
• Random gesture triggering for training data
• Varied lighting conditions
• Multiple camera viewpoints
• Occlusion scenarios (partial visibility)


ROS 2 TOPICS FOR GESTURES:
──────────────────────────
/camera/image_raw   → sensor_msgs/Image (RGB from Unity)
/body_pose          → geometry_msgs/PoseArray (skeleton keypoints)
/gesture_detected   → std_msgs/String (recognized gesture label)
/gesture_confidence → std_msgs/Float64 (recognition confidence)
```

## Pattern 4: Collaborative Task

```text
COLLABORATIVE MANIPULATION
══════════════════════════

SCENARIO: Human and robot work together to move a large object

COOPERATIVE LIFTING:
───────────────────

    Initial                Lifting                 Moving
    ───────               ───────                 ──────

    ┌───┐                 ┌───┐                   ┌───┐
    │ H │                 │ H │                   │ H │───►
    └─┬─┘                 └─┬─┘                   └─┬─┘
      │                     │                       │
    ┌─┴─┐                 ┌─┴─┐                   ┌─┴─┐
    │███│                 │███│                   │███│───►
    └─┬─┘                 └─┬─┘                   └─┬─┘
      │                     │                       │
   ┌──┴──┐              ┌──┴──┐                 ┌──┴──┐
   │Robot│              │Robot│                 │Robot│───►
   └─────┘              └─────┘                 └─────┘

  Both grasp      Synchronized lift      Coordinated movement
  opposite ends   based on force feedback  human leads, robot follows


FORCE COORDINATION:
───────────────────

    Human applies force →      Robot senses and responds

         Fh = 10N →            ┌───────────────────┐
    ┌───┐                      │ Impedance Control │
    │ H ├──────┐               │                   │
    └───┘      │               │ Fr = -Kp(x-xd)    │
               │               │    - Kd(v-vd)     │
         ┌─────┴─────┐         │                   │
         │   OBJECT  │         │ Allows compliant  │
         └─────┬─────┘         │ motion following  │
               │               │ human intention   │
    ┌──────────┴──────────┐    └───────────────────┘
    │        Robot        │
    │    (force sensing)  │    Fr = 8N (compliant follow)
    └─────────────────────┘


ROS 2 TOPICS FOR COLLABORATION:
───────────────────────────────
/wrench_human_estimate  → geometry_msgs/Wrench (estimated human force)
/robot_wrench           → geometry_msgs/Wrench (robot applied force)
/object_pose            → geometry_msgs/Pose (shared object pose)
/collaboration_state    → std_msgs/String (waiting, grasped, lifting, moving)
```

## Unity Implementation Checklist

```text
UNITY HRI SCENE REQUIREMENTS
════════════════════════════

□ HUMAN AVATARS
  ├── Rigged humanoid mesh (Mixamo or custom)
  ├── Animator controller with HRI states
  ├── IK setup for hand reaching
  └── NavMeshAgent for navigation

□ ROBOT MODEL
  ├── URDF imported via Unity Robotics Hub
  ├── ArticulationBody components for physics
  ├── Gripper with contact sensing
  └── Sensor simulations (cameras, force)

□ ENVIRONMENT
  ├── Indoor scene (home, office, hospital)
  ├── Obstacles and furniture
  ├── Lighting variations (time of day)
  └── NavMesh baked for navigation

□ INTERACTION TRIGGERS
  ├── Proximity detection (OnTriggerEnter)
  ├── Gesture animation triggers
  ├── Object spawn points
  └── Task initiation zones

□ ROS 2 INTEGRATION
  ├── ROSConnection singleton configured
  ├── Publishers for sensor data
  ├── Subscribers for commands
  └── Service for reset/spawn
```

## Usage in Book

- **Referenced in**: Chapter 5 (Core Concept 4: Human-Robot Interaction Scenarios)
- **Purpose**: Illustrate HRI scenarios that benefit from Unity's visual and physics capabilities
- **Learning Goal**: Design HRI simulations for training socially-aware robot behaviors

## Key Takeaways

1. **Object handoff**: Requires force sensing, human hand tracking, and coordinated timing
2. **Social navigation**: Must respect proxemics zones; Unity enables realistic crowd simulation
3. **Gesture recognition**: Unity provides photorealistic training data with diverse humans/conditions
4. **Collaborative tasks**: Force/impedance control essential; Unity simulates contact physics
5. **Human NPCs**: Use Mixamo animations + NavMesh for realistic movement patterns
6. **Training diversity**: Unity enables randomization of humans, lighting, scenarios for robust ML
