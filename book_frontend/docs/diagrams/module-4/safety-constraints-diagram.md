---
title: Safety Constraints Diagram
module: 4
chapter: 5
type: architecture
---

# Safety Constraints Diagram

## Overview

This diagram illustrates the multi-layered safety architecture for VLA systems. Safety constraints operate at every level from high-level intent validation through real-time motor control, ensuring the robot operates safely even when autonomous language-driven commands are executed.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                     MULTI-LAYER SAFETY ARCHITECTURE                         │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 5: INTENT SAFETY (Semantic Level)                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  User Command: "Throw the cup at John"                                     │
│                       │                                                     │
│                       ▼                                                     │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                  INTENT VALIDATOR                                    │   │
│  │                                                                      │   │
│  │  Blacklist Check:                                                   │   │
│  │    ✗ "throw at person" → BLOCKED (harm potential)                   │   │
│  │    ✗ "hit", "attack", "hurt" → BLOCKED                              │   │
│  │    ✗ "destroy", "break" → REQUIRES CONFIRMATION                     │   │
│  │                                                                      │   │
│  │  Action: REJECT with explanation                                    │   │
│  │  Response: "I can't throw objects at people. Would you like me to   │   │
│  │            hand the cup to John instead?"                           │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  Safe Alternatives: Pick, Place, Give, Find, Move                          │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 4: PLAN SAFETY (Task Level)                                           │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Generated Plan: PICK(knife) → MOVE_TO(high_speed) → GIVE(person)         │
│                       │                                                     │
│                       ▼                                                     │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                  PLAN VALIDATOR                                      │   │
│  │                                                                      │   │
│  │  Object Risk Assessment:                                            │   │
│  │    knife: SHARP, DANGEROUS                                          │   │
│  │    → Require: slow_approach, careful_grasp, safe_orientation       │   │
│  │                                                                      │   │
│  │  Speed Limits Near Humans:                                          │   │
│  │    high_speed within 1.5m of person → MODIFY                        │   │
│  │    → Replace: MOVE_TO(reduced_speed, trajectory_smoothing)          │   │
│  │                                                                      │   │
│  │  Handover Safety:                                                   │   │
│  │    sharp_object → handle_first orientation required                 │   │
│  │    → Add: ORIENT(handle_toward_person) before GIVE                  │   │
│  │                                                                      │   │
│  │  Modified Plan: PICK(knife, careful) → ORIENT(handle_first) →       │   │
│  │                 MOVE_TO(slow) → GIVE(person, wait_for_grasp)        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 3: MOTION SAFETY (Trajectory Level)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Planned Trajectory: q(t) over t ∈ [0, T]                                  │
│                       │                                                     │
│                       ▼                                                     │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                TRAJECTORY VALIDATOR                                  │   │
│  │                                                                      │   │
│  │  ┌─────────────────────────────────────────────────────────────┐    │   │
│  │  │  Collision Check                                            │    │   │
│  │  │                                                             │    │   │
│  │  │  ┌──────────┐    ┌──────────┐    ┌──────────┐              │    │   │
│  │  │  │ Self     │    │ Environ- │    │ Human    │              │    │   │
│  │  │  │Collision │    │  ment    │    │Proximity │              │    │   │
│  │  │  │          │    │Collision │    │          │              │    │   │
│  │  │  │ Links vs │    │ Arm vs   │    │ Arm vs   │              │    │   │
│  │  │  │ links    │    │ objects  │    │ person   │              │    │   │
│  │  │  │          │    │ tables   │    │ > 0.3m   │              │    │   │
│  │  │  └──────────┘    └──────────┘    └──────────┘              │    │   │
│  │  │       ✓               ✓              ⚠                     │    │   │
│  │  │                                  WARNING:                   │    │   │
│  │  │                              min_distance = 0.25m           │    │   │
│  │  │                              → Slow to 50% speed           │    │   │
│  │  └─────────────────────────────────────────────────────────────┘    │   │
│  │                                                                      │   │
│  │  ┌─────────────────────────────────────────────────────────────┐    │   │
│  │  │  Kinematic/Dynamic Limits                                   │    │   │
│  │  │                                                             │    │   │
│  │  │  Joint Position:  q ∈ [q_min, q_max]           ✓ PASS      │    │   │
│  │  │  Joint Velocity:  dq ∈ [-dq_max, dq_max]       ✓ PASS      │    │   │
│  │  │  Joint Accel:     ddq ∈ [-ddq_max, ddq_max]    ✓ PASS      │    │   │
│  │  │  Cartesian Vel:   v_ee < 1.0 m/s               ✓ PASS      │    │   │
│  │  └─────────────────────────────────────────────────────────────┘    │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 2: CONTROL SAFETY (Real-Time Level)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Control Loop: 1kHz                                                        │
│                       │                                                     │
│                       ▼                                                     │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                REAL-TIME SAFETY MONITOR                              │   │
│  │                                                                      │   │
│  │  ┌─────────────┬─────────────┬─────────────┬─────────────┐          │   │
│  │  │   Force     │  Velocity   │   Torque    │  Position   │          │   │
│  │  │   Limit     │   Limit     │   Limit     │   Limit     │          │   │
│  │  │             │             │             │             │          │   │
│  │  │ F < 50N     │ v < 1.0m/s  │ τ < τ_max   │ q ∈ bounds  │          │   │
│  │  │             │             │             │             │          │   │
│  │  │ Current:    │ Current:    │ Current:    │ Current:    │          │   │
│  │  │   32N ✓     │   0.4m/s ✓  │   45% ✓     │   OK ✓      │          │   │
│  │  └─────────────┴─────────────┴─────────────┴─────────────┘          │   │
│  │                                                                      │   │
│  │  ┌───────────────────────────────────────────────────────────────┐  │   │
│  │  │  Violation Response                                           │  │   │
│  │  │                                                               │  │   │
│  │  │  Warning (80% limit): Reduce velocity, log event              │  │   │
│  │  │  Soft Stop (100%):    Controlled deceleration to stop        │  │   │
│  │  │  Hard Stop (120%):    Immediate brake engagement              │  │   │
│  │  │  E-Stop (critical):   Power cut to actuators                  │  │   │
│  │  └───────────────────────────────────────────────────────────────┘  │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ LAYER 1: HARDWARE SAFETY (Physical Level)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                  PHYSICAL SAFETY SYSTEMS                             │   │
│  │                                                                      │   │
│  │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐           │   │
│  │  │  Mechanical   │  │  Electrical   │  │  Emergency    │           │   │
│  │  │  Limits       │  │  Protection   │  │  Stop         │           │   │
│  │  │               │  │               │  │               │           │   │
│  │  │ • Hard stops  │  │ • Current     │  │ • Big red     │           │   │
│  │  │   at joint    │  │   limiters    │  │   button      │           │   │
│  │  │   extremes    │  │ • Fuses       │  │ • Wireless    │           │   │
│  │  │ • Compliant   │  │ • Thermal     │  │   pendant     │           │   │
│  │  │   joints      │  │   cutoffs     │  │ • Light       │           │   │
│  │  │ • Breakaway   │  │               │  │   curtain     │           │   │
│  │  │   mechanisms  │  │               │  │               │           │   │
│  │  └───────────────┘  └───────────────┘  └───────────────┘           │   │
│  │                                                                      │   │
│  │  ┌───────────────────────────────────────────────────────────────┐  │   │
│  │  │  Collaborative Robot Features                                 │  │   │
│  │  │                                                               │  │   │
│  │  │  • Rounded edges (no pinch points)                           │  │   │
│  │  │  • Low-inertia design                                        │  │   │
│  │  │  • Backdrivable joints (can push arm away)                   │  │   │
│  │  │  • Soft covers on contact surfaces                           │  │   │
│  │  │  • ISO 10218 / ISO 15066 compliant                           │  │   │
│  │  └───────────────────────────────────────────────────────────────┘  │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────┐
│                     SAFETY DECISION MATRIX                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌───────────────────────────────────────────────────────────────────────┐ │
│  │                                                                       │ │
│  │  Severity    │ Detection    │ Response     │ Recovery               │ │
│  │  ────────────┼──────────────┼──────────────┼─────────────────────── │ │
│  │  Low         │ Warning      │ Log + Alert  │ Continue               │ │
│  │  (80% limit) │ threshold    │              │                        │ │
│  │              │              │              │                        │ │
│  │  Medium      │ Soft limit   │ Slow down    │ Replan if needed       │ │
│  │  (100%)      │ exceeded     │ gracefully   │                        │ │
│  │              │              │              │                        │ │
│  │  High        │ Hard limit   │ Controlled   │ Human intervention     │ │
│  │  (120%)      │ exceeded     │ stop         │ may be required        │ │
│  │              │              │              │                        │ │
│  │  Critical    │ Danger       │ Emergency    │ Full system check      │ │
│  │  (imminent)  │ imminent     │ power off    │ before restart         │ │
│  │              │              │              │                        │ │
│  └───────────────────────────────────────────────────────────────────────┘ │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Descriptions

### Layer 5: Intent Safety

**Purpose**: Prevent dangerous commands at the semantic level

**Mechanisms**:
- Blacklist of harmful action types (throw_at_person, attack)
- Object risk classification (sharp, heavy, breakable)
- Target validation (is target appropriate for action?)

**Response**: Reject with explanation and safe alternative suggestion

### Layer 4: Plan Safety

**Purpose**: Ensure action sequences are safe as a whole

**Mechanisms**:
- Object-aware parameter selection (careful grasp for knives)
- Speed limits based on human proximity
- Required preconditions for dangerous actions

**Response**: Modify plan parameters or insert safety actions

### Layer 3: Motion Safety

**Purpose**: Validate trajectories before execution

**Mechanisms**:
- Self-collision checking
- Environment collision avoidance
- Human proximity monitoring
- Kinematic/dynamic limit verification

**Response**: Replan trajectory or reduce speed

### Layer 2: Control Safety

**Purpose**: Real-time monitoring during execution

**Mechanisms**:
- Force/torque limits (50N typical)
- Velocity limits (1m/s end-effector)
- Position bounds checking
- Watchdog timers

**Response**: Warning, soft stop, hard stop, or emergency stop

### Layer 1: Hardware Safety

**Purpose**: Physical last-resort protection

**Mechanisms**:
- Mechanical hard stops
- Electrical current limits
- Emergency stop buttons
- Compliant/backdrivable joints

**Response**: Physical prevention of damage

---

## Key Insights

### Defense in Depth

No single layer is sufficient:
- Intent safety can be fooled by clever phrasing
- Plan safety doesn't see real-time perturbations
- Control safety can't prevent all collisions
- Hardware safety is last resort only

All layers work together for robust protection.

### Graceful Degradation

Safety responses scale with severity:
1. Warning: Log and continue
2. Slow down: Reduce risk while continuing
3. Stop: Controlled halt, preserving state
4. Emergency: Immediate power cut

### Transparency

Safety rejections should be explained:
- Don't just say "no"
- Explain why the action is unsafe
- Suggest safe alternatives when possible
- Build user trust in the system

---

## Related Concepts

- **From Module 1**: ROS 2 safety patterns and lifecycle nodes
- **From Module 2**: Safety testing in simulation
- **From Module 3**: Isaac safety monitoring
- **In This Module**: Capstone Architecture, Execution Monitoring

---

## Real-World Application

Safety architectures are mandated by standards:

**ISO 10218** specifies safety requirements for industrial robots, including force limits, speed limits, and emergency stop functionality.

**ISO 15066** extends safety requirements to collaborative robots operating near humans, with specific power and force thresholds for different body parts.

**ISO/TS 15066 Power and Force Limits**:
- Head: 75N transient, 110N quasi-static
- Chest: 140N / 280N
- Hand: 140N / 280N

These standards inform the specific thresholds used in each safety layer.
