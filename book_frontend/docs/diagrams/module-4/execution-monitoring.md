---
title: Execution Monitoring
module: 4
chapter: 5
type: flow
---

# Execution Monitoring

## Overview

This diagram illustrates the execution monitoring system that observes robot actions, detects failures, and triggers appropriate recovery responses. Robust execution monitoring is essential for autonomous operation in unstructured environments.

---

## Diagram

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                      EXECUTION MONITORING SYSTEM                            │
└─────────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────────────┐
│                         SENSOR INPUTS                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
│  │   Vision    │  │Force/Torque │  │   Joint     │  │  External   │       │
│  │   (30 Hz)   │  │  (1000 Hz)  │  │   State     │  │  Sensors    │       │
│  │             │  │             │  │  (1000 Hz)  │  │             │       │
│  │ • RGB image │  │ • Wrist F/T │  │ • Position  │  │ • Proximity │       │
│  │ • Depth     │  │ • Gripper   │  │ • Velocity  │  │ • Contact   │       │
│  │ • Detections│  │   force     │  │ • Torque    │  │ • IMU       │       │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘       │
│         │                │                │                │               │
│         └────────────────┼────────────────┼────────────────┘               │
│                          │                │                                 │
└──────────────────────────┼────────────────┼─────────────────────────────────┘
                           │                │
                           ▼                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      STATE ESTIMATION                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    MULTI-MODAL STATE FUSION                          │   │
│  │                                                                      │   │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐      │   │
│  │  │  Robot State    │  │  Object State   │  │  Environment    │      │   │
│  │  │                 │  │                 │  │    State        │      │   │
│  │  │ • Arm pose      │  │ • Target pose   │  │ • Obstacles     │      │   │
│  │  │ • Gripper state │  │ • Held object   │  │ • Human pos     │      │   │
│  │  │ • Contact state │  │ • Object state  │  │ • Free space    │      │   │
│  │  │   (touching?)   │  │   (grasped?)    │  │                 │      │   │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘      │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                    │                                       │
│                                    ▼                                       │
│                      ┌───────────────────────────┐                         │
│                      │    UNIFIED WORLD STATE    │                         │
│                      │    Updated at 30 Hz       │                         │
│                      └───────────────────────────┘                         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      EXPECTATION MONITORING                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Current Action: PICK(red_cup)                                             │
│  Expected Postcondition: gripper.holding = red_cup                         │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    EXPECTATION CHECKERS                              │   │
│  │                                                                      │   │
│  │  ┌───────────────────┬───────────────────┬───────────────────┐      │   │
│  │  │   Pose Match      │   Force Match     │  Visual Match     │      │   │
│  │  │                   │                   │                   │      │   │
│  │  │   Expected:       │   Expected:       │   Expected:       │      │   │
│  │  │   arm @ grasp_pos │   F > 5N (grasp)  │   cup in gripper  │      │   │
│  │  │                   │                   │                   │      │   │
│  │  │   Actual:         │   Actual:         │   Actual:         │      │   │
│  │  │   arm @ grasp_pos │   F = 8N          │   cup detected    │      │   │
│  │  │                   │                   │   in gripper      │      │   │
│  │  │   ✓ MATCH         │   ✓ MATCH         │   ✓ MATCH         │      │   │
│  │  └───────────────────┴───────────────────┴───────────────────┘      │   │
│  │                                                                      │   │
│  │  Overall: ALL EXPECTATIONS MET → Action SUCCEEDED                   │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      ANOMALY DETECTION                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    FAILURE MODE DETECTION                            │   │
│  │                                                                      │   │
│  │  ┌────────────────────────────────────────────────────────────────┐ │   │
│  │  │ GRASP FAILURE                                                  │ │   │
│  │  │                                                                │ │   │
│  │  │ Symptoms:                                                      │ │   │
│  │  │   • Gripper closed but F < expected (object not grasped)      │ │   │
│  │  │   • Object still at original position (visual)                │ │   │
│  │  │   • No weight change in F/T sensor                            │ │   │
│  │  │                                                                │ │   │
│  │  │ Diagnosis: GRASP_SLIP or GRASP_MISS                           │ │   │
│  │  └────────────────────────────────────────────────────────────────┘ │   │
│  │                                                                      │   │
│  │  ┌────────────────────────────────────────────────────────────────┐ │   │
│  │  │ COLLISION DETECTED                                             │ │   │
│  │  │                                                                │ │   │
│  │  │ Symptoms:                                                      │ │   │
│  │  │   • Unexpected force spike (F > threshold)                    │ │   │
│  │  │   • Position error increasing (can't reach target)            │ │   │
│  │  │   • Torque at limits                                          │ │   │
│  │  │                                                                │ │   │
│  │  │ Diagnosis: COLLISION_STATIC or COLLISION_DYNAMIC              │ │   │
│  │  └────────────────────────────────────────────────────────────────┘ │   │
│  │                                                                      │   │
│  │  ┌────────────────────────────────────────────────────────────────┐ │   │
│  │  │ OBJECT MOVED                                                   │ │   │
│  │  │                                                                │ │   │
│  │  │ Symptoms:                                                      │ │   │
│  │  │   • Target object not at expected position (visual)           │ │   │
│  │  │   • Object detected at different location                     │ │   │
│  │  │                                                                │ │   │
│  │  │ Diagnosis: TARGET_MOVED or TARGET_LOST                        │ │   │
│  │  └────────────────────────────────────────────────────────────────┘ │   │
│  │                                                                      │   │
│  │  ┌────────────────────────────────────────────────────────────────┐ │   │
│  │  │ TIMEOUT                                                        │ │   │
│  │  │                                                                │ │   │
│  │  │ Symptoms:                                                      │ │   │
│  │  │   • Action duration > max_duration                            │ │   │
│  │  │   • Progress stalled (no state change)                        │ │   │
│  │  │                                                                │ │   │
│  │  │ Diagnosis: ACTION_TIMEOUT                                      │ │   │
│  │  └────────────────────────────────────────────────────────────────┘ │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      RECOVERY DECISION                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    RECOVERY STRATEGY SELECTOR                        │   │
│  │                                                                      │   │
│  │  Failure: GRASP_SLIP (object slipped during grasp)                  │   │
│  │                                                                      │   │
│  │  ┌─────────────────────────────────────────────────────────────┐    │   │
│  │  │  Strategy Options:                                          │    │   │
│  │  │                                                             │    │   │
│  │  │  1. RETRY (same approach)                                   │    │   │
│  │  │     Conditions: attempts < 2, object still reachable        │    │   │
│  │  │     Action: Retry grasp with same parameters                │    │   │
│  │  │                                                             │    │   │
│  │  │  2. RETRY_MODIFIED                                          │    │   │
│  │  │     Conditions: attempts >= 2, object still visible         │    │   │
│  │  │     Action: Try different grasp pose or force               │    │   │
│  │  │                                                             │    │   │
│  │  │  3. REPLAN                                                  │    │   │
│  │  │     Conditions: object moved significantly                  │    │   │
│  │  │     Action: Generate new plan from current state            │    │   │
│  │  │                                                             │    │   │
│  │  │  4. ABORT                                                   │    │   │
│  │  │     Conditions: safety risk or max attempts reached         │    │   │
│  │  │     Action: Stop, report failure, request human help        │    │   │
│  │  │                                                             │    │   │
│  │  │  Selected: RETRY_MODIFIED (attempt 3 with adjusted force)   │    │   │
│  │  └─────────────────────────────────────────────────────────────┘    │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      RECOVERY EXECUTION                                     │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    RECOVERY ACTIONS                                  │   │
│  │                                                                      │   │
│  │  Before Recovery:                                                   │   │
│  │    1. Ensure safe state (stop motion, open gripper if stuck)        │   │
│  │    2. Update world model (re-detect objects)                        │   │
│  │    3. Log failure details for learning                              │   │
│  │                                                                      │   │
│  │  Execute Recovery:                                                  │   │
│  │    1. Modified PICK with:                                           │   │
│  │       - grasp_force: 10N → 15N (increased)                         │   │
│  │       - approach_speed: 0.1m/s → 0.05m/s (slower)                  │   │
│  │       - grasp_offset: adjusted based on slip direction             │   │
│  │                                                                      │   │
│  │  After Recovery Attempt:                                            │   │
│  │    1. Verify success via expectation monitoring                     │   │
│  │    2. If success: continue with next action                        │   │
│  │    3. If fail: escalate to next recovery strategy                  │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                      EXECUTION STATUS                                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  Current Status:                                                     │   │
│  │                                                                      │   │
│  │  Task: "Pick up the red cup"                                        │   │
│  │  Action: PICK (attempt 3/5)                                         │   │
│  │  Progress: ████████░░ 80%                                           │   │
│  │                                                                      │   │
│  │  Recovery History:                                                  │   │
│  │    Attempt 1: GRASP_SLIP → RETRY                                    │   │
│  │    Attempt 2: GRASP_SLIP → RETRY_MODIFIED (force +25%)              │   │
│  │    Attempt 3: IN_PROGRESS                                           │   │
│  │                                                                      │   │
│  │  Outputs:                                                           │   │
│  │    → /execution/status: RECOVERING                                  │   │
│  │    → /execution/feedback: "Adjusting grasp force..."               │   │
│  │    → /execution/progress: 0.80                                      │   │
│  │                                                                      │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Component Descriptions

### Sensor Inputs

**Vision (30 Hz)**: RGB images, depth maps, object detections
**Force/Torque (1000 Hz)**: Wrist forces, gripper contact forces
**Joint State (1000 Hz)**: Position, velocity, torque for all joints
**External Sensors**: Proximity sensors, contact switches, IMU

### State Estimation

**Robot State**: Arm pose, gripper configuration, contact detection
**Object State**: Target pose, whether object is grasped
**Environment State**: Obstacles, human positions, free space

### Expectation Monitoring

**Pose Match**: Is the robot where it should be?
**Force Match**: Are contact forces as expected?
**Visual Match**: Does the scene look correct?

### Anomaly Detection

**Grasp Failure**: Object not grasped despite gripper closing
**Collision**: Unexpected force, position error
**Object Moved**: Target not at expected location
**Timeout**: Action taking too long

### Recovery Decision

**Retry**: Same approach, low-cost recovery
**Retry Modified**: Adjust parameters based on failure
**Replan**: Generate new plan from current state
**Abort**: Stop and request human intervention

---

## Data Flow

1. **Sensors**: Continuous data from all sensor modalities
2. **Fusion**: Combine into unified world state at 30Hz
3. **Expectations**: Compare state to action postconditions
4. **Detection**: Identify specific failure modes
5. **Decision**: Select appropriate recovery strategy
6. **Recovery**: Execute recovery actions
7. **Status**: Report to user interface and logs

---

## Key Insights

### Multi-Modal Verification

Single sensors can be fooled:
- Vision: Object occluded during grasp
- Force: Gripper touching table, not object

Combining modalities increases reliability.

### Graceful Recovery Hierarchy

Recovery strategies escalate:
1. Simple retry (cheap, often works)
2. Parameter adjustment (adapt to conditions)
3. Full replan (major state change)
4. Human help (safety or repeated failure)

### Failure Logging for Learning

Every failure is a learning opportunity:
- Record failure type, conditions, recovery outcome
- Analyze patterns to improve future behavior
- Update grasp models, force thresholds

---

## Related Concepts

- **From Module 1**: ROS 2 diagnostics and lifecycle
- **From Module 2**: Simulated failure injection for testing
- **From Module 3**: Isaac sensor simulation
- **In This Module**: Safety Constraints, Capstone Architecture

---

## Real-World Application

Execution monitoring enables robust autonomous operation:

**Amazon Robotics** warehouse robots continuously monitor pick success via vision and force, with multi-level recovery to maintain throughput.

**Intuitive Surgical's da Vinci** surgical robots use redundant sensing with conservative anomaly detection for patient safety.

**Boston Dynamics** robots use extensive state estimation and expectation checking to maintain balance and recover from perturbations.

The key insight is that monitoring must be tight enough to catch failures early but not so sensitive that normal variation triggers false alarms.
