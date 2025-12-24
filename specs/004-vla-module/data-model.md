# Data Model: Module 4 - Vision-Language-Action (VLA)

**Feature**: 004-vla-module
**Date**: 2025-12-24
**Purpose**: Define conceptual entities for VLA educational content

---

## Overview

This data model defines the conceptual entities taught in Module 4. These are **educational abstractions** used to explain VLA systems, not database schemas or API specifications. The model aligns with the three-layer architecture (Perception → Planning → Execution).

---

## Entity Definitions

### 1. VoiceCommand

**Description**: Natural language input from user, before and after speech recognition processing.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| raw_audio | AudioStream | Raw audio input from microphone |
| transcript | String | Text output from speech recognition |
| confidence | Float (0-1) | Speech recognition confidence score |
| timestamp | DateTime | When command was received |
| language | String | Detected or specified language code |

**Relationships**:
- Produces one or more `Intent` after parsing
- May reference `SceneState` for context resolution

**State Transitions**:
```
RECORDING → TRANSCRIBING → TRANSCRIBED → PARSING → PARSED
```

---

### 2. Intent

**Description**: The parsed action type representing what the user wants the robot to do.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| action_type | Enum | Primary action (pick, place, move, find, etc.) |
| confidence | Float (0-1) | Intent classification confidence |
| original_text | String | Source text from VoiceCommand |
| requires_clarification | Boolean | Whether intent is ambiguous |

**Supported Action Types**:
- `PICK` - Grasp an object
- `PLACE` - Put object at location
- `MOVE` - Navigate to position
- `FIND` - Locate object or person
- `SPEAK` - Generate verbal response
- `WAIT` - Pause execution
- `STOP` - Emergency halt

**Relationships**:
- Extracted from one `VoiceCommand`
- Contains zero or more `Entity` objects
- Triggers creation of `ActionGraph`

---

### 3. Entity

**Description**: Parameters extracted from commands - objects, locations, attributes, quantities.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| entity_type | Enum | OBJECT, LOCATION, ATTRIBUTE, QUANTITY, PERSON |
| value | String | Extracted value (e.g., "cup", "kitchen") |
| confidence | Float (0-1) | Entity extraction confidence |
| is_resolved | Boolean | Whether entity maps to known item |
| resolved_id | String | ID of resolved object/location |

**Examples**:
- "Pick up the **red cup**" → OBJECT: cup, ATTRIBUTE: red
- "Move to the **kitchen**" → LOCATION: kitchen
- "Give **three** apples to **John**" → QUANTITY: 3, PERSON: John

**Relationships**:
- Belongs to one `Intent`
- May reference `SceneState` for resolution
- Used by `PrimitiveAction` as parameters

---

### 4. ActionGraph

**Description**: Directed acyclic graph (DAG) of primitive actions representing a decomposed task.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| root_intent | Intent | Original intent that spawned this graph |
| nodes | List[PrimitiveAction] | Actions in execution order |
| edges | List[Dependency] | Dependencies between actions |
| status | Enum | PLANNING, READY, EXECUTING, COMPLETED, FAILED |
| created_at | DateTime | When graph was generated |

**Graph Properties**:
- Acyclic (no circular dependencies)
- Single root node (entry point)
- May have multiple leaf nodes (parallel endings)
- Topologically sortable for execution order

**Example Graph**:
```
"Clean the table" →

    [locate_table]
          ↓
    [identify_objects]
          ↓
    [pick_object_1] → [pick_object_2] → [pick_object_3]
          ↓                ↓                 ↓
    [place_in_bin] → [place_in_bin] → [place_in_bin]
                              ↓
                        [wipe_surface]
```

**Relationships**:
- Created from one `Intent`
- Contains multiple `PrimitiveAction` nodes
- Monitored by `ExecutionState`

---

### 5. PrimitiveAction

**Description**: Atomic robot capability that cannot be further decomposed.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| action_id | String | Unique identifier |
| action_type | Enum | Type of primitive (see below) |
| parameters | Dict | Action-specific parameters |
| preconditions | List[Condition] | Required conditions to start |
| postconditions | List[Condition] | Expected conditions after completion |
| timeout | Duration | Maximum execution time |
| status | Enum | PENDING, EXECUTING, COMPLETED, FAILED, SKIPPED |

**Primitive Action Types**:
| Type | Description | Parameters |
|------|-------------|------------|
| `MOVE_TO` | Navigate to position | target_pose, speed |
| `GRASP` | Close gripper on object | object_id, force |
| `RELEASE` | Open gripper | speed |
| `LOOK_AT` | Direct gaze | target_point |
| `SPEAK` | Generate speech | text, volume |
| `WAIT` | Time delay | duration |
| `VERIFY` | Check condition | condition |

**Relationships**:
- Belongs to one `ActionGraph`
- May depend on other `PrimitiveAction` nodes
- Executed by robot controller
- Monitored for `SafetyConstraint` violations

---

### 6. SceneState

**Description**: Visual understanding of the environment including objects, positions, and relationships.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| timestamp | DateTime | When scene was captured |
| objects | List[DetectedObject] | Objects in scene |
| relationships | List[SpatialRelation] | Spatial relationships |
| free_space | OccupancyGrid | Navigable areas |
| confidence | Float (0-1) | Overall scene understanding confidence |

**DetectedObject Structure**:
| Field | Type | Description |
|-------|------|-------------|
| object_id | String | Unique identifier |
| class_name | String | Object category (cup, table, etc.) |
| position | Pose3D | 6DoF position in world frame |
| bounding_box | BBox3D | 3D bounding volume |
| attributes | Dict | Color, size, material, etc. |
| confidence | Float | Detection confidence |

**SpatialRelation Structure**:
| Field | Type | Description |
|-------|------|-------------|
| subject | String | Object ID |
| relation | Enum | ON, NEXT_TO, INSIDE, ABOVE, BELOW |
| object | String | Reference object ID |

**Relationships**:
- Used by `Intent` for entity resolution
- Consulted by `ActionGraph` for planning
- Updated continuously during execution

---

### 7. ExecutionState

**Description**: Runtime status of action sequence execution.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| action_graph_id | String | Currently executing graph |
| current_action | String | Active primitive action |
| completed_actions | List[String] | Finished action IDs |
| failed_actions | List[String] | Failed action IDs |
| start_time | DateTime | When execution began |
| elapsed_time | Duration | Time since start |
| status | Enum | IDLE, EXECUTING, PAUSED, COMPLETED, FAILED, ABORTED |

**Status Transitions**:
```
IDLE → EXECUTING → COMPLETED
           ↓
        PAUSED → EXECUTING
           ↓
        FAILED → (replanning) → EXECUTING
           ↓
        ABORTED
```

**Relationships**:
- Monitors one `ActionGraph`
- Triggers replanning on failure
- Reports to safety monitor

---

### 8. SafetyConstraint

**Description**: Rule that limits robot behavior for safe operation.

**Attributes**:
| Attribute | Type | Description |
|-----------|------|-------------|
| constraint_id | String | Unique identifier |
| constraint_type | Enum | Type (see below) |
| parameters | Dict | Constraint-specific limits |
| severity | Enum | WARNING, STOP, EMERGENCY |
| is_active | Boolean | Whether constraint is enabled |

**Constraint Types**:
| Type | Description | Parameters |
|------|-------------|------------|
| `FORCE_LIMIT` | Maximum contact force | max_force_n |
| `VELOCITY_LIMIT` | Maximum joint/end-effector speed | max_vel |
| `EXCLUSION_ZONE` | Forbidden regions | zone_geometry |
| `COLLISION_AVOID` | Minimum distance to obstacles | min_distance |
| `INTENT_VERIFY` | Confirm dangerous commands | actions_list |
| `WATCHDOG` | Timeout for system responsiveness | timeout_ms |

**Relationships**:
- Monitored during all `PrimitiveAction` execution
- Can pause or abort `ExecutionState`
- Logged for safety audit

---

## Entity Relationship Diagram

```
┌─────────────────┐
│  VoiceCommand   │
└────────┬────────┘
         │ parses to
         ▼
┌─────────────────┐     ┌─────────────────┐
│     Intent      │────►│     Entity      │
└────────┬────────┘     └─────────────────┘
         │ decomposes to            │
         ▼                          │ references
┌─────────────────┐                 │
│   ActionGraph   │                 ▼
└────────┬────────┘     ┌─────────────────┐
         │ contains     │   SceneState    │
         ▼              └─────────────────┘
┌─────────────────┐
│ PrimitiveAction │
└────────┬────────┘
         │ monitored by
         ▼
┌─────────────────┐     ┌─────────────────┐
│ ExecutionState  │────►│SafetyConstraint │
└─────────────────┘     └─────────────────┘
```

---

## Usage in Chapters

| Entity | Primary Chapter | Supporting Chapters |
|--------|-----------------|---------------------|
| VoiceCommand | Ch 2: Voice-to-Action | Ch 4, Ch 5 |
| Intent | Ch 2: Voice-to-Action | Ch 3, Ch 4 |
| Entity | Ch 2: Voice-to-Action | Ch 4 |
| ActionGraph | Ch 3: LLM Planning | Ch 5 |
| PrimitiveAction | Ch 3: LLM Planning | Ch 5 |
| SceneState | Ch 4: Multimodal Fusion | Ch 2, Ch 5 |
| ExecutionState | Ch 5: Capstone | Ch 3 |
| SafetyConstraint | Ch 5: Capstone | All chapters |
