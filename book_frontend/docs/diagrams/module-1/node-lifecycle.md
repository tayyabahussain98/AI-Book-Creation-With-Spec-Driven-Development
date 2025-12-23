# ROS 2 Node Lifecycle Diagram Specification

**Purpose**: Illustrate the state machine governing managed ROS 2 node lifecycles

**Diagram Type**: State machine / Finite state automaton

## States

### 1. Unconfigured (Initial State)
- **Color**: Gray
- **Description**: Node exists but resources not allocated
- **Characteristics**:
  - Node object created
  - No publishers/subscribers active
  - No hardware connections established

### 2. Inactive
- **Color**: Yellow
- **Description**: Node configured but not actively processing data
- **Characteristics**:
  - Resources allocated (topics declared)
  - Connections established but paused
  - Ready to activate quickly

### 3. Active
- **Color**: Green
- **Description**: Node fully operational and processing messages
- **Characteristics**:
  - Publishers sending data
  - Subscribers processing callbacks
  - Timers firing
  - Services/actions available

### 4. Finalized (End State)
- **Color**: Black
- **Description**: Node shutdown, resources cleaned up
- **Characteristics**:
  - All connections closed
  - Memory freed
  - Cannot return to other states

## Transitions

### From Unconfigured
- **configure()** → Inactive
  - Allocate resources
  - Declare parameters
  - Create publishers/subscribers (but don't activate)

### From Inactive
- **activate()** → Active
  - Start publishing/subscribing
  - Enable timers
  - Begin message processing
- **cleanup()** → Unconfigured
  - Release resources
  - Return to initial state

### From Active
- **deactivate()** → Inactive
  - Pause publishing/subscribing
  - Stop timers
  - Maintain connections
- **shutdown()** → Finalized
  - Emergency shutdown
  - Clean up and exit

### From Any State (Except Finalized)
- **shutdown()** → Finalized
  - Graceful or emergency termination

## Error Handling

- **ErrorProcessing State** (implicit):
  - If transition fails, node enters error state
  - Requires manual intervention or restart

## Why Lifecycle Matters

### For Robotics:
1. **Controlled Startup**: Don't activate actuators before sensors are ready
2. **Graceful Degradation**: Deactivate failed components without full shutdown
3. **Hot Swapping**: Reconfigure nodes without restarting entire system
4. **Safety**: Ensure proper initialization order (sensors before controllers)

### Example Scenario:
```
1. Robot boots → All nodes Unconfigured
2. System calls configure() on all nodes → Inactive
3. Safety checks pass → Call activate() → Active
4. Emergency stop pressed → Call deactivate() → Inactive (safe state)
5. Issue resolved → Call activate() → Active
6. Shutdown command → shutdown() → Finalized
```

## Visual Layout

```
    ┌──────────────┐
    │ Unconfigured │ (Initial)
    └───┬──────────┘
        │ configure()
        ▼
    ┌──────────┐ ◄─── cleanup()
    │ Inactive  │
    └─┬────────┘
      │ activate() / deactivate()
      ▼
    ┌──────────┐
    │  Active   │
    └─┬────────┘
      │ shutdown()
      ▼
    ┌──────────┐
    │ Finalized │ (End)
    └──────────┘
```

## Managed vs Unmanaged Nodes

- **Managed Nodes**: Follow this lifecycle (used for hardware nodes, safety-critical systems)
- **Unmanaged Nodes**: Skip lifecycle, immediately active (used for simple data processing)

## Usage in Book

- **Referenced in**: Chapter 2 (Core Concept 1: Nodes and Executors)
- **Purpose**: Explain how ROS 2 provides deterministic node startup/shutdown
- **Contrast**: Traditional software starts immediately; robotics needs controlled initialization
