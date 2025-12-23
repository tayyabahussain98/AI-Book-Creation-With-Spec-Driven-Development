# Nav2 Architecture

**Purpose**: Illustrate the complete Nav2 (Navigation2) architecture for ROS 2, showing how SLAM/localization, global/local planning, costmap management, and recovery behaviors work together to enable autonomous navigation.

**Context**: Nav2 is the ROS 2 navigation stack used by mobile robots (wheeled, legged, humanoid) for autonomous navigation from point A to point B while avoiding obstacles.

---

## ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    NAV2 ARCHITECTURE                             │
└─────────────────────────────────────────────────────────────────┘

MAPPING & LOCALIZATION LAYER
━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌──────────────────┐         ┌──────────────────┐
│  SLAM (Mapping)  │         │  AMCL (Adaptive  │
│  • Cartographer  │         │  Monte Carlo     │
│  • SLAM Toolbox  │────────>│  Localization)   │
│  Builds map      │  map    │  Tracks pose     │
└────────┬─────────┘         └────────┬─────────┘
         │                            │
         v                            v
┌─────────────────────────────────────────────────────────────────┐
│                    STATIC MAP (OccupancyGrid)                    │
│  • 2D grid representation (0=free, 100=occupied, -1=unknown)    │
│  • Resolution: 5cm typical                                       │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             v

COSTMAP LAYER (LOCAL + GLOBAL)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│                         COSTMAP LAYERS                           │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │Static Layer │→ │Obstacle     │→ │Inflation    │→ Costmap    │
│  │(from map)   │  │Layer (sens.)│  │Layer        │  (0-254)    │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│                                                                  │
│  Local Costmap:  3x3m rolling window, 5 Hz updates              │
│  Global Costmap: Full map, 1 Hz updates                         │
└────────┬────────────────────────────────────────────────────────┘
         │
         v

PLANNING LAYER
━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│                     PLANNER SERVER                               │
├─────────────────────────────────────────────────────────────────┤
│  Global Planner (nav2_navfn_planner)                            │
│  ┌────────────────────────────────────┐                         │
│  │  Input:  Start pose, Goal pose     │                         │
│  │  Uses:   Global costmap            │                         │
│  │  Algorithm: A* or Dijkstra         │                         │
│  │  Output: Waypoint path (global)    │                         │
│  └────────────────────────────────────┘                         │
└────────┬────────────────────────────────────────────────────────┘
         │ Global path (waypoints)
         v
┌─────────────────────────────────────────────────────────────────┐
│                   CONTROLLER SERVER                              │
├─────────────────────────────────────────────────────────────────┤
│  Local Planner (dwb_core::DWBLocalPlanner or TEB)               │
│  ┌────────────────────────────────────┐                         │
│  │  Input:  Global path, current pose │                         │
│  │  Uses:   Local costmap             │                         │
│  │  Algorithm: DWA or TEB             │                         │
│  │  Output: cmd_vel (twist commands)  │                         │
│  └────────────────────────────────────┘                         │
│                                                                  │
│  Trajectory Scoring:                                            │
│  • Path distance (follow global path)                           │
│  • Goal distance (progress toward goal)                         │
│  • Obstacle distance (collision avoidance)                      │
│  • Velocity (prefer smoother motion)                            │
└────────┬────────────────────────────────────────────────────────┘
         │ cmd_vel (geometry_msgs/Twist)
         v
┌──────────────────┐
│   Robot Base     │
│   Controller     │
│   (wheels/legs)  │
└──────────────────┘

BEHAVIOR COORDINATION
━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│                      BT NAVIGATOR                                │
│                   (Behavior Tree Executor)                       │
├─────────────────────────────────────────────────────────────────┤
│  Behavior Tree (BT) XML:                                         │
│                                                                  │
│    Root (Fallback - try in order)                               │
│      ├─ ComputePathToPose (Global plan)                         │
│      │   └─ FollowPath (Local plan)                             │
│      ├─ Recovery Behaviors (if navigation fails):               │
│      │   ├─ ClearCostmap (reset obstacle memory)                │
│      │   ├─ Spin (rotate in place to reorient)                  │
│      │   ├─ BackUp (reverse to escape local minima)             │
│      │   └─ Wait (pause for dynamic obstacles)                  │
│      └─ Goal reached? Success                                   │
│                                                                  │
│  BT ticks at 10 Hz, coordinates all nav actions                 │
└────────┬────────────────────────────────────────────────────────┘
         │
         v

RECOVERY BEHAVIORS
━━━━━━━━━━━━━━━━━━
┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  Clear Costmap   │  │   Spin (Rotate)  │  │  BackUp (Reverse)│
│  Reset obstacles │  │  360° rotation   │  │  Move backward   │
│  if stuck        │  │  to reorient     │  │  0.5m to escape  │
└──────────────────┘  └──────────────────┘  └──────────────────┘

LIFECYCLE MANAGEMENT
━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│                    LIFECYCLE MANAGER                             │
├─────────────────────────────────────────────────────────────────┤
│  Manages lifecycle states for all Nav2 nodes:                   │
│  • Unconfigured → Configuring → Inactive → Active               │
│  • Handles failures, restarts nodes                             │
│  • Coordinates startup/shutdown of navigation stack             │
└─────────────────────────────────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════
DATA FLOW SUMMARY
═══════════════════════════════════════════════════════════════════

1. Sensors → SLAM → Static Map
2. Static Map + Sensor Data → Costmap Layers → Local/Global Costmaps
3. Goal Pose + Global Costmap → Global Planner → Waypoint Path
4. Waypoint Path + Local Costmap → Local Planner → cmd_vel
5. cmd_vel → Robot Base → Motion
6. If stuck → BT Navigator → Recovery Behaviors → Retry
```

---

## Component Explanations

### 1. Mapping & Localization

**SLAM (Cartographer / SLAM Toolbox)**:
- **Purpose**: Build a 2D occupancy grid map of the environment
- **Input**: Lidar scans, odometry, IMU
- **Output**: `nav_msgs/OccupancyGrid` (static map)
- **When**: During initial exploration or unknown environments

**AMCL (Adaptive Monte Carlo Localization)**:
- **Purpose**: Track robot pose within a known map
- **Algorithm**: Particle filter (maintains multiple pose hypotheses)
- **Input**: Lidar scans, odometry, known map
- **Output**: `geometry_msgs/PoseWithCovarianceStamped` (robot pose)
- **When**: During navigation in pre-built maps

### 2. Costmap Layers

**Static Layer**:
- Source: Pre-built map from SLAM
- Represents permanent obstacles (walls, furniture)
- Updated rarely (1 Hz or on map change)

**Obstacle Layer**:
- Source: Real-time sensor data (lidar, depth cameras)
- Represents dynamic obstacles (people, moving objects)
- Updated frequently (5-10 Hz)

**Inflation Layer**:
- Expands obstacles by robot radius + safety margin
- Creates cost gradient: 254 (lethal) → 253 (inscribed) → ... → 0 (free)
- Prevents collision by keeping robot center away from obstacles

**Voxel Layer** (optional, for 3D sensors):
- 3D obstacle representation (height-aware)
- Important for humanoids: detect low obstacles (steps) and high obstacles (tables)

**Local vs Global Costmap**:
- **Local**: 3x3m rolling window, high-frequency updates (5 Hz), detailed
- **Global**: Full map, low-frequency updates (1 Hz), coarse planning

### 3. Planning

**Global Planner (Planner Server)**:
- **Algorithm**: A* (heuristic search) or Dijkstra (exhaustive search)
- **Input**: Start pose, goal pose, global costmap
- **Output**: Coarse waypoint path from start to goal
- **Frequency**: Replan every 1-5 seconds or when path blocked
- **Pros**: Finds optimal path avoiding known obstacles
- **Cons**: Doesn't consider robot dynamics (velocity, acceleration)

**Local Planner (Controller Server)**:
- **Algorithm**: DWA (Dynamic Window Approach) or TEB (Timed Elastic Band)
- **Input**: Global path, current pose, local costmap
- **Output**: Velocity commands (`cmd_vel`) for smooth trajectory
- **Frequency**: 10-20 Hz (real-time control)
- **Pros**: Considers robot dynamics, avoids dynamic obstacles
- **Cons**: Can get stuck in local minima without recovery

**DWA vs TEB**:
- **DWA**: Samples velocity space, scores trajectories (fast, simple)
- **TEB**: Optimizes trajectory with elastic band (smoother, handles constraints better)
- **Humanoids**: DWA preferred for simplicity; TEB for smooth footstep transitions

### 4. Behavior Tree Navigator

**Purpose**: Coordinate navigation actions and recovery behaviors

**Behavior Tree Structure**:
```
Fallback (try in order until success)
├─ Sequence: ComputePath → FollowPath
├─ Recovery: ClearCostmap → Retry
├─ Recovery: Spin → Retry
├─ Recovery: BackUp → Retry
└─ Failure (give up)
```

**Recovery Behaviors**:
1. **ClearCostmap**: Reset dynamic obstacles (if sensor glitch)
2. **Spin**: Rotate 360° to gather sensor data, reorient
3. **BackUp**: Reverse 0.5m to escape local minima
4. **Wait**: Pause 5-10s for dynamic obstacles to clear

**When Recovery Triggers**:
- Local planner fails to find valid trajectory (oscillation, stuck)
- Path blocked by new obstacle
- Goal unreachable (timeout)

### 5. Lifecycle Management

**Purpose**: Manage startup, shutdown, and failure recovery of Nav2 nodes

**Lifecycle States**:
1. **Unconfigured**: Node exists but not initialized
2. **Inactive**: Configured but not processing data
3. **Active**: Fully operational
4. **Finalized**: Shutting down

**Why It Matters**: If a node crashes (e.g., local planner), Lifecycle Manager restarts it without killing the entire navigation stack.

---

## Key Insights

### 1. Two-Level Planning Hierarchy
**Why?**: Global planner finds long-term path (slow, 1 Hz); local planner executes short-term trajectory (fast, 10-20 Hz). This separation enables efficient planning + reactive obstacle avoidance.

### 2. Costmap Fusion
**Why?**: Static map provides structure (walls); dynamic obstacles from sensors; inflation adds safety margin. Fusion creates unified representation for planning.

### 3. Behavior Trees for Robustness
**Why?**: Navigation often fails (path blocked, sensor noise, odometry drift). BT systematically tries recovery strategies instead of giving up immediately.

### 4. Local vs Global Trade-off
**Global Costmap**: Coarse, infrequent updates, full map → long-term planning
**Local Costmap**: Fine, frequent updates, small window → reactive control
Result: Efficient computation + responsive obstacle avoidance

### 5. Humanoid-Specific Considerations
- **Slower velocities**: 0.4 m/s vs 1.0+ m/s for wheeled robots
- **Wider safety margins**: 0.5m inflation vs 0.3m (bipedal instability)
- **Voxel layer**: Detect steps, curbs, low/high obstacles
- **Footstep planning**: Replace DWA with ZMP-aware planner (advanced)

---

## Real-World Application

**Use Case**: Office humanoid robot navigating from desk to meeting room

**Phase 1: Mapping** (one-time)
- SLAM Toolbox builds map while teleoperating robot
- Save map: 50m × 30m office (1500 m²)

**Phase 2: Autonomous Navigation**
- User sends goal: "Meeting Room A"
- Global Planner: A* path through hallways (15m path)
- Local Planner: DWA follows path, avoids person (replans locally)
- Recovery: Person blocks path → BackUp → Replan → Continue

**Performance**:
- Planning: 100ms (global) + 50ms (local) = 150ms total
- Frequency: Global replan every 5s, local control 10 Hz
- Success Rate: 95% (5% failures trigger recovery behaviors)
- Failure Modes: Narrow doorways, dynamic crowds, sensor glitches

**Cost-Benefit**:
- Nav2 is free, open-source (ROS 2)
- Alternative: Proprietary navigation ($$$ licensing)
- Trade-off: Nav2 requires tuning for specific robots/environments
