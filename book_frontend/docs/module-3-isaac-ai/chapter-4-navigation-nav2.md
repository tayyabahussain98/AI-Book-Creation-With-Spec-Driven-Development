---
sidebar_position: 4
title: Chapter 4 - Autonomous Navigation with Nav2
---

# Chapter 4: Autonomous Navigation with Nav2

## Prerequisites
- **Module 1**: ROS 2 navigation basics, tf transformations
- **Chapter 3**: Perception pipelines and visual SLAM (cuVSLAM)
- **Understanding**: Basic path planning concepts (A*, Dijkstra)

## Learning Objectives
By the end of this chapter, you will be able to:

1. **Explain** the Nav2 architecture and how its components work together for autonomous navigation
2. **Identify** humanoid-specific navigation constraints (ZMP stability, footstep planning, slower velocities)
3. **Describe** how costmaps fuse static maps with dynamic sensor data for collision-free planning
4. **Evaluate** recovery behaviors and when they're triggered during navigation failures
5. **Configure** Nav2 parameters for bipedal robots (footprint, inflation, velocity limits)

---

## Introduction

Autonomous navigation is the cornerstone capability for mobile robots. A humanoid robot working in an office must navigate from point A to point B while avoiding walls, furniture, and people—all without human intervention. This requires solving three simultaneous challenges: **Where am I?** (localization), **Where should I go?** (global planning), and **How do I get there safely?** (local planning + obstacle avoidance).

**Nav2** (Navigation2) is the ROS 2 navigation stack that solves these problems for mobile robots. While originally designed for wheeled robots, Nav2's modular architecture allows adaptation for bipedal humanoids. However, humanoids face unique challenges: they walk with discrete footsteps (not continuous motion), require dynamic balance (ZMP constraints), and move much slower (0.4 m/s vs 1.0+ m/s for wheeled robots).

This chapter explores how Nav2 enables humanoid navigation, focusing on the architectural differences required for bipedal stability, footstep planning, and safe operation in human environments.

---

## Core Concept 1: Nav2 Architecture Components

### Overview

Nav2 orchestrates multiple specialized nodes that work together to achieve autonomous navigation:

1. **SLAM / Localization**: Track robot pose in the world
2. **Costmaps**: Represent obstacles and free space
3. **Planners**: Compute collision-free paths
4. **Controllers**: Execute paths with smooth motion
5. **Behavior Trees**: Coordinate actions and recovery

### Diagram: Nav2 Architecture

See [Nav2 Architecture](../diagrams/module-3/nav2-architecture.md) for a complete visualization of how these components interact.

### 1. SLAM and Localization

**Mapping (SLAM)**:
- **Purpose**: Build a 2D occupancy grid map during exploration
- **Tools**: Cartographer (lidar-based), SLAM Toolbox (flexible), or cuVSLAM (visual)
- **Output**: `nav_msgs/OccupancyGrid` (0=free, 100=occupied, -1=unknown)
- **When**: One-time mapping phase or continuous mapping in changing environments

**Localization (AMCL)**:
- **Purpose**: Track robot pose within a known map
- **Algorithm**: Adaptive Monte Carlo Localization (particle filter with 100-500 particles)
- **Input**: Lidar scans + odometry + known map
- **Output**: Robot pose with covariance (uncertainty estimate)
- **Frequency**: 10-20 Hz pose updates

**Why Both?** SLAM builds maps; AMCL localizes in pre-built maps. Most deployment scenarios use AMCL (faster, less compute).

### 2. Global Planner

**Purpose**: Find a coarse waypoint path from start to goal

**Algorithm**: A* (heuristic search) or Dijkstra (exhaustive)
- **A*** finds shortest path using distance heuristic (faster for long paths)
- **Dijkstra** explores all options (slower but guaranteed optimal)

**Inputs**: Start pose, goal pose, global costmap

**Output**: Sequence of waypoints [(x₀, y₀, θ₀), ..., (xₙ, yₙ, θₙ)]

**Frequency**: Replan every 1-5 seconds or when path blocked

**Humanoid Consideration**: Use A* (faster) since humanoids execute slowly—no need for exhaustive search.

### 3. Local Planner (Controller)

**Purpose**: Execute global path with smooth, collision-free trajectory

**Algorithm**: DWA (Dynamic Window Approach) or TEB (Timed Elastic Band)
- **DWA**: Sample velocity space, score trajectories (fast, simple)
- **TEB**: Optimize trajectory with elastic band (smoother, handles constraints better)

**Inputs**: Global path, current pose, local costmap, robot velocity

**Output**: Velocity commands (`cmd_vel`: linear + angular velocity)

**Frequency**: 10-20 Hz for wheeled robots, 5-10 Hz for humanoids

**Trajectory Scoring**:
- Path distance: How closely does trajectory follow global path?
- Goal distance: Does trajectory progress toward goal?
- Obstacle distance: How far from obstacles?
- Velocity: Prefer smoother, lower jerk trajectories

**Humanoid Choice**: DWA is typical, but TEB can produce smoother footstep transitions (at higher compute cost).

### 4. Behavior Tree Navigator

**Purpose**: Coordinate navigation actions and recovery strategies

**Behavior Tree Structure**:
```
Root (Fallback: try in order until success)
├─ Sequence: ComputePath → FollowPath
├─ Recovery: ClearCostmap → Retry
├─ Recovery: Spin (rotate 360°) → Retry
├─ Recovery: BackUp (reverse 0.5m) → Retry
└─ Failure: Give up, request human assistance
```

**Recovery Behaviors**: Triggered when navigation fails (stuck, path blocked, oscillation)
1. **ClearCostmap**: Reset dynamic obstacles (sensor glitch recovery)
2. **Spin**: Rotate 360° to gather sensor data and reorient
3. **BackUp**: Reverse 0.5m to escape local minima
4. **Wait**: Pause 5-10s for dynamic obstacles to clear

**Why Behavior Trees?** Navigation fails frequently in real environments. BTs systematically try recovery strategies instead of immediately giving up.

---

## Core Concept 2: Humanoid-Specific Navigation

### Differences from Wheeled Robots

| Constraint | Wheeled Robot | Humanoid Robot |
|------------|---------------|----------------|
| Motion Model | Continuous (differential drive) | Discrete (footsteps) |
| Max Velocity | 1.0-2.0 m/s | 0.4-0.8 m/s |
| Turning | In-place rotation (0m radius) | 0.5-1.0m turning radius |
| Stopping | Instant (friction) | 2-3 steps (1.0m distance) |
| Stability | Static (always upright) | Dynamic (ZMP constraints) |
| Footprint | Fixed rectangle | Foot placement pattern |

### Zero Moment Point (ZMP) Stability

**Definition**: The point on the ground where the net moment from gravity and inertial forces is zero.

**Stability Condition**: ZMP must remain inside the **support polygon** (convex hull of foot contact points).

**Support Phases**:
- **Single Support**: One foot on ground → small support polygon (~0.025 m²) → less stable
- **Double Support**: Both feet on ground → larger support polygon (~0.15 m²) → more stable

**Practical Implication**: Humanoids spend 60-70% of walk cycle in single support → must carefully plan footsteps to maintain ZMP within foot boundary.

**Safety Margin**: Keep ZMP at least 2cm from support polygon edge to tolerate:
- External pushes (e.g., person bumps into robot)
- Uneven terrain (e.g., slight floor slope)
- Model uncertainties (e.g., CoM estimation error)

### Footstep Planning

**Challenge**: Nav2 provides continuous waypoint paths. Humanoids need discrete foot placements.

**Workflow**:
1. **Discretization**: Sample Nav2 path at step_length intervals (0.2-0.4m)
2. **ZMP Check**: Verify each footstep maintains balance (ZMP inside support polygon)
3. **Collision Check**: Ensure foot doesn't land on obstacle (query costmap)
4. **Kinematics**: Verify hip/knee/ankle can reach target (IK solvable)
5. **Trajectory**: Generate smooth swing trajectory (lift, swing, land)

**Turning Constraint**: Max 30° turn per step → 90° turn requires 3 steps (2.4s vs 0.5s for wheeled)

**Stopping Constraint**: Must complete current step before stopping (cannot stop mid-swing)

### Diagram: Humanoid Footstep Planning

See [Humanoid Footstep Planning](../diagrams/module-3/humanoid-footstep-planning.md) for detailed ZMP calculations and footstep sequence generation.

---

## Core Concept 3: Dynamic Obstacle Avoidance and Replanning

### Costmap Updates

**Local Costmap** (3×3m rolling window, 5-10 Hz updates):
- Fuses static map + real-time sensor data (lidar, depth camera)
- Marks dynamic obstacles (people, moving robots)
- Clears free space via raycasting

**Update Cycle**:
1. Sensor data arrives (e.g., lidar scan at 10 Hz)
2. Mark obstacle cells where lidar detects objects
3. Clear free space cells along lidar rays
4. Apply inflation layer (expand obstacles by safety margin)
5. Publish updated costmap

**Effect**: Local planner sees updated obstacles within 100ms, triggers replanning if path blocked.

### Local Planner Replanning

**Triggers**:
- Current trajectory collides with new obstacle (cost > threshold)
- Path oscillation detected (robot stuck)
- Goal unreachable (timeout)

**Response**:
1. **Immediate**: Try alternative trajectory (shift laterally, reduce velocity)
2. **Short-term** (0.5s): Request new global path from planner server
3. **Long-term** (5s): Trigger recovery behavior (backup, spin, clear costmap)

**Humanoid Adaptation**: Allow 200-500ms replanning latency (footstep transitions are slow). Wheeled robots replan in 50ms.

### Recovery Behaviors for Humanoids

Standard Nav2 recovery behaviors adapted for bipedal robots:

1. **ClearCostmap**: Reset obstacle memory (phantom obstacles from sensor noise)
2. **Spin**: Rotate 360° in place using footsteps (takes 8-10 steps, ~8s)
3. **SideStep**: Move laterally 0.3m to avoid obstacle (humanoid-specific)
4. **Squat**: Lower center of mass to increase stability before retry
5. **BackUp**: Reverse 0.5m using backward steps (slower than wheeled)

**When to Use**:
- Stuck in narrow corridor → SideStep or BackUp
- Lost localization (AMCL diverged) → Spin to re-localize
- Sensor glitch (false obstacles) → ClearCostmap
- Crowded area (dynamic obstacles) → Wait behavior

---

## Core Concept 4: Costmap Configuration for Bipedal Robots

### Footprint Geometry

**Wheeled Robot**: Single rectangular footprint (e.g., 0.5m × 0.4m)

**Humanoid Robot**: Defined by foot placement pattern
- Foot dimensions: 0.25m (length) × 0.1m (width)
- Foot separation: 0.2m (side-to-side)
- Effective footprint: ~0.3m × 0.3m (narrower than wheeled)

**Nav2 Configuration**:
```yaml
footprint: "[[0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]"
# Rectangular 0.3m × 0.3m approximation
```

### Inflation Radius

**Purpose**: Add safety margin around obstacles

**Wheeled Robot**: 0.3m inflation (robot radius + 0.1m safety)

**Humanoid Robot**: 0.5m inflation (larger margin due to:
- Bipedal instability (sway during walking)
- Longer stopping distance (2-3 steps = 1.0m)
- Balance recovery space (may need side-step)

**Cost Gradient**:
```
Cost(distance) = 253 × exp(-k × (distance - robot_radius))
k = 5.0 for humanoids (vs 3.0 for wheeled) → steeper gradient
```

**Effect**: Planner avoids obstacles more conservatively, routing humanoid with wider clearance.

### Voxel Layer for 3D Obstacles

**Why Humanoids Need 3D Awareness**:
- **Low obstacles** (0.1-0.3m): Steps, curbs → trip hazard
- **High obstacles** (0.7-2.0m): Tables, desks → head collision

**Configuration**:
```yaml
voxel_layer:
  max_obstacle_height: 2.0  # Detect up to 2m (head height)
  min_obstacle_height: 0.05 # Detect down to 5cm (small step)
  z_resolution: 0.05        # 5cm vertical bins
  z_voxels: 40              # 2.0m / 0.05m = 40 layers
```

**Behavior**: Costmap marks cells as occupied if **any** voxel in the column is occupied → prevents walking under tables or over steps.

### Diagram: Costmap Layers

See [Costmap Layers](../diagrams/module-3/costmap-layers.md) for detailed layer composition and inflation calculations.

---

## Hands-On Example: Nav2 Humanoid Config

Below is a conceptual Nav2 configuration file showing key parameters adapted for humanoid navigation.

**File**: [nav2_params_humanoid.yaml](../code-examples/module-3/nav2_params_humanoid.yaml)

```yaml
# Nav2 Parameters for Humanoid Robot

controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Lower than wheeled (20 Hz)

    DWBLocalPlanner:
      # Velocity limits (much slower than wheeled)
      max_vel_x: 0.4       # m/s (vs 0.8-1.5 for wheeled)
      max_vel_theta: 0.5   # rad/s (vs 1.0-2.0 for wheeled)

      # Acceleration limits (conservative for balance)
      acc_lim_x: 0.3       # m/s² (vs 1.0+ for wheeled)
      acc_lim_theta: 0.5   # rad/s² (vs 2.0+ for wheeled)

      # Footprint geometry (narrower than wheeled)
      footprint: "[[0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15]]"

local_costmap:
  ros__parameters:
    robot_radius: 0.2      # Safety margin beyond footprint
    plugins: ["voxel_layer", "inflation_layer"]

    voxel_layer:
      max_obstacle_height: 2.0  # Detect obstacles up to 2m
      z_resolution: 0.05

    inflation_layer:
      inflation_radius: 0.5      # 50cm inflation (vs 30cm wheeled)
      cost_scaling_factor: 5.0   # Steeper gradient (vs 3.0 wheeled)
```

**Key Takeaways**:
1. Velocity limits reduced 50% vs wheeled robots
2. Wider inflation radius (0.5m) for balance safety margin
3. Voxel layer enabled to detect steps and overhead obstacles
4. Lower control frequency (10 Hz) matches slower footstep execution

---

## Summary

This chapter introduced **Nav2 navigation** adapted for humanoid robots. Key takeaways:

1. **Nav2 architecture**: SLAM/localization + costmaps + global/local planners + behavior trees enable autonomous navigation
2. **Humanoid constraints**: Discrete footsteps, ZMP stability, 50% slower velocities, larger safety margins
3. **Footstep planning**: Discretize Nav2 paths into foot placements with ZMP and collision checking
4. **Dynamic obstacle avoidance**: Local costmap fuses sensor data at 5-10 Hz, triggers replanning when obstacles detected
5. **Recovery behaviors**: ClearCostmap, Spin, SideStep, BackUp handle navigation failures systematically
6. **Costmap tuning**: Wider inflation (0.5m), voxel layer for 3D obstacles, conservative velocity limits

**Next Steps**: Chapter 5 explores **sim-to-real transfer** using reinforcement learning and domain randomization to train robust navigation policies in Isaac Sim that transfer to real humanoid robots.

---

## Self-Assessment

Test your understanding of Nav2 navigation for humanoids:

1. **What are the three main components of the Nav2 architecture and their purposes?**
   <details>
   <summary>Show Answer</summary>
   (1) **SLAM/Localization**: Track robot pose in the world (AMCL for known maps, Cartographer/SLAM Toolbox for mapping). (2) **Planners**: Global planner (A*/Dijkstra) finds waypoint path; local planner (DWA/TEB) generates smooth trajectory. (3) **Costmaps**: Fuse static map + sensor data + inflation to represent obstacles for collision-free planning.
   </details>

2. **Why do humanoids navigate slower than wheeled robots?**
   <details>
   <summary>Show Answer</summary>
   Humanoids use discrete footsteps (0.3m step length, 0.8s per step → 0.4 m/s max) vs wheeled robots' continuous motion (1.0-2.0 m/s). Humanoids also have turning radius constraints (0.5-1.0m vs instant in-place rotation) and require 2-3 steps to stop (vs instant friction-based stopping).
   </details>

3. **What is the Zero Moment Point (ZMP) and why does it matter for humanoid navigation?**
   <details>
   <summary>Show Answer</summary>
   ZMP is the point on the ground where net moment from gravity and inertia is zero. For stability, ZMP must remain inside the support polygon (convex hull of foot contact points). Violating this constraint causes the robot to fall. Footstep planners must verify ZMP stays at least 2cm from polygon edge for safety.
   </details>

4. **How do costmap layers work together to represent obstacles?**
   <details>
   <summary>Show Answer</summary>
   **Static Layer**: Pre-built map (walls, furniture). **Obstacle Layer**: Real-time sensor data (people, moving objects). **Voxel Layer**: 3D obstacles by height (steps, tables). **Inflation Layer**: Expands obstacles by safety margin (0.5m for humanoids). Layers fused into unified costmap with cost gradient (0=free, 254=lethal).
   </details>

5. **Name three recovery behaviors in Nav2 and when each is used.**
   <details>
   <summary>Show Answer</summary>
   (1) **ClearCostmap**: Reset obstacle memory when phantom obstacles from sensor noise cause stuck condition. (2) **Spin**: Rotate 360° to gather sensor data and re-localize when AMCL diverges. (3) **BackUp**: Reverse 0.5m to escape local minima when planner oscillates between invalid trajectories.
   </details>

6. **Why do humanoids need a voxel layer in their costmap?**
   <details>
   <summary>Show Answer</summary>
   Voxel layer provides 3D obstacle awareness. Humanoids must avoid **low obstacles** (0.1-0.3m steps/curbs → trip hazard) and **high obstacles** (0.7-2.0m tables/desks → head collision). Wheeled robots only care about ground-level obstacles. Voxel layer detects obstacles by height using depth cameras or 3D lidar.
   </details>

7. **How does footstep planning adapt Nav2's continuous paths for humanoid robots?**
   <details>
   <summary>Show Answer</summary>
   Footstep planner discretizes Nav2's waypoint path at step_length intervals (0.2-0.4m), places feet perpendicular to path tangent, alternates left/right with foot_width separation (0.2m). Each footstep is verified for: (1) ZMP stability (inside support polygon), (2) collision-free (query costmap), (3) kinematic reachability (IK solvable), (4) smooth swing trajectory (lift-swing-land).
   </details>

8. **Why is the inflation radius larger for humanoids (0.5m) compared to wheeled robots (0.3m)?**
   <details>
   <summary>Show Answer</summary>
   Humanoids require wider safety margins due to: (1) bipedal instability (sway during walking), (2) longer stopping distance (1.0m for 2-3 footsteps vs 0.3m instant friction stop), (3) balance recovery space (may need emergency side-step). Wider inflation prevents collisions during dynamic maneuvers.
   </details>
