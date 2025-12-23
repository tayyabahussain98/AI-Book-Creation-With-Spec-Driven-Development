# Costmap Layers

**Purpose**: Illustrate how Nav2 fuses multiple data sources (static map, sensor data, inflation) into a unified costmap representation used for collision-free path planning.

**Context**: Costmaps are 2D grids where each cell contains a cost value (0=free, 254=lethal obstacle). Planners avoid high-cost areas to prevent collisions.

---

## ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    COSTMAP LAYER COMPOSITION                     │
└─────────────────────────────────────────────────────────────────┘

LAYER 1: STATIC LAYER (from SLAM map)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────┐
│  ███████████░░░░░░░░░░░░░░░░░░░░░  │  Legend:
│  █        █░░░░░░░░░░░░░░░░░░░░░  │  █ = Wall (cost 254)
│  █  Room  █░░░░░░░░░░░░░░░░░░░░░  │  ░ = Free space (cost 0)
│  █        █░░░░░░░░░░░░░░░░░░░░░  │  ▓ = Unknown (cost 255)
│  ████  ████░░░░░░░░░░░░░░░░░░░░░  │
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  Static obstacles:
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  • Walls, furniture
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  • Rarely change
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  • From pre-built map
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  Update: 1 Hz
└─────────────────────────────────────┘
                ↓
              MERGE
                ↓

LAYER 2: OBSTACLE LAYER (from sensors)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────┐
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  Legend:
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  ◆ = Dynamic obstacle
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │      (person, box)
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │      cost 254
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  ░ = Free space (cost 0)
│  ░░░░░░░░░░░░░░░░░░◆◆◆░░░░░░░░░░░  │
│  ░░░░░░░░░░░░░░░░░░◆◆◆░░░░░░░░░░░  │  Dynamic obstacles:
│  ░░░░░░░░░░░░░░░░░░◆◆◆░░░░░░░░░░░  │  • People, robots
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  • Frequently change
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  • From lidar/depth cam
└─────────────────────────────────────┘  Update: 5-10 Hz
                ↓
              MERGE
                ↓

LAYER 3: VOXEL LAYER (3D obstacles - optional)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────┐
│  Side View (X-Z plane):             │
│  ┌───────────────────────────────┐  │  Legend:
│  │ 2.0m ░░░░░░░░░░░░░░░░░░░░░░░░ │  │  ■ = High obstacle
│  │      ░░░░░░░░░░░░░░░░░░░░░░░░ │  │      (table, desk)
│  │ 1.5m ░░░░░░░░░░■■■░░░░░░░░░░░ │  │      0.7-1.5m height
│  │      ░░░░░░░░░░■■■░░░░░░░░░░░ │  │  ▪ = Low obstacle
│  │ 1.0m ░░░░░░░░░░■■■░░░░░░░░░░░ │  │      (step, curb)
│  │      ░░░░░░░░░░░░░░░░░░░░░░░░ │  │      0.1-0.3m height
│  │ 0.5m ░░░░░░░░░░░░░░░░░░░░░░░░ │  │  ░ = Free space
│  │      ░░░░▪▪▪░░░░░░░░░░░░░░░░░ │  │
│  │ 0.0m ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ │  │  ▓ = Ground
│  └───────────────────────────────┘  │
│                                     │  Important for humanoids:
│  3D voxel grid (0.05m resolution)  │  • Detect steps (trip hazard)
│  Collapsed to 2D for costmap       │  • Detect tables (head collision)
└─────────────────────────────────────┘  Update: 5 Hz
                ↓
              MERGE
                ↓

COMBINED COSTMAP (before inflation)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────┐
│  ███████████░░░░░░░░░░░░░░░░░░░░░  │  Cost values:
│  █        █░░░░░░░░░░░░░░░░░░░░░  │  254 = Lethal obstacle
│  █  Room  █░░░░░░░░░░░░░░░░░░░░░  │  253 = Inscribed (footprint)
│  █        █░░░░░░░░░░░░░░░░░░░░░  │  128 = Possibly circumscribed
│  ████  ████░░░░░░░░░░░░░░░░░░░░░  │    0 = Free space
│  ░░░░░░░░░░░░░░░░░░◆◆◆░░░░░░░░░░  │  255 = Unknown
│  ░░░░░░░░░░░░░░░░░░◆◆◆░░░░░░░░░░  │
│  ░░░░░░░░░░░░░░░░░░◆◆◆░░░░░░░░░░  │  Obstacles merged:
│  ░░░░▪▪▪░░░░░░░░░░░░░░░░░░░░░░░░  │  • Static (walls)
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  • Dynamic (person)
└─────────────────────────────────────┘  • 3D (step, table)
                ↓
          INFLATION LAYER
                ↓

LAYER 4: INFLATION LAYER (safety margins)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────┐
│  ███████████▓▓▓░░░░░░░░░░░░░░░░░░  │  Cost gradient:
│  ██▓▓▓▓▓▓██▓▓▓░░░░░░░░░░░░░░░░░░  │  254 = Lethal (obstacle)
│  ██▓Room▓██▓▓▓░░░░░░░░░░░░░░░░░░  │  253 = Inscribed (footprint)
│  ██▓▓▓▓▓▓██▓▓▓░░░░░░░░░░░░░░░░░░  │  200 = High cost (close)
│  ████▓▓████▓▓▓░░░░░░░░░░░░░░░░░░  │  100 = Medium cost
│  ░░░▓▓▓░░░░░░░░░░▓▓▓▓▓░░░░░░░░░░  │   50 = Low cost
│  ░░░░░░░░░░░░░░░▓▓◆◆◆▓▓░░░░░░░░░  │    0 = Free space
│  ░░░░░░░░░░░░░░░▓▓◆◆◆▓▓░░░░░░░░░  │
│  ░░░▪▪▪░░░░░░░░░░▓▓▓▓▓░░░░░░░░░░  │  Inflation parameters:
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │  • Radius: 0.5m (humanoid)
└─────────────────────────────────────┘  • Cost: exp(-1.0 * dist)
                ↓
         FINAL COSTMAP
                ↓
┌─────────────────────────────────────┐
│      USED BY PLANNERS                │
│  • Global Planner: A* pathfinding   │
│  • Local Planner: DWA trajectory    │
└─────────────────────────────────────┘

═══════════════════════════════════════════════════════════════════
INFLATION COST FUNCTION
═══════════════════════════════════════════════════════════════════

Cost(distance) = 253 * exp(-1.0 * cost_scaling_factor * (distance - inscribed_radius))

Where:
• inscribed_radius = robot_radius (footprint)
• distance = distance from obstacle center
• cost_scaling_factor = 5.0 (humanoid, vs 3.0 wheeled)

Example (humanoid: robot_radius=0.2m, inflation_radius=0.5m):
┌──────────────┬──────────┬────────────────────────────┐
│ Distance (m) │   Cost   │ Meaning                    │
├──────────────┼──────────┼────────────────────────────┤
│     0.00     │   254    │ Lethal (obstacle center)   │
│     0.20     │   253    │ Inscribed (footprint edge) │
│     0.30     │   171    │ High cost (close)          │
│     0.40     │   116    │ Medium cost                │
│     0.50     │    78    │ Low cost (inflation edge)  │
│     0.60+    │     0    │ Free space                 │
└──────────────┴──────────┴────────────────────────────┘

═══════════════════════════════════════════════════════════════════
DATA FLOW TIMELINE
═══════════════════════════════════════════════════════════════════

Time t=0:
  Static Layer:     Load from map file (once at startup)
  Obstacle Layer:   Initialize empty
  Voxel Layer:      Initialize empty
  Inflation Layer:  Inflate static obstacles

Time t=100ms (sensor update):
  Obstacle Layer:   Update with lidar scan (mark/clear cells)
  Voxel Layer:      Update with depth camera (3D occupancy)
  → Trigger costmap update

Time t=110ms (costmap update @ 10 Hz):
  Merge: Static + Obstacle + Voxel → Combined costmap
  Inflate: Apply inflation layer to combined costmap
  Publish: nav_msgs/OccupancyGrid to /local_costmap

Time t=120ms (planner uses costmap):
  Local Planner (DWA): Query costmap for trajectory scoring
  → Avoid high-cost cells (inflated obstacles)
  → Generate cmd_vel for smooth, collision-free motion
```

---

## Component Explanations

### 1. Static Layer
**Source**: Pre-built map from SLAM (OccupancyGrid)

**Purpose**: Represent permanent obstacles (walls, furniture, doorways)

**Update Frequency**: 1 Hz or on map change (rarely)

**Cost Assignment**:
- Free space: 0
- Occupied (wall): 254
- Unknown: 255 (if map has unexplored regions)

**Why It Matters**: Provides long-term structure for global planning. Without static layer, robot would "forget" walls not currently visible.

### 2. Obstacle Layer
**Source**: Real-time sensor data (lidar, depth cameras)

**Purpose**: Detect dynamic obstacles (people, moving robots, boxes)

**Update Frequency**: 5-10 Hz (synchronized with sensor rate)

**Marking & Clearing**:
- **Marking**: Set cells to 254 when lidar detects obstacle
- **Clearing**: Set cells to 0 when lidar sees through them
- **Raycasting**: Clear cells along ray from sensor to obstacle

**Why It Matters**: Enables reactive obstacle avoidance. Without obstacle layer, robot would collide with people/objects not in static map.

**Important Parameters**:
- `observation_sources`: Which sensors to use (e.g., `/scan`, `/camera/depth/points`)
- `max_obstacle_height`: Ignore obstacles above this height (e.g., 2.0m for humanoid head)
- `obstacle_range`: Maximum range to mark obstacles (e.g., 2.5m for local costmap)
- `raytrace_range`: Maximum range to clear free space (e.g., 3.0m)

### 3. Voxel Layer (Optional - 3D Aware)
**Source**: 3D sensors (depth cameras, stereo cameras, 3D lidar)

**Purpose**: Distinguish obstacles by height (important for legged robots)

**Representation**: 3D voxel grid (x, y, z), collapsed to 2D for planning

**Use Cases**:
- **Low obstacles** (0.1-0.3m): Steps, curbs, speed bumps → trip hazard for humanoids
- **High obstacles** (0.7-2.0m): Tables, desks, shelves → head collision risk
- **Pass-under obstacles**: Overhead pipes, archways → safe for wheeled robots, dangerous for tall humanoids

**Why Humanoids Need This**: Wheeled robots only care about obstacles at ground level. Humanoids need to avoid low steps (trip) and high tables (head collision).

**Configuration**:
```yaml
voxel_layer:
  max_obstacle_height: 2.0  # Detect up to 2m (humanoid head height)
  min_obstacle_height: 0.05 # Detect as low as 5cm (small step)
  z_resolution: 0.05        # 5cm vertical resolution
  z_voxels: 40              # 2.0m / 0.05m = 40 voxels
```

### 4. Inflation Layer
**Purpose**: Add safety margin around obstacles

**Why Needed**: Planners work with robot center point. Inflation ensures robot footprint doesn't collide with obstacles.

**Cost Function**:
```
Cost(d) = 253 * exp(-1.0 * k * (d - r_inscribed))
```
Where:
- `d` = distance from obstacle
- `r_inscribed` = robot radius (footprint)
- `k` = cost_scaling_factor (controls gradient steepness)

**Effect**:
- Obstacles "grow" by `inflation_radius`
- Cost gradient: 254 (obstacle) → 0 (free) over `inflation_radius` distance
- Planner prefers lower-cost paths (farther from obstacles)

**Humanoid Settings**:
- `inflation_radius`: 0.5m (vs 0.3m for wheeled) - wider margin for balance
- `cost_scaling_factor`: 5.0 (vs 3.0) - steeper gradient, more cautious

---

## Key Insights

### 1. Layer Fusion Strategy
**Why multiple layers?** Separation of concerns:
- Static Layer: Long-term structure (walls)
- Obstacle Layer: Short-term dynamics (people)
- Voxel Layer: Height awareness (steps, tables)
- Inflation Layer: Safety margins (collision prevention)

Each layer updated at different frequencies → efficient compute.

### 2. Marking vs Clearing in Obstacle Layer
**Problem**: Sensor noise can cause false obstacles (e.g., dust in lidar)

**Solution**:
- **Marking**: Require 2-3 consecutive detections before marking cell as obstacle
- **Clearing**: Clear cells aggressively (single raycast)
- **Result**: Transient noise cleared quickly, persistent obstacles remain

### 3. Inflation Prevents "Wall Hugging"
**Without inflation**: Planner finds shortest path, often touching walls (cost 0 vs 0.1m away)

**With inflation**: Planner avoids inflated areas (cost 50-200), prefers paths farther from walls

**Trade-off**: Inflation radius too large → robot can't fit through narrow doorways

### 4. Local vs Global Costmap Differences

| Property | Local Costmap | Global Costmap |
|----------|---------------|----------------|
| Size | 3x3m | Full map (e.g., 50x30m) |
| Rolling | Yes (follows robot) | No (static origin) |
| Resolution | 0.05m (fine) | 0.05m or coarser |
| Update Rate | 5-10 Hz | 1 Hz |
| Layers | Static + Obstacle + Voxel + Inflation | Static + Obstacle + Inflation |
| Purpose | Reactive control | Long-term planning |

**Why separate?** Local costmap expensive to compute → only update small window. Global costmap updated slowly → sufficient for coarse planning.

### 5. Humanoid-Specific Tuning
**Challenges**:
- Bipedal instability → wider inflation radius (0.5m vs 0.3m)
- Height variation → voxel layer to detect steps/tables
- Slower motion → can tolerate lower update rates (5 Hz vs 10 Hz for wheeled)

**Configuration Strategy**:
1. Start with wheeled robot defaults
2. Increase `inflation_radius` by 50-100%
3. Enable voxel layer if 3D sensor available
4. Reduce update rates by 50% to match slower motion

---

## Real-World Application

**Use Case**: Humanoid navigating office with people and furniture

**Static Layer**: Office map (walls, desks, chairs from SLAM)

**Obstacle Layer**: Lidar detects person walking → marks 0.5m × 0.5m region as obstacle

**Voxel Layer**: Depth camera detects table at 0.75m height → marks as high obstacle (avoid head collision)

**Inflation Layer**: Expands person outline by 0.5m → planner routes around with safety margin

**Result**:
- Robot plans path avoiding static furniture (static layer)
- Replans locally when person crosses path (obstacle layer)
- Slows down near tables to avoid head collision (voxel layer)
- Maintains 0.5m clearance from all obstacles (inflation layer)

**Performance**:
- Local costmap update: 10ms (5 Hz achievable)
- Memory: 3×3m @ 0.05m resolution = 3600 cells × 1 byte = 3.6 KB
- CPU: 5-10% on modern processor (Jetson AGX Orin)
