# Humanoid Footstep Planning

**Purpose**: Illustrate the footstep planning workflow for bipedal humanoid robots, showing how Zero Moment Point (ZMP) stability constraints, collision checking, and balance considerations differ from wheeled robot path planning.

**Context**: Unlike wheeled robots that can turn in place, humanoids must plan discrete footstep sequences while maintaining dynamic balance. Footstep planning is the bridge between high-level Nav2 paths and low-level motor control.

---

## ASCII Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│              HUMANOID FOOTSTEP PLANNING WORKFLOW                 │
└─────────────────────────────────────────────────────────────────┘

INPUT: Nav2 Global Path
━━━━━━━━━━━━━━━━━━━━━━━
┌──────────────────────────────────┐
│  Nav2 Waypoint Path              │
│  [(x₀, y₀, θ₀) → ... → (xₙ, yₙ, θₙ)]
│                                  │
│  Wheeled robot: continuous curve │
│  Humanoid: must discretize       │
└────────┬─────────────────────────┘
         │
         v

STAGE 1: FOOTSTEP DISCRETIZATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│              Path → Footstep Sequence                            │
├─────────────────────────────────────────────────────────────────┤
│  Input: Waypoint path (x, y, θ)                                 │
│  Output: Sequence of foot placements (L/R alternating)          │
│                                                                  │
│  Algorithm:                                                      │
│  1. Sample path at step_length intervals (0.2-0.4m)             │
│  2. For each sample:                                             │
│     - Place foot perpendicular to path direction                │
│     - Alternate left/right feet                                 │
│     - Maintain foot_width separation (0.2m)                     │
│  3. Adjust for turns (smaller steps, pivot foot)                │
│                                                                  │
│  Example (straight walk):                                        │
│    Start: (0, 0, 0°)                                             │
│    Step 1: L foot (0.3, 0.1, 0°)  ←Left                         │
│    Step 2: R foot (0.6, -0.1, 0°) ←Right                        │
│    Step 3: L foot (0.9, 0.1, 0°)  ←Left                         │
│    ...                                                           │
└────────┬────────────────────────────────────────────────────────┘
         │
         v

STAGE 2: ZMP STABILITY CHECK
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│         Zero Moment Point (ZMP) Constraint Verification          │
├─────────────────────────────────────────────────────────────────┤
│  ZMP Definition: Point where net moment from gravity & inertia   │
│  is zero. For stable walking, ZMP must stay inside support      │
│  polygon (convex hull of foot contact points).                  │
│                                                                  │
│  Single Support Phase (one foot on ground):                     │
│  ┌─────────────────────────┐                                    │
│  │    Body CoM (Center of Mass)                                 │
│  │      •  (x_com, y_com)                                       │
│  │      │                                                        │
│  │      │ Projection                                            │
│  │      v                                                        │
│  │  ┌───────┐                                                   │
│  │  │ Left  │  ← Support polygon (must contain ZMP)             │
│  │  │ Foot  │                                                   │
│  │  └───────┘                                                   │
│  │    ZMP here ^  ✓ Stable (inside foot)                        │
│  └─────────────────────────┘                                    │
│                                                                  │
│  Double Support Phase (both feet on ground):                    │
│  ┌─────────────────────────┐                                    │
│  │    Body CoM                                                  │
│  │      •                                                        │
│  │      │                                                        │
│  │      v                                                        │
│  │  ┌───────┐     ┌───────┐                                    │
│  │  │ Left  │     │ Right │  ← Support polygon (larger)        │
│  │  │ Foot  │     │ Foot  │                                    │
│  │  └───────┘     └───────┘                                    │
│  │       ZMP here ^  ✓ More stable (between feet)               │
│  └─────────────────────────┘                                    │
│                                                                  │
│  Stability Criteria:                                             │
│    distance(ZMP, support_polygon_edge) > safety_margin          │
│    safety_margin = 0.02m typical (2cm from edge)                │
│                                                                  │
│  If unstable → Reject footstep, generate alternative            │
└────────┬────────────────────────────────────────────────────────┘
         │
         v

STAGE 3: COLLISION CHECKING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│            Foot Placement Collision Avoidance                    │
├─────────────────────────────────────────────────────────────────┤
│  Check each footstep against:                                    │
│                                                                  │
│  1. Static Obstacles (from costmap):                             │
│     - Query costmap at foot position                             │
│     - Reject if cost > threshold (e.g., cost > 50)               │
│     - Foot footprint: 0.1m × 0.25m rectangle                     │
│                                                                  │
│  2. Terrain Constraints:                                         │
│     - Slope: max 15° for stable footing                          │
│     - Step height: max 0.15m (typical stair height)              │
│     - Surface type: avoid slippery surfaces (if detected)        │
│                                                                  │
│  3. Inter-foot Constraints:                                      │
│     - Minimum separation: 0.15m (avoid self-collision)           │
│     - Maximum separation: 0.6m (avoid overstretching)            │
│     - Kinematic reachability: hip/knee/ankle joint limits        │
│                                                                  │
│  Collision Detected → Try Alternative Footsteps:                │
│    - Shift foot laterally (±0.05m)                               │
│    - Reduce step length (0.3m → 0.2m)                            │
│    - Insert intermediate step (split long step)                  │
│    - If all fail → Request global replan from Nav2               │
└────────┬────────────────────────────────────────────────────────┘
         │
         v

STAGE 4: BALANCE CONSTRAINTS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│          Dynamic Balance During Turns & Stops                    │
├─────────────────────────────────────────────────────────────────┤
│  Turning Constraints:                                            │
│    - Max turn angle per step: 30° (vs instant for wheeled)      │
│    - For 90° turn: Requires 3 steps minimum                      │
│      Step 1: Pivot left foot, turn 30°                           │
│      Step 2: Step right foot, turn 30°                           │
│      Step 3: Step left foot, turn 30° → Total 90°                │
│                                                                  │
│  Stopping Constraints:                                           │
│    - Cannot stop mid-step (must complete current step)           │
│    - Must end in double-support stance (both feet down)          │
│    - Deceleration: 2-3 steps required from max velocity          │
│                                                                  │
│  Starting Constraints:                                           │
│    - Initial stance: double-support (stable)                     │
│    - Acceleration: 2-3 steps to reach max velocity               │
│    - Weight shift: 0.5s per foot (can't start instantly)         │
│                                                                  │
│  Balance During Disturbances:                                    │
│    - Unexpected push: May need extra recovery step               │
│    - Slope transition: Adjust foot angle for ground contact      │
│    - Slippery surface: Reduce step length, increase double-      │
│      support time                                                │
└────────┬────────────────────────────────────────────────────────┘
         │
         v

STAGE 5: TRAJECTORY GENERATION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
┌─────────────────────────────────────────────────────────────────┐
│        Foot Swing Trajectory (3D spline)                         │
├─────────────────────────────────────────────────────────────────┤
│  For each footstep (current → next):                             │
│                                                                  │
│  1. Lift Phase (0.2s):                                           │
│     - Lift foot 0.05m above ground                               │
│     - Accelerate vertically                                      │
│     - Maintain balance on support foot                           │
│                                                                  │
│  2. Swing Phase (0.4s):                                          │
│     - Move foot horizontally toward target                       │
│     - Peak height: 0.08m (clearance over obstacles)              │
│     - Cubic spline trajectory for smooth motion                  │
│                                                                  │
│  3. Landing Phase (0.2s):                                        │
│     - Lower foot to ground                                       │
│     - Decelerate vertically                                      │
│     - Transition to double-support                               │
│                                                                  │
│  Total step time: 0.8s (1.25 steps/sec)                         │
│  Compare to wheeled: continuous motion at 0.8 m/s                │
│                                                                  │
│  Trajectory Smoothness:                                          │
│    - Jerk-limited (avoid sudden accelerations)                   │
│    - Minimize energy (efficient gait)                            │
│    - Avoid singularities (knee lock, ankle limits)               │
└────────┬────────────────────────────────────────────────────────┘
         │
         v

OUTPUT: Motor Commands
━━━━━━━━━━━━━━━━━━━━━━
┌──────────────────────────────────┐
│  Joint Trajectory (Inverse Kinematics)
│  • Hip angles (θ_hip_L, θ_hip_R)  │
│  • Knee angles (θ_knee_L, θ_knee_R)
│  • Ankle angles (θ_ankle_L, θ_ankle_R)
│  Published at 200 Hz               │
└────────┬─────────────────────────┘
         │
         v
┌──────────────────────────────────┐
│  Robot Executes Footsteps         │
│  • Low-level motor control        │
│  • Torque control for balance     │
│  • IMU feedback for stability     │
└──────────────────────────────────┘

═══════════════════════════════════════════════════════════════════
ZMP STABILITY MARGIN CALCULATION
═══════════════════════════════════════════════════════════════════

Support Polygon (single foot):
  Foot dimensions: 0.25m (length) × 0.1m (width)
  Support polygon vertices: [(0.125, 0.05), (0.125, -0.05),
                              (-0.125, -0.05), (-0.125, 0.05)]

ZMP position (from CoM projection):
  ZMP_x = x_CoM - (h_CoM / g) * ẍ_CoM
  ZMP_y = y_CoM - (h_CoM / g) * ÿ_CoM

  Where:
    x_CoM, y_CoM = Center of Mass position
    h_CoM = Height of CoM above ground (typically 0.8-1.0m)
    ẍ_CoM, ÿ_CoM = CoM acceleration
    g = 9.81 m/s² (gravity)

Stability Margin:
  margin = min_distance(ZMP, polygon_edges)

  Safe: margin > 0.02m (2cm from edge)
  Risky: 0.01m < margin < 0.02m (warning, slow down)
  Unstable: margin < 0.01m (fall risk, emergency stop)

═══════════════════════════════════════════════════════════════════
HUMANOID VS WHEELED NAVIGATION COMPARISON
═══════════════════════════════════════════════════════════════════

┌──────────────────────┬────────────────┬─────────────────┐
│ Property             │ Wheeled Robot  │ Humanoid Robot  │
├──────────────────────┼────────────────┼─────────────────┤
│ Max Velocity         │ 1.0-2.0 m/s    │ 0.4-0.8 m/s     │
│ Max Acceleration     │ 1.5 m/s²       │ 0.3 m/s²        │
│ Turn Radius          │ 0 (in-place)   │ 0.5-1.0m        │
│ Turn Rate            │ 2.0 rad/s      │ 0.5 rad/s       │
│ Step Time            │ N/A            │ 0.8s per step   │
│ Stopping Distance    │ 0.3m           │ 1.0m (2-3 steps)│
│ Footprint            │ 0.4×0.4m       │ 0.3×0.3m        │
│ Inflation Radius     │ 0.3m           │ 0.5m            │
│ Planning Frequency   │ 10-20 Hz       │ 5-10 Hz         │
│ Replanning Latency   │ 50ms           │ 200ms (footstep)│
│ Recovery Behaviors   │ Rotate, backup │ Side-step, squat│
└──────────────────────┴────────────────┴─────────────────┘
```

---

## Component Explanations

### 1. Footstep Discretization
**Challenge**: Nav2 provides continuous waypoint paths suitable for wheeled robots. Humanoids require discrete foot placements.

**Discretization Strategy**:
1. Sample path at `step_length` intervals (0.2-0.4m based on robot size)
2. Place feet perpendicular to path tangent at each sample
3. Alternate left/right feet with `foot_width` separation (0.15-0.2m)
4. Adjust for sharp turns: reduce step length, increase turn steps

**Example**: 90° turn
- Wheeled robot: Rotate in place (0.5s)
- Humanoid: 3 footsteps × 0.8s = 2.4s (5x slower)

### 2. ZMP Stability
**Zero Moment Point (ZMP)**: The point on the ground where the net moment from gravity and inertial forces is zero.

**Stability Condition**: ZMP must remain inside the **support polygon** (convex hull of foot contact points).

**Single vs Double Support**:
- **Single Support**: Support polygon = single foot area (~0.025 m²)
- **Double Support**: Support polygon = both feet + area between (~0.15 m²)
- Double support is 6x more stable

**Safety Margin**: Keep ZMP at least 2cm from polygon edge to tolerate disturbances (pushes, uneven terrain).

**Why It Matters**: Violating ZMP constraint → robot falls. Unlike wheeled robots (statically stable), humanoids are **dynamically stable** only during controlled motion.

### 3. Collision Checking
**Foot-Level Collision**: Query costmap at each footstep location
- Foot footprint: 0.1m × 0.25m rectangle
- Reject if costmap cost > threshold (e.g., 50)
- Alternative: shift foot ±0.05m laterally or reduce step length

**Terrain Constraints**:
- **Slope**: Max 15° for stable footing (steeper → slip risk)
- **Step Height**: Max 0.15m (standard stair height)
- **Surface Type**: Avoid slippery surfaces if detected (requires tactile sensing or vision)

**Inter-Foot Constraints**:
- Minimum separation: 0.15m (avoid leg collision)
- Maximum separation: 0.6m (kinematic limit, avoid overstretching)
- Reachability: Check inverse kinematics (IK) solvability for hip/knee/ankle

### 4. Balance During Turns & Stops
**Turning**:
- Max turn angle per step: 30° (limited by hip rotation + balance)
- For 90° turn: 3 steps minimum (30° + 30° + 30°)
- Turning radius: 0.5-1.0m (vs 0m for wheeled robots)

**Stopping**:
- Cannot stop mid-step (momentum must transfer to stance foot)
- Must end in double-support stance (both feet down for stability)
- Deceleration: 2-3 steps required from max velocity (0.4 m/s → 0)

**Starting**:
- Initial stance: double-support (stable initialization)
- Weight shift: 0.5s to transfer weight from one foot to other
- Acceleration: 2-3 steps to reach max velocity

**Recovery from Disturbances**:
- Unexpected push: Insert extra side-step or backward step
- Slope transition: Adjust foot angle for full ground contact
- Slippery surface: Reduce step length 50%, increase double-support time

### 5. Trajectory Generation
**Foot Swing Phases**:
1. **Lift** (0.2s): Lift foot 0.05m, shift weight to support foot
2. **Swing** (0.4s): Move foot horizontally, peak height 0.08m (obstacle clearance)
3. **Landing** (0.2s): Lower foot to ground, transition to double-support

**Total Step Time**: 0.8s → 1.25 steps/sec → max forward velocity 0.4 m/s (for 0.3m step length)

**Trajectory Smoothness**:
- **Jerk-limited**: Smooth acceleration profiles (avoid sudden jerks → loss of balance)
- **Energy-efficient**: Minimize joint torques (pendulum-like swing)
- **Singularity avoidance**: Avoid knee lock (0° or 180°), ankle limits

**Comparison to Wheeled**:
- Wheeled: Continuous motion at 0.8 m/s (2x faster)
- Humanoid: Discrete steps at 0.4 m/s (50% slower but more agile on stairs/uneven terrain)

---

## Key Insights

### 1. Why Footstep Planning is Hard
**Curse of Dimensionality**:
- Wheeled robot: 3 DOF (x, y, θ) → search space = R²×S¹
- Humanoid: 12+ DOF (6 per leg) → search space = R¹²
- Result: Footstep planning is NP-hard, requires heuristics

**Dynamic Constraints**:
- Must maintain ZMP stability at all times
- Cannot instantaneously change velocity/direction (inertia)
- Recovery from disturbances requires replanning entire sequence

### 2. Trade-off: Speed vs Stability
**Fast Walking** (0.6-0.8 m/s):
- Shorter double-support time → less stable
- Larger ZMP deviations → closer to polygon edge
- Higher risk of falling if disturbed

**Slow Walking** (0.2-0.4 m/s):
- Longer double-support time → more stable
- Smaller ZMP deviations → larger safety margins
- Lower risk but slower task completion

**Practical Choice**: Indoor humanoids typically walk at 0.4 m/s (balance of speed/safety)

### 3. Humanoid-Specific Recovery Behaviors
Unlike wheeled robots (rotate, backup), humanoids use:
- **Side-step**: Move laterally to avoid obstacle
- **Squat**: Lower center of mass to increase stability
- **Arm swing**: Use arms for balance (humans do this naturally)
- **Step-in-place**: Mark time to reorient without net motion

### 4. When Footstep Planning Fails
**Replanning Triggers**:
- ZMP constraint violated (instability detected)
- Footstep collision with obstacle
- Path blocked (dynamic obstacle)
- Kinematic infeasibility (IK solution doesn't exist)

**Fallback Strategy**:
1. Try alternative footsteps (shift laterally, reduce step length)
2. Request global replan from Nav2 (find new path)
3. Execute recovery behavior (side-step, backup)
4. If all fail: Stop safely in double-support stance, request human assistance

### 5. Integration with Nav2
**Nav2 Provides**: Waypoint path (x, y, θ) at 1 Hz

**Footstep Planner Provides**: Foot placements (x_L, y_L, θ_L, x_R, y_R, θ_R) at 5-10 Hz

**Low-Level Controller Executes**: Joint trajectories (θ_hip, θ_knee, θ_ankle) at 200 Hz

**Three-Layer Hierarchy**:
- Global: Nav2 (1 Hz, long-term planning)
- Local: Footstep Planner (5-10 Hz, discrete steps)
- Reactive: Motor Controller (200 Hz, continuous torque)

---

## Real-World Application

**Use Case**: Humanoid robot navigating crowded office

**Scenario**: Robot receives goal "Go to desk 15m away" from Nav2

**Phase 1: Initial Footstep Plan** (1s)
- Nav2 provides 20 waypoints along 15m path
- Footstep planner discretizes into 50 footsteps (0.3m each)
- ZMP check: All footsteps stable (pass)
- Collision check: Path clear (pass)

**Phase 2: Execution** (40s)
- Execute footsteps 1-20 (16s, 8m progress)
- Person crosses path at step 21

**Phase 3: Dynamic Replanning** (2s)
- Footstep planner detects collision at step 21-23
- Try alternatives: Shift laterally +0.1m → Still collides
- Request Nav2 replan: New path around person
- Generate new footsteps 21-50

**Phase 4: Continue** (24s)
- Complete remaining 30 footsteps
- Arrive at goal in double-support stance

**Total Time**: 67s for 15m (0.22 m/s average, including replan overhead)

**Comparison**:
- Wheeled robot: 15s for 15m @ 1.0 m/s (4.5x faster)
- Human walker: 12s for 15m @ 1.25 m/s (5.6x faster)
- Trade-off: Humanoid can climb stairs, navigate uneven terrain (wheeled cannot)
