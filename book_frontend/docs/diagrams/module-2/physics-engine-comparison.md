# Physics Engine Comparison Specification

**Purpose**: Compare DART, Bullet, and ODE physics engines to help learners choose the right engine for their simulation needs

**Diagram Type**: comparison

## Physics Engine Selection Matrix

| Feature | DART (Default) | Bullet | ODE |
|---------|----------------|--------|-----|
| **Primary Strength** | Articulated robots | Rigid body dynamics | Joint stability |
| **Constraint Solver** | Dantzig LCP | Sequential Impulse | Dantzig/PGS |
| **Default Timestep** | 1ms (1kHz) | 2ms (500Hz) | 5ms (200Hz) |
| **Contact Handling** | Accurate | Fast | Conservative |
| **Soft Bodies** | Limited | ✓ Full support | ✗ None |
| **GPU Acceleration** | ✗ None | Partial | ✗ None |
| **Joint Limit Accuracy** | ★★★★★ | ★★★☆☆ | ★★★★☆ |
| **High-DoF Robots** | ★★★★★ | ★★★☆☆ | ★★★★☆ |
| **Collision Speed** | ★★★☆☆ | ★★★★★ | ★★☆☆☆ |
| **Memory Efficiency** | ★★★☆☆ | ★★★★★ | ★★★★☆ |
| **Gazebo Integration** | Native (default) | Plugin | Legacy |

## Decision Flowchart

```text
                    START: Choosing a Physics Engine
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ Is your robot articulated     │
                    │ (multiple joints)?            │
                    └───────────────┬───────────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              │ YES                 │                     │ NO
              ▼                     │                     ▼
    ┌─────────────────┐             │         ┌─────────────────────┐
    │ DART            │             │         │ Simple rigid bodies?│
    │ (Default)       │             │         └──────────┬──────────┘
    │                 │             │                    │
    │ Best for:       │             │         ┌─────────┼─────────┐
    │ • Humanoids     │             │         │ YES     │         │ NO
    │ • Manipulators  │             │         ▼         │         ▼
    │ • Multi-joint   │             │    ┌────────┐     │    ┌────────┐
    └─────────────────┘             │    │ Bullet │     │    │ DART   │
                                    │    │        │     │    │        │
                                    │    │ Fast   │     │    │ General│
                                    │    │ scenes │     │    │ purpose│
                                    │    └────────┘     │    └────────┘
                                    │
                                    ▼
                    ┌───────────────────────────────┐
                    │ Need soft body simulation?    │
                    │ (cloth, deformable objects)   │
                    └───────────────┬───────────────┘
                                    │
              ┌─────────────────────┼─────────────────────┐
              │ YES                 │                     │ NO
              ▼                     │                     ▼
    ┌─────────────────┐             │         ┌─────────────────────┐
    │ Bullet          │             │         │ Stability issues    │
    │                 │             │         │ with existing sim?  │
    │ Soft body       │             │         └──────────┬──────────┘
    │ support         │             │                    │
    └─────────────────┘             │         ┌─────────┼─────────┐
                                    │         │ YES     │         │ NO
                                    │         ▼         │         ▼
                                    │    ┌────────┐     │    ┌────────┐
                                    │    │  ODE   │     │    │ DART   │
                                    │    │        │     │    │        │
                                    │    │ Stable │     │    │ Default│
                                    │    │ joints │     │    │ choice │
                                    │    └────────┘     │    └────────┘
```

## Detailed Engine Profiles

### DART (Dynamic Animation and Robotics Toolkit)

```text
┌─────────────────────────────────────────────────────────────────┐
│                          DART                                   │
│                   "The Roboticist's Choice"                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ARCHITECTURE:                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│  │ Skeleton     │───►│ BodyNode    │───►│ Joint        │     │
│  │ (robot tree) │    │ (link)      │    │ (constraint) │     │
│  └──────────────┘    └──────────────┘    └──────────────┘     │
│                                                                 │
│  KEY FEATURES:                                                  │
│  • Featherstone algorithm for articulated body dynamics         │
│  • Accurate joint limit enforcement (hard constraints)          │
│  • Native inverse dynamics support                              │
│  • Jacobian computation for control                             │
│                                                                 │
│  SOLVER OPTIONS:                                                │
│  • Dantzig: Fast, general purpose (default)                     │
│  • PGS: Better for high-friction scenarios                      │
│                                                                 │
│  RECOMMENDED SETTINGS:                                          │
│  ┌─────────────────────────────────────────────────┐           │
│  │ <physics type="dart">                           │           │
│  │   <max_step_size>0.001</max_step_size>          │           │
│  │   <dart>                                        │           │
│  │     <solver><solver_type>dantzig</solver_type>  │           │
│  │     </solver>                                   │           │
│  │   </dart>                                       │           │
│  │ </physics>                                      │           │
│  └─────────────────────────────────────────────────┘           │
│                                                                 │
│  BEST FOR:                                                      │
│  ✓ Humanoid robots (30+ DoF)                                   │
│  ✓ Industrial manipulators                                      │
│  ✓ Legged locomotion                                           │
│  ✓ Contact-rich manipulation                                    │
│                                                                 │
│  AVOID FOR:                                                     │
│  ✗ Large-scale scenes (1000+ objects)                          │
│  ✗ Soft body simulation                                        │
│  ✗ Real-time performance on low-end hardware                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Bullet Physics

```text
┌─────────────────────────────────────────────────────────────────┐
│                         BULLET                                  │
│                   "The Game Engine Favorite"                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ARCHITECTURE:                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│  │ btDynamics   │───►│ btRigidBody │───►│ btConstraint │     │
│  │ World        │    │ (collision) │    │ (joint)      │     │
│  └──────────────┘    └──────────────┘    └──────────────┘     │
│                                                                 │
│  KEY FEATURES:                                                  │
│  • Optimized broad-phase collision detection (DBVT)             │
│  • Soft body simulation (cloth, rope, deformable)               │
│  • GPU acceleration (partial, via OpenCL)                       │
│  • Excellent memory efficiency                                  │
│                                                                 │
│  SOLVER: Sequential Impulse (SI)                                │
│  • Fast convergence for simple contacts                         │
│  • Less accurate for complex joint chains                       │
│                                                                 │
│  RECOMMENDED SETTINGS:                                          │
│  ┌─────────────────────────────────────────────────┐           │
│  │ <physics type="bullet">                         │           │
│  │   <max_step_size>0.002</max_step_size>          │           │
│  │   <bullet>                                      │           │
│  │     <solver><iters>50</iters></solver>          │           │
│  │   </bullet>                                     │           │
│  │ </physics>                                      │           │
│  └─────────────────────────────────────────────────┘           │
│                                                                 │
│  BEST FOR:                                                      │
│  ✓ Large object counts (debris, particles)                     │
│  ✓ Soft body dynamics (cloth, ropes)                           │
│  ✓ Fast prototyping                                            │
│  ✓ Mobile robot navigation (simple dynamics)                   │
│                                                                 │
│  AVOID FOR:                                                     │
│  ✗ Precision joint control                                     │
│  ✗ Force-based manipulation                                    │
│  ✗ Humanoid balance control                                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### ODE (Open Dynamics Engine)

```text
┌─────────────────────────────────────────────────────────────────┐
│                           ODE                                   │
│                   "The Stable Veteran"                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ARCHITECTURE:                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│  │ dWorld       │───►│ dBody       │───►│ dJoint       │     │
│  │ (simulation) │    │ (rigid)     │    │ (constraint) │     │
│  └──────────────┘    └──────────────┘    └──────────────┘     │
│                                                                 │
│  KEY FEATURES:                                                  │
│  • Mature, well-tested codebase (20+ years)                     │
│  • Conservative contact handling (fewer penetrations)           │
│  • ERP/CFM parameters for joint compliance                      │
│  • Stable at larger timesteps (5ms)                             │
│                                                                 │
│  SOLVER OPTIONS:                                                │
│  • Dantzig: Default, general purpose                            │
│  • PGS (QuickStep): Faster, less accurate                       │
│                                                                 │
│  RECOMMENDED SETTINGS:                                          │
│  ┌─────────────────────────────────────────────────┐           │
│  │ <physics type="ode">                            │           │
│  │   <max_step_size>0.005</max_step_size>          │           │
│  │   <ode>                                         │           │
│  │     <solver><type>world</type></solver>         │           │
│  │     <constraints>                               │           │
│  │       <cfm>0.0001</cfm>                         │           │
│  │       <erp>0.2</erp>                            │           │
│  │     </constraints>                              │           │
│  │   </ode>                                        │           │
│  │ </physics>                                      │           │
│  └─────────────────────────────────────────────────┘           │
│                                                                 │
│  BEST FOR:                                                      │
│  ✓ Legacy simulations (Gazebo Classic compatibility)           │
│  ✓ When DART has stability issues                              │
│  ✓ Simple wheeled robots                                       │
│  ✓ Lower-frequency control loops (100-200 Hz)                  │
│                                                                 │
│  AVOID FOR:                                                     │
│  ✗ High-frequency control (1kHz)                               │
│  ✗ Complex articulated robots                                  │
│  ✗ New projects (prefer DART)                                  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Performance Comparison

```text
                    COMPUTATION TIME PER PHYSICS STEP
                    (Lower is better)

                Humanoid (30 DoF)    Mobile Robot (4 DoF)    Debris (100 objects)
                ─────────────────    ────────────────────    ────────────────────
DART            │████████░░│ 0.8ms   │██░░░░░░░░│ 0.2ms     │████████████│ 1.2ms
Bullet          │██████████│ 1.0ms   │█░░░░░░░░░│ 0.1ms     │████░░░░░░░░│ 0.4ms
ODE             │██████████│ 1.0ms   │███░░░░░░░│ 0.3ms     │██████████░░│ 1.0ms


                    JOINT ACCURACY (Position error after 10 seconds)
                    (Lower is better)

                Humanoid (30 DoF)    Manipulator (6 DoF)
                ─────────────────    ───────────────────
DART            │█░░░░░░░░░│ 0.1°    │█░░░░░░░░░│ 0.05°
Bullet          │█████░░░░░│ 0.5°    │████░░░░░░│ 0.4°
ODE             │██░░░░░░░░│ 0.2°    │██░░░░░░░░│ 0.2°


                    STABILITY AT DIFFERENT TIMESTEPS
                    (✓ = stable, ✗ = unstable)

Timestep        DART        Bullet      ODE
────────────────────────────────────────────
1ms (1kHz)      ✓           ✓           ✓
2ms (500Hz)     ✓           ✓           ✓
5ms (200Hz)     ⚠ (tuning)  ✓           ✓
10ms (100Hz)    ✗           ⚠           ✓
```

## Switching Engines in SDF

```xml
<!-- DART (Default - recommended for most robotics) -->
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <dart>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>

<!-- Bullet (For soft bodies or large object counts) -->
<physics name="bullet_physics" type="bullet">
  <max_step_size>0.002</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <bullet>
    <solver>
      <iters>50</iters>
    </solver>
  </bullet>
</physics>

<!-- ODE (For legacy compatibility or stability issues) -->
<physics name="ode_physics" type="ode">
  <max_step_size>0.005</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <ode>
    <solver>
      <type>world</type>
      <precon_iters>0</precon_iters>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0001</cfm>
      <erp>0.2</erp>
    </constraints>
  </ode>
</physics>
```

## Usage in Book

- **Referenced in**: Chapter 2 (Core Concept 2: Physics Engine Configuration)
- **Purpose**: Provide decision criteria for engine selection before learners start building simulations
- **Learning Goal**: Understand trade-offs so learners can troubleshoot instability by switching engines or adjusting parameters

## Key Takeaways

1. **DART is the default**: Use it for articulated robots (humanoids, manipulators) with 1ms timestep
2. **Bullet for speed**: Better for large object counts, soft bodies, and simple dynamics
3. **ODE for stability**: Legacy option when DART has issues; works at larger timesteps
4. **Timestep matters**: Smaller = more accurate but slower; match to your control loop frequency
5. **Solver tuning**: Dantzig (fast), PGS (stable high-friction), iteration count affects accuracy
6. **Collision detector**: DART can use Bullet's collision (fast) while keeping DART's dynamics (accurate)
