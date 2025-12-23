# Humanoid Stability & ZMP Specification

**Purpose**: Visualize Zero-Moment Point (ZMP), Center of Mass (CoM), and support polygon concepts essential for humanoid robot balance

**Diagram Type**: data_flow

## Humanoid Stability Overview

```text
                    HUMANOID STABILITY FUNDAMENTALS
                    ═══════════════════════════════

                         CENTER OF MASS (CoM)
                              ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
                              ┃                             ┃
                              ┃  The point where total      ┃
                              ┃  mass is concentrated       ┃
                              ┃                             ┃
                              ┃  Height: ~1m for humanoid   ┃
                              ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
                              │
                              │ Gravity acts here: F = mg
                              │
                              ▼
              ZERO-MOMENT POINT (ZMP)
              ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
              ┃                                           ┃
              ┃  Ground point where total moment = 0      ┃
              ┃  (sum of gravity + inertia forces)        ┃
              ┃                                           ┃
              ┃  If ZMP inside support polygon: STABLE    ┃
              ┃  If ZMP outside: FALLING                  ┃
              ┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛
                              │
                              ▼
         ┌────────────────────────────────────────┐
         │          SUPPORT POLYGON               │
         │                                        │
         │   Convex hull of contact points        │
         │   between feet and ground              │
         │                                        │
         └────────────────────────────────────────┘
```

## Support Polygon Configurations

```text
DOUBLE SUPPORT (Both feet on ground)
════════════════════════════════════

Top View:
┌───────────────────────────────────────────────────┐
│                                                   │
│     ┌───────┐               ┌───────┐            │
│     │ Left  │               │ Right │            │
│     │ Foot  │               │ Foot  │            │
│     └───┬───┘               └───┬───┘            │
│         │                       │                │
│         └───────────●───────────┘                │
│                    ZMP                           │
│         (large support polygon)                  │
│                                                  │
│  Support Polygon = Convex hull of both feet     │
│  ════════════════════════════════════════       │
│  ┌─────────────────────────────────────┐        │
│  │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│        │
│  │░░░░░░░░░░STABLE░REGION░░░░░░░░░░░░░│        │
│  │░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░│        │
│  └─────────────────────────────────────┘        │
│                                                  │
│  Area: ~0.15 m² (typical humanoid)              │
│  ZMP margin: ~10 cm to edge                     │
│                                                  │
└───────────────────────────────────────────────────┘


SINGLE SUPPORT (One foot lifted)
════════════════════════════════

Top View:
┌───────────────────────────────────────────────────┐
│                                                   │
│     ┌───────┐                                    │
│     │ Left  │           ○ Right foot            │
│     │ Foot  │             (lifted)              │
│     └───┬───┘                                    │
│         │                                        │
│         ●  ZMP must stay here!                  │
│                                                  │
│  Support Polygon = Single foot only             │
│  ═══════════════════════════════                │
│  ┌───────┐                                      │
│  │░░░░░░░│  ← Much smaller!                     │
│  │░░░░░░░│                                      │
│  └───────┘                                      │
│                                                  │
│  Area: ~0.02 m² (10× smaller than double)       │
│  ZMP margin: ~3 cm to edge (critical!)          │
│                                                  │
└───────────────────────────────────────────────────┘
```

## ZMP Stability Criterion

```text
                    STABILITY ANALYSIS
                    ══════════════════

SIDE VIEW:
                    CoM
                     ●─────────┐
                     │         │ Mass × Gravity
                     │         ▼
         ┌───────────┼───────────────────────┐
         │           │                       │
         │           │                       │  Robot Body
         │           │                       │
         └───────────┼───────────────────────┘
                     │
                     │
         ════════════▼════════════════════════
                    ZMP
                     ●

         ├─── Support Polygon (foot width) ───┤


STABLE (ZMP inside polygon):
━━━━━━━━━━━━━━━━━━━━━━━━━━━

         │←── foot ──→│
         ┌────────────┐
    ░░░░░│░░░░●░░░░░░░│░░░░░  ZMP inside = OK
         └────────────┘
              ZMP

    ΣMoments at ZMP = 0
    No rotational acceleration
    Robot stays upright


UNSTABLE (ZMP outside polygon):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

         │←── foot ──→│
         ┌────────────┐
    ░░░░░│░░░░░░░░░░░░│░●░░░  ZMP outside = FALLING!
         └────────────┘
                      ZMP

    ΣMoments at ZMP ≠ 0
    Net torque causes rotation
    Robot tips over
```

## CoM vs ZMP Relationship

```text
STATIC CASE (standing still):
════════════════════════════

    CoM directly above ZMP
    ZMP = projection of CoM onto ground plane

              CoM
               ●
               │
               │ Straight down
               │
    ═══════════▼═══════════
              ZMP
               ●


DYNAMIC CASE (walking/moving):
══════════════════════════════

    ZMP shifts due to acceleration
    ZMP ≠ CoM projection

              CoM (moving forward →)
               ●─────→ acceleration
              /│
             / │
            /  │
    ═══════▼═══▼═══════════
          ZMP  │
           ●   │
               │
    ZMP leads CoM when decelerating
    ZMP lags CoM when accelerating


ZMP FORMULA:
════════════

         Σ(mᵢ × (g + aᵢ) × pᵢ)
    ZMP = ─────────────────────
          Σ(mᵢ × (g + aᵢ))

    Where:
    • mᵢ = mass of link i
    • g  = gravity vector
    • aᵢ = acceleration of link i
    • pᵢ = position of link i CoM
```

## Stability Margins

```text
                ZMP STABILITY MARGIN
                ═══════════════════

Top View of Support Polygon:
┌─────────────────────────────────────────────────────┐
│                                                     │
│      ┌───────────────────────────────────┐         │
│      │                                   │         │
│      │     ┌───────────────────┐        │         │
│      │     │                   │        │         │
│      │     │     ┌───────┐    │        │         │
│      │     │     │       │    │        │         │
│      │     │     │   ●   │    │        │ ◄─ Safe zone
│      │     │     │  ZMP  │    │        │    (margin > 5cm)
│      │     │     └───────┘    │        │         │
│      │     │      5cm margin  │        │         │
│      │     └───────────────────┘        │         │
│      │          10cm margin             │         │
│      └───────────────────────────────────┘         │
│                 Polygon edge                        │
│                                                     │
└─────────────────────────────────────────────────────┘

MARGIN GUIDELINES:
══════════════════

    │ Margin  │ Status        │ Action Required          │
    │─────────│───────────────│──────────────────────────│
    │ > 10 cm │ SAFE          │ None                     │
    │ 5-10 cm │ CAUTION       │ Prepare recovery motion  │
    │ 2-5 cm  │ CRITICAL      │ Execute balance recovery │
    │ < 2 cm  │ FALLING       │ Emergency stop / catch   │
```

## Walking Gait ZMP Trajectory

```text
WALKING CYCLE - ZMP MOVEMENT
════════════════════════════

Phase 1: Double Support (both feet down)
─────────────────────────────────────────
    ┌─────┐         ┌─────┐
    │ L   │─────────│ R   │
    │     │    ●    │     │  ZMP in middle
    └─────┘   ZMP   └─────┘


Phase 2: Weight Transfer (shift to left foot)
─────────────────────────────────────────────
    ┌─────┐         ┌─────┐
    │ L   │←────────│ R   │
    │  ●  │         │     │  ZMP moves to L foot
    └─────┘   ZMP   └─────┘


Phase 3: Single Support (right foot lifted)
──────────────────────────────────────────
    ┌─────┐
    │ L   │         ○  R (swinging)
    │  ●  │
    └─────┘   ZMP   ZMP must stay in L foot polygon!


Phase 4: Heel Strike (right foot lands)
───────────────────────────────────────
    ┌─────┐         ┌─────┐
    │ L   │    ●────│ R   │
    │     │   ZMP   │     │  ZMP transfers to new polygon
    └─────┘         └─────┘


TOP VIEW - COMPLETE GAIT CYCLE:
═══════════════════════════════

    ┌─────┐         ┌─────┐         ┌─────┐
    │ L1  │         │ R1  │         │ L2  │
    │     │         │     │         │     │
    └──┬──┘         └──┬──┘         └──┬──┘
       │               │               │
       ●───────────────●───────────────●
      ZMP path        ZMP             ZMP
    (side-to-side movement during walking)
```

## Gazebo Implementation

```xml
<!-- Force/Torque Sensor at Ankle for ZMP Calculation -->
<sensor name="left_ankle_ft" type="force_torque">
  <always_on>true</always_on>
  <update_rate>1000</update_rate>
  <force_torque>
    <frame>child</frame>
    <measure_direction>child_to_parent</measure_direction>
  </force_torque>
</sensor>

<!-- Contact Sensor on Foot for Support Detection -->
<sensor name="left_foot_contact" type="contact">
  <always_on>true</always_on>
  <update_rate>1000</update_rate>
  <contact>
    <collision>left_foot_collision</collision>
  </contact>
</sensor>
```

```python
# ZMP Calculation from Force/Torque Sensors (pseudo-code)
def calculate_zmp(left_ft, right_ft, left_pos, right_pos):
    """
    Calculate ZMP from ankle force/torque readings.

    Args:
        left_ft: (fx, fy, fz, tx, ty, tz) from left ankle
        right_ft: (fx, fy, fz, tx, ty, tz) from right ankle
        left_pos: (x, y, z) position of left ankle
        right_pos: (x, y, z) position of right ankle

    Returns:
        (zmp_x, zmp_y) in world frame
    """
    # Total vertical force
    fz_total = left_ft.fz + right_ft.fz

    if fz_total < 10:  # Threshold for foot contact
        return None  # Both feet in air (falling!)

    # ZMP from moment balance
    # ZMP_x = -(ty_left + ty_right) / fz_total
    # ZMP_y = (tx_left + tx_right) / fz_total

    zmp_x = -(left_ft.ty + right_ft.ty +
              left_pos.x * left_ft.fz +
              right_pos.x * right_ft.fz) / fz_total

    zmp_y = (left_ft.tx + right_ft.tx +
             left_pos.y * left_ft.fz +
             right_pos.y * right_ft.fz) / fz_total

    return (zmp_x, zmp_y)
```

## Usage in Book

- **Referenced in**: Chapter 2 (Core Concept 3: Humanoid Stability Fundamentals)
- **Purpose**: Visualize abstract stability concepts that are critical for humanoid control
- **Learning Goal**: Understand why humanoids fall and how ZMP-based controllers maintain balance

## Key Takeaways

1. **CoM** is where mass is concentrated; **ZMP** is where ground reaction forces balance moments
2. **Support polygon** = convex hull of foot contacts; shrinks dramatically in single support
3. **Stability criterion**: ZMP inside support polygon = stable; outside = falling
4. **Static vs dynamic**: ZMP equals CoM projection only when stationary; acceleration shifts ZMP
5. **Walking**: ZMP oscillates side-to-side, transferring between feet during gait
6. **Safety margins**: Keep ZMP more than 5cm from polygon edge; less than 2cm triggers recovery
7. **Gazebo sensors**: Force/torque at ankles + contact sensors enable ZMP calculation
