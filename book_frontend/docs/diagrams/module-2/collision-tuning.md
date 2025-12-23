# Collision Geometry Tuning Specification

**Purpose**: Guide learners through debugging and optimizing collision geometry for stable robot simulation

**Diagram Type**: comparison

## Visual vs Collision Geometry

```text
┌─────────────────────────────────────────────────────────────────┐
│              VISUAL vs COLLISION GEOMETRY                       │
└─────────────────────────────────────────────────────────────────┘

VISUAL GEOMETRY                    COLLISION GEOMETRY
(What you see)                     (What physics uses)
───────────────                    ──────────────────

┌─────────────────┐                ┌─────────────────┐
│                 │                │                 │
│    Complex      │                │    Simple       │
│    Mesh         │                │    Primitives   │
│    (10K faces)  │                │    (~10 faces)  │
│                 │                │                 │
│    ╭─────╮      │                │    ┌─────┐      │
│   ╭│     │╮     │                │    │     │      │
│  ╭─│     │─╮    │       →        │    │     │      │
│  │ │     │ │    │                │    │     │      │
│  │ ╰─────╯ │    │                │    └─────┘      │
│  ╰─────────╯    │                │                 │
│                 │                │                 │
└─────────────────┘                └─────────────────┘

Purpose: Rendering             Purpose: Physics computation
GPU: Handles complexity        CPU: Needs simplicity
Rate: 60 Hz                    Rate: 1000 Hz

BEST PRACTICE:
─────────────
• Visual: Use detailed CAD mesh (aesthetics)
• Collision: Use simplified primitives (performance)
• Match overall dimensions; ignore fine details
```

## Collision Primitive Selection

```text
PRIMITIVE SELECTION GUIDE
═════════════════════════

┌─────────────────┬─────────────────┬─────────────────┐
│      BOX        │    CYLINDER     │     SPHERE      │
├─────────────────┼─────────────────┼─────────────────┤
│   ┌───────┐     │      ┌─┐        │       ╭─╮       │
│   │       │     │     ╱   ╲       │      ╱   ╲      │
│   │       │     │    │     │      │     │     │     │
│   └───────┘     │    │     │      │      ╲   ╱      │
│                 │     ╲   ╱       │       ╰─╯       │
│                 │      └─┘        │                 │
├─────────────────┼─────────────────┼─────────────────┤
│ Best for:       │ Best for:       │ Best for:       │
│ • Torso         │ • Legs/Arms     │ • Wheels        │
│ • Feet          │ • Fingers       │ • Joints        │
│ • Rectangular   │ • Cylindrical   │ • Ball bearings │
│   components    │   components    │ • Rolling parts │
├─────────────────┼─────────────────┼─────────────────┤
│ Collision cost: │ Collision cost: │ Collision cost: │
│ LOW (8 vertices)│ MEDIUM (caps +  │ LOWEST (radius  │
│                 │ side checks)    │ only)           │
└─────────────────┴─────────────────┴─────────────────┘

When to use MESH collision:
───────────────────────────
• Complex concave shapes (gripper fingers with grooves)
• When primitives can't approximate well
• Trade-off: 10-100× slower collision detection
• Use convex decomposition to optimize

<collision>
  <geometry>
    <mesh>
      <!-- Use simplified mesh, not visual mesh -->
      <uri>model://robot/meshes/gripper_collision.stl</uri>
    </mesh>
  </geometry>
</collision>
```

## Common Collision Problems

### Problem 1: Adjacent Links Overlap

```text
SYMPTOM: Robot jitters or explodes on spawn
─────────────────────────────────────────

Before (BROKEN):                    After (FIXED):
───────────────                     ──────────────

    Link A                              Link A
    ┌─────────┐                         ┌─────────┐
    │█████████│                         │         │
    │█████████│←── Collision            │         │
    │████┬████│    overlap!             │    ┬    │
    └────┼────┘                         └────┼────┘
         │                                   │ 2mm gap
    ┌────┼────┐                         ┌────┼────┐
    │████┴████│                         │    ┴    │
    │█████████│                         │         │
    │█████████│                         │         │
    └─────────┘                         └─────────┘
    Link B                              Link B


SOLUTIONS:
──────────
1. Reduce collision size (keep visual same):
   <collision>
     <geometry>
       <cylinder radius="0.048" length="0.29"/>  <!-- 2mm smaller -->
     </geometry>
   </collision>

2. Offset collision geometry:
   <collision>
     <pose>0 0 0.002 0 0 0</pose>  <!-- 2mm Z offset -->
     <geometry>...</geometry>
   </collision>

3. Disable self-collision (last resort):
   <model>
     <self_collide>false</self_collide>
   </model>
```

### Problem 2: Foot Penetrates Ground

```text
SYMPTOM: Robot sinks into floor or bounces
──────────────────────────────────────────

Before (BROKEN):                    After (FIXED):
───────────────                     ──────────────

Ground ════════════════             Ground ════════════════
         │                                    │
      ┌──┴──┐                              ┌──┴──┐
      │█████│←── Foot partially            │     │
      │█████│    through ground            │     │←── Foot rests
      └─────┘                              └─────┘    on surface


SOLUTIONS:
──────────
1. Adjust initial spawn height:
   <pose>0 0 0.05 0 0 0</pose>  <!-- Start 5cm above ground -->

2. Increase contact stiffness:
   <surface>
     <contact>
       <ode>
         <kp>1e7</kp>  <!-- Stiffer contact (was 1e6) -->
         <kd>100</kd>
         <min_depth>0.001</min_depth>  <!-- Penetration tolerance -->
       </ode>
     </contact>
   </surface>

3. Ensure flat foot collision:
   <collision>
     <geometry>
       <!-- Box, not rounded mesh -->
       <box><size>0.2 0.1 0.03</size></box>
     </geometry>
   </collision>
```

### Problem 3: Joints Hit Limits Hard

```text
SYMPTOM: Joint snaps or vibrates at limit
─────────────────────────────────────────

Before (BROKEN):                    After (FIXED):
───────────────                     ──────────────

Position                            Position
    │                                   │
 +90°├─────────╮                    +88°├───────╮
    │         │                        │       │soft
    │         │hard                    │       │stop
    │         │stop                    │       │
  0°├─────────┼───► time            0°├───────┼───► time
    │                                   │


SOLUTIONS:
──────────
1. Add joint damping (absorbs energy at limits):
   <axis>
     <dynamics>
       <damping>10.0</damping>  <!-- N·m·s/rad -->
       <friction>1.0</friction>
     </dynamics>
   </axis>

2. Software limits inside hardware limits:
   <!-- Hardware limit: ±90° -->
   <!-- Software limit: ±88° (2° buffer) -->
   controller_config.yaml:
     joint_limits:
       shoulder:
         min: -1.536  # -88°
         max: 1.536   # +88°

3. Use effort limits to prevent slamming:
   <axis>
     <limit>
       <lower>-1.57</lower>
       <upper>1.57</upper>
       <effort>50</effort>   <!-- Cap torque -->
       <velocity>2.0</velocity>  <!-- Cap speed -->
     </limit>
   </axis>
```

### Problem 4: Gripper Slips Objects

```text
SYMPTOM: Gripper can't hold objects
───────────────────────────────────

Before (BROKEN):                    After (FIXED):
───────────────                     ──────────────

  Gripper                             Gripper
  ┌─────┐                             ┌─────┐
  │     │                             │▓▓▓▓▓│←── High friction
  │  ○  │←── Object slips             │  ●  │    surface
  │     │                             │▓▓▓▓▓│
  └─────┘                             └─────┘


SOLUTIONS:
──────────
1. Increase friction on gripper pads:
   <collision name="gripper_pad">
     <surface>
       <friction>
         <ode>
           <mu>2.0</mu>   <!-- Very high friction -->
           <mu2>2.0</mu2>
         </ode>
         <torsional>
           <coefficient>1.0</coefficient>
         </torsional>
       </friction>
     </surface>
   </collision>

2. Add soft contact (compliance):
   <surface>
     <contact>
       <ode>
         <soft_cfm>0.001</soft_cfm>  <!-- Soft constraint -->
         <soft_erp>0.9</soft_erp>
       </ode>
     </contact>
   </surface>

3. Use grasp plugin (force closure detection):
   <plugin filename="gz-sim-grasp-gripper-system"
           name="gz::sim::systems::GraspGripper">
     <gripper_link>left_finger</gripper_link>
     <gripper_link>right_finger</gripper_link>
     <attach_distance>0.01</attach_distance>
   </plugin>
```

## Collision Debugging Workflow

```text
COLLISION DEBUGGING CHECKLIST
═════════════════════════════

Step 1: Enable Collision Visualization
───────────────────────────────────────
• Gazebo GUI: View → Collisions (toggle on)
• Or via command:
  gz service -s /gui/view/collisions --reqtype gz.msgs.Boolean \
    --reptype gz.msgs.Boolean --req 'data: true'

Step 2: Slow Down Physics
─────────────────────────
• Set real_time_factor to 0.1 (10× slower)
• Watch collision geometry during motion

Step 3: Check Console for Warnings
──────────────────────────────────
• "Collision detected between..." → Overlap issue
• "Maximum corrective velocity exceeded" → Contact too stiff
• "Joint position limit reached" → Needs damping

Step 4: Isolate Problem Link
────────────────────────────
• Disable collision on links one at a time:
  <collision name="test">
    <pose>0 0 -1000 0 0 0</pose>  <!-- Move collision away -->
    ...
  </collision>
• When problem disappears, you found the culprit

Step 5: Tune Parameters Iteratively
───────────────────────────────────
• Start with default values
• Change ONE parameter at a time
• Test for 10+ seconds of simulation
• Document working values

COMMON PARAMETER RANGES:
────────────────────────
Parameter          │ Range          │ Default  │ Notes
───────────────────┼────────────────┼──────────┼──────────────
mu (friction)      │ 0.1 - 2.0      │ 1.0      │ Higher = grip
kp (stiffness)     │ 1e4 - 1e8      │ 1e6      │ Higher = harder
kd (damping)       │ 1 - 1000       │ 100      │ Higher = less bounce
damping (joint)    │ 0.1 - 50       │ 0        │ Higher = slower
min_depth          │ 0.0001 - 0.01  │ 0.001    │ Penetration before push
```

## Collision Geometry Best Practices

```text
DESIGN GUIDELINES
═════════════════

1. MATCH OVERALL SHAPE, NOT DETAILS
───────────────────────────────────
   Visual (complex)        Collision (simple)
   ┌──────────────┐        ┌──────────────┐
   │ ╭────────╮   │        │              │
   │╭╯        ╰╮  │   →    │              │
   │╰╮        ╭╯  │        │              │
   │ ╰────────╯   │        │              │
   └──────────────┘        └──────────────┘
   (curved handle)          (bounding box)


2. CONVEX DECOMPOSITION FOR CONCAVE SHAPES
──────────────────────────────────────────
   Original concave        Decomposed convex
   ┌────┐                  ┌────┐ ┌────┐
   │    └───┐              │    │ │    │
   │        │      →       │    │ │    │
   │    ┌───┘              │    │ │    │
   └────┘                  └────┘ └────┘
   (L-shape, 1 mesh)       (2 boxes, faster)


3. COLLISION MARGIN HIERARCHY
─────────────────────────────
   Large margin (torso)    Small margin (fingers)
   ┌───────────────┐       ┌─────────┐
   │               │       │┌───────┐│
   │   ┌───────┐   │       ││       ││
   │   │ visual│   │       ││visual ││
   │   │       │   │       ││       ││
   │   └───────┘   │       │└───────┘│
   │               │       └─────────┘
   └───────────────┘
   Margin: 5cm              Margin: 1mm


4. FOOT CONTACT SURFACE
───────────────────────
   Bad (single point)      Good (flat surface)
        ╱╲                  ┌──────────┐
       ╱  ╲                 │          │
      ╱    ╲                │          │
   ══╱══════╲══          ══════════════
   (unstable)             (stable, large
                           support polygon)
```

## Usage in Book

- **Referenced in**: Chapter 3 (Core Concept 3: Joint and Collision Tuning)
- **Purpose**: Provide practical debugging guidance for the most common simulation issues
- **Learning Goal**: Diagnose and fix collision problems independently without trial-and-error

## Key Takeaways

1. **Visual ≠ Collision**: Use detailed meshes for rendering, simple primitives for physics
2. **Adjacent links need gaps**: 1-2mm clearance prevents self-collision jitter
3. **Friction enables grip**: μ ≥ 0.8 for robot feet, μ ≥ 1.5 for gripper pads
4. **Damping prevents oscillation**: Add joint damping to stabilize near limits
5. **Debug visually first**: Enable collision view in Gazebo before tuning parameters
6. **One parameter at a time**: Systematic tuning beats random changes
