# Unity URDF Import Workflow Specification

**Purpose**: Document the process of importing URDF robot models into Unity and configuring them for simulation

**Diagram Type**: workflow

## URDF Import Pipeline

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                        URDF → UNITY IMPORT WORKFLOW                          │
└─────────────────────────────────────────────────────────────────────────────┘

┌──────────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│    URDF      │    │   URDF       │    │  GameObject  │    │ Physics-     │
│    File      │───►│   Importer   │───►│  Hierarchy   │───►│ Ready Robot  │
│   (.urdf)    │    │   Package    │    │  (Prefab)    │    │  in Scene    │
└──────────────┘    └──────────────┘    └──────────────┘    └──────────────┘
      │                    │                   │                    │
      │                    ▼                   ▼                    ▼
      │            ┌──────────────┐    ┌──────────────┐    ┌──────────────┐
      │            │ • Parse XML  │    │ • Links →    │    │ • Articula-  │
      │            │ • Load meshes│    │   GameObjects│    │   tionBody   │
      └───────────►│ • Resolve    │    │ • Joints →   │    │ • Drives     │
   + mesh files    │   paths      │    │   Articula-  │    │ • Colliders  │
   (.dae, .stl)    │ • Material   │    │   tions      │    │ • Materials  │
                   │   mapping    │    │ • Visual/    │    │ • Sensors    │
                   └──────────────┘    │   Collision  │    └──────────────┘
                                       └──────────────┘
```

## Step-by-Step Import Process

```text
STEP 1: INSTALL UNITY ROBOTICS HUB
══════════════════════════════════

Unity Package Manager → Add package by name:
  • com.unity.robotics.urdf-importer
  • com.unity.robotics.ros-tcp-connector

┌─────────────────────────────────────────────────────────────┐
│  Window → Package Manager → + → Add package by name...      │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐ │
│  │ Name: com.unity.robotics.urdf-importer               │ │
│  │ Version: 0.5.2-preview (or latest)                    │ │
│  └───────────────────────────────────────────────────────┘ │
│                                                             │
│  [Add]                                                      │
└─────────────────────────────────────────────────────────────┘


STEP 2: PREPARE URDF PACKAGE
════════════════════════════

Required folder structure:

  robot_description/
  ├── urdf/
  │   └── robot.urdf          ← Main URDF file
  ├── meshes/
  │   ├── visual/
  │   │   ├── base_link.dae   ← Visual meshes (Collada preferred)
  │   │   ├── arm_link.dae
  │   │   └── ...
  │   └── collision/
  │       ├── base_link.stl   ← Collision meshes (simplified)
  │       ├── arm_link.stl
  │       └── ...
  └── materials/              ← Optional textures
      ├── base_diffuse.png
      └── base_normal.png


STEP 3: IMPORT INTO UNITY
═════════════════════════

1. Drag robot_description folder into Unity Assets/

2. Right-click on robot.urdf → "Import Robot from URDF"

3. Configure import settings:

┌─────────────────────────────────────────────────────────────┐
│              URDF Import Settings                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Axis Type:  ○ Y-Up (Unity default)                        │
│              ● Z-Up (ROS/URDF convention) ← Select this    │
│                                                             │
│  Mesh Decomposer: ○ None                                   │
│                   ● VHACD (convex decomposition)           │
│                   ○ Unity's Default                        │
│                                                             │
│  ☑ Use URDF Inertia Values                                 │
│  ☑ Create Articulation Bodies (vs Rigidbodies)             │
│  ☑ Import Visual Meshes                                    │
│  ☑ Import Collision Meshes                                 │
│                                                             │
│  [Import]                                                   │
└─────────────────────────────────────────────────────────────┘


STEP 4: VERIFY IMPORT
═════════════════════

Check in Unity Hierarchy:

  robot (ArticulationBody - ROOT)
  ├── base_link (ArticulationBody)
  │   ├── base_link_visual (MeshRenderer)
  │   ├── base_link_collision (MeshCollider)
  │   └── shoulder_link (ArticulationBody) ← Child joint
  │       ├── shoulder_link_visual
  │       ├── shoulder_link_collision
  │       └── elbow_link (ArticulationBody)
  │           └── ...
  └── ...
```

## ArticulationBody Component Details

```text
ARTICULATIONBODY vs RIGIDBODY
═════════════════════════════

┌─────────────────────────────────────────────────────────────────────────────┐
│                                                                             │
│  ArticulationBody (RECOMMENDED)        │  Rigidbody (LEGACY)                │
│  ─────────────────────────────────     │  ──────────────────────            │
│                                        │                                    │
│  ✓ Reduced coordinate physics          │  × Joint constraint solving        │
│  ✓ Stable for long chains              │  × Jitter with many joints         │
│  ✓ Proper joint limits                 │  × Limit violations under load     │
│  ✓ Direct drive control                │  × Requires joint motors           │
│  ✓ Matches ROS joint model             │  × Different joint semantics       │
│                                        │                                    │
│  Use for: Robot arms, humanoids        │  Use for: Simple objects, props    │
│                                        │                                    │
└─────────────────────────────────────────────────────────────────────────────┘


ARTICULATIONBODY INSPECTOR
══════════════════════════

┌─────────────────────────────────────────────────────────────┐
│  Articulation Body                                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Mass: 2.5 ────────────────── kg (from URDF <mass>)        │
│                                                             │
│  ☑ Use Gravity                                              │
│                                                             │
│  Linear Damping: 0.05 ──────── Velocity damping            │
│  Angular Damping: 0.05                                      │
│                                                             │
│  ─────────────────────────────────────────────────────────  │
│  JOINT SETTINGS (if not root)                               │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  Articulation Joint Type: ● Revolute (1-DOF rotation)      │
│                           ○ Prismatic (1-DOF translation)  │
│                           ○ Spherical (3-DOF rotation)     │
│                           ○ Fixed                          │
│                                                             │
│  Anchor Position: X[0] Y[0] Z[0.15] ── Joint origin        │
│  Anchor Rotation: X[0] Y[0] Z[0]                           │
│                                                             │
│  ─────────────────────────────────────────────────────────  │
│  X DRIVE (for revolute about X)                             │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  Lower Limit: -90 ────────────── degrees                   │
│  Upper Limit: 90                                            │
│                                                             │
│  Stiffness: 10000 ────────────── Position gain (Kp)        │
│  Damping: 100 ────────────────── Velocity gain (Kd)        │
│  Force Limit: 1000 ───────────── Max torque (N·m)          │
│                                                             │
│  Target: 0 ───────────────────── Position target (degrees) │
│  Target Velocity: 0 ──────────── Velocity target (deg/s)   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## URDF to Unity Mapping

```text
URDF ELEMENT → UNITY COMPONENT MAPPING
══════════════════════════════════════

URDF Element                    Unity Component              Notes
─────────────────────────────────────────────────────────────────────────────
<robot>                         Root GameObject              Parent container
<link>                          GameObject +                 One per link
                                ArticulationBody
<visual>                        Child GameObject +           Visual mesh
                                MeshFilter +
                                MeshRenderer
<collision>                     Child GameObject +           Physics collision
                                MeshCollider
<inertial>                      ArticulationBody settings    Mass, inertia tensor
<joint type="revolute">         ArticulationBody joint       Revolute type
<joint type="prismatic">        ArticulationBody joint       Prismatic type
<joint type="fixed">            ArticulationBody joint       Fixed type
<joint type="continuous">       ArticulationBody joint       Revolute, no limits
<limit lower="" upper="">       X/Y/Z Drive limits           In degrees
<limit effort="">               X/Y/Z Drive force limit      In N or N·m
<limit velocity="">             Not directly mapped          Use in controller
<dynamics damping="">           Drive damping                Kd coefficient
<material>                      Material asset               Auto-created


COORDINATE FRAME CONVERSION
═══════════════════════════

ROS/URDF (Z-up, X-forward):     Unity (Y-up, Z-forward):

      Z ▲                              Y ▲
        │                                │
        │                                │
        │                                │
        └────────► X                     └────────► Z
       ╱                               ╱
      ╱                               ╱
     ▼ Y                             ▼ X

Importer handles conversion automatically when "Z-up" selected.

JOINT AXIS EXAMPLE:
  URDF: <axis xyz="0 0 1"/>  (rotation about Z)
  Unity: X Drive enabled     (after Z→Y conversion)
```

## Common Import Issues and Solutions

```text
TROUBLESHOOTING URDF IMPORT
═══════════════════════════

┌─────────────────────────────────────────────────────────────────────────────┐
│ ISSUE                          │ CAUSE                 │ SOLUTION           │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ "Mesh not found" error         │ Path mismatch         │ Use package:// or  │
│                                │                       │ relative paths     │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ Robot oriented wrong           │ Axis convention       │ Set "Z-Up" in      │
│                                │                       │ import settings    │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ Joints explode/jitter          │ Using Rigidbody       │ Use Articulation-  │
│                                │                       │ Body instead       │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ Robot falls through floor      │ No collision or       │ Add ground plane   │
│                                │ gravity enabled       │ with collider      │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ Joints don't move              │ Drives not enabled    │ Set Drive stiffness│
│                                │                       │ and damping > 0    │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ Wrong joint limits             │ Radians vs degrees    │ Unity uses degrees;│
│                                │                       │ URDF uses radians  │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ Missing materials              │ URDF material not     │ Manually assign    │
│                                │ supported format      │ Unity materials    │
├────────────────────────────────┼───────────────────────┼────────────────────┤
│ Collision mesh too complex     │ Visual mesh used      │ Create simplified  │
│                                │ for collision         │ collision meshes   │
└─────────────────────────────────────────────────────────────────────────────┘


MESH PATH RESOLUTION
════════════════════

URDF mesh reference formats:

1. Package path (ROS convention):
   <mesh filename="package://robot_description/meshes/base.dae"/>
   → Unity resolves to: Assets/robot_description/meshes/base.dae

2. Relative path:
   <mesh filename="meshes/base.dae"/>
   → Unity resolves relative to URDF file location

3. Absolute path (NOT recommended):
   <mesh filename="/home/user/robot/meshes/base.dae"/>
   → Will fail in Unity; avoid absolute paths


SUPPORTED MESH FORMATS
══════════════════════

Format      Visual    Collision    Notes
────────────────────────────────────────────────────────
.dae        ✓         ✓            Best for textures
.stl        ✓         ✓ ★          Preferred for collision
.obj        ✓         △            Limited material support
.fbx        ✓         ✓            Unity native format
.glb/.gltf  ✓         ✓            Modern format, good textures

★ = Recommended for collision (simple geometry)
```

## Post-Import Configuration

```text
POST-IMPORT SETUP CHECKLIST
═══════════════════════════

□ PHYSICS LAYER SETUP
  ├── Create "Robot" layer
  ├── Assign all robot parts to Robot layer
  └── Configure collision matrix (Robot vs Environment)

□ DRIVE TUNING
  ├── Adjust stiffness for position control response
  ├── Adjust damping to prevent oscillation
  └── Set appropriate force limits

□ VISUAL POLISH
  ├── Replace materials with Unity PBR materials
  ├── Add emission for LEDs/screens
  └── Configure shadows and reflections

□ SENSOR ATTACHMENT
  ├── Add Camera component for vision
  ├── Add contact sensors (OnCollisionEnter)
  └── Add ray-based sensors (Physics.Raycast)

□ ROS INTEGRATION
  ├── Add JointStatePublisher script
  ├── Add JointCommandSubscriber script
  └── Configure ROSConnection


EXAMPLE: JOINT CONTROL SCRIPT
═════════════════════════════

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointController : MonoBehaviour
{
    private ArticulationBody[] joints;

    void Start()
    {
        // Find all articulation bodies in hierarchy
        joints = GetComponentsInChildren<ArticulationBody>();

        // Subscribe to joint commands
        ROSConnection.GetOrCreateInstance()
            .Subscribe<JointStateMsg>("/joint_commands", OnJointCommand);
    }

    void OnJointCommand(JointStateMsg msg)
    {
        for (int i = 0; i < msg.position.Length; i++)
        {
            if (i < joints.Length)
            {
                var drive = joints[i].xDrive;
                // Convert radians to degrees
                drive.target = (float)(msg.position[i] * Mathf.Rad2Deg);
                joints[i].xDrive = drive;
            }
        }
    }
}
```

## Usage in Book

- **Referenced in**: Chapter 5 (Core Concept 2: Importing Robot Models)
- **Purpose**: Guide learners through URDF import process for Unity simulation
- **Learning Goal**: Successfully import a robot model and configure joints for ROS 2 control

## Key Takeaways

1. **Unity Robotics Hub**: Essential package for URDF import and ROS integration
2. **ArticulationBody**: Use instead of Rigidbody for stable multi-joint robots
3. **Z-up conversion**: Always select "Z-Up" to match ROS/URDF coordinate conventions
4. **Drive tuning**: Stiffness (Kp) and damping (Kd) control joint response characteristics
5. **Mesh preparation**: Use .dae for visuals, simplified .stl for collisions
6. **Path resolution**: Use package:// or relative paths; avoid absolute paths
