# URDF to SDF Conversion Workflow Specification

**Purpose**: Guide learners through the process of converting URDF robot descriptions to SDF format for Gazebo simulation

**Diagram Type**: workflow

## Conversion Workflow Overview

```text
┌─────────────────────────────────────────────────────────────────┐
│                 URDF → SDF CONVERSION WORKFLOW                  │
└─────────────────────────────────────────────────────────────────┘

Step 1: VALIDATE URDF
┌──────────────────────────────────────────────────────────────┐
│  $ check_urdf robot.urdf                                     │
│                                                              │
│  ✓ Link count: 10                                           │
│  ✓ Joint count: 9                                           │
│  ✓ Tree structure valid                                     │
│  ✓ No floating links                                        │
│                                                              │
│  Output: URDF is valid or errors to fix                     │
└──────────────────────────────────────────────────────────────┘
       │
       │ If valid
       ↓
Step 2: CONVERT TO SDF
┌──────────────────────────────────────────────────────────────┐
│  $ gz sdf -p robot.urdf > robot.sdf                          │
│                                                              │
│  Gazebo parses URDF and outputs equivalent SDF               │
│                                                              │
│  Automatic conversions:                                      │
│  • <robot> → <model>                                        │
│  • <gazebo> extensions → native SDF plugins                 │
│  • Material colors preserved                                │
│                                                              │
│  Lost in conversion:                                        │
│  • ROS-specific tags (transmission, ros2_control)           │
│  • Must re-add as SDF plugins                               │
└──────────────────────────────────────────────────────────────┘
       │
       ↓
Step 3: VALIDATE SDF
┌──────────────────────────────────────────────────────────────┐
│  $ gz sdf -k robot.sdf                                       │
│                                                              │
│  Checks SDF against schema:                                  │
│  ✓ Valid SDF version                                        │
│  ✓ Required elements present                                │
│  ✓ No malformed XML                                         │
│                                                              │
│  Common warnings (OK to ignore):                            │
│  ⚠ Link with no inertia (will be treated as static)        │
│  ⚠ Collision without visual (performance optimization)      │
└──────────────────────────────────────────────────────────────┘
       │
       ↓
Step 4: ADD GAZEBO-SPECIFIC FEATURES
┌──────────────────────────────────────────────────────────────┐
│  Manual additions to robot.sdf:                             │
│                                                              │
│  <!-- Sensors (not in URDF) -->                             │
│  <sensor name="lidar" type="gpu_lidar">                     │
│    <lidar>...</lidar>                                       │
│    <plugin>...</plugin>                                     │
│  </sensor>                                                  │
│                                                              │
│  <!-- Plugins (replace <gazebo> tags) -->                   │
│  <plugin filename="gz-sim-diff-drive-system"                │
│          name="gz::sim::systems::DiffDrive">                │
│    ...                                                      │
│  </plugin>                                                  │
│                                                              │
│  <!-- Surface properties -->                                │
│  <surface><friction>...</friction></surface>                │
└──────────────────────────────────────────────────────────────┘
       │
       ↓
Step 5: TEST IN GAZEBO
┌──────────────────────────────────────────────────────────────┐
│  $ gz sim world_with_robot.sdf                               │
│                                                              │
│  Verification checklist:                                     │
│  □ Robot spawns at correct pose                             │
│  □ No collision warnings in console                         │
│  □ Joints move within limits                                │
│  □ Sensors publish data                                     │
│  □ Robot stable under gravity                               │
└──────────────────────────────────────────────────────────────┘
```

## Command Reference

### Validation Commands

```bash
# Validate URDF syntax and structure
check_urdf robot.urdf

# Alternative: Use urdf_parser_py
ros2 run urdfdom_py display_urdf robot.urdf

# Validate SDF syntax
gz sdf -k robot.sdf

# Print SDF in canonical form (helps debugging)
gz sdf -p robot.sdf --print
```

### Conversion Commands

```bash
# Basic conversion: URDF → SDF
gz sdf -p robot.urdf > robot.sdf

# Convert with specific SDF version
gz sdf -p robot.urdf --sdf-version 1.9 > robot.sdf

# Convert xacro first, then to SDF
ros2 run xacro xacro robot.urdf.xacro > robot.urdf
gz sdf -p robot.urdf > robot.sdf

# One-liner (xacro → URDF → SDF)
ros2 run xacro xacro robot.urdf.xacro | gz sdf -p /dev/stdin > robot.sdf
```

### Inspection Commands

```bash
# Print URDF link/joint tree
check_urdf robot.urdf --print

# List all models in an SDF file
gz model -l -f robot.sdf

# Print model info from Gazebo server
gz model -m robot_name --info
```

## Common Conversion Issues

### Issue 1: Missing Inertia Values

```text
SYMPTOM:
─────────
Warning: Link 'my_link' has no inertia element.
         Treating as static (massless).

CAUSE:
──────
URDF link missing <inertial> block.

SOLUTION:
─────────
Add inertial properties to URDF before converting:

<link name="my_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0"
             iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  ...
</link>
```

### Issue 2: Gazebo Extensions Not Converted

```text
SYMPTOM:
─────────
Plugins from <gazebo> tags don't work in new Gazebo.

CAUSE:
──────
<gazebo> tags use Gazebo Classic plugin syntax.
Gazebo (Harmonic) uses different plugin names.

SOLUTION:
─────────
Replace Gazebo Classic plugins with new equivalents:

BEFORE (URDF with Gazebo Classic):
<gazebo>
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    ...
  </plugin>
</gazebo>

AFTER (SDF with Gazebo Harmonic):
<plugin filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_radius>0.1</wheel_radius>
  <topic>cmd_vel</topic>
</plugin>
```

### Issue 3: Mesh Path Not Found

```text
SYMPTOM:
─────────
Error: Unable to find file [package://my_robot/meshes/base.dae]

CAUSE:
──────
SDF doesn't understand ROS package:// URIs.

SOLUTION:
─────────
Option A: Use absolute paths in SDF
  <mesh><uri>file:///home/user/ws/src/my_robot/meshes/base.dae</uri></mesh>

Option B: Use model:// URIs (set GZ_SIM_RESOURCE_PATH)
  export GZ_SIM_RESOURCE_PATH=$HOME/ws/src/my_robot
  <mesh><uri>model://meshes/base.dae</uri></mesh>

Option C: Copy meshes to Gazebo model directory
  ~/.gz/models/my_robot/meshes/base.dae
  <mesh><uri>model://my_robot/meshes/base.dae</uri></mesh>
```

### Issue 4: Joint Limits Violated

```text
SYMPTOM:
─────────
Warning: Joint 'arm_joint' position 2.5 exceeds upper limit 1.57

CAUSE:
──────
Initial pose or controller commands exceed URDF limits.

SOLUTION:
─────────
1. Check joint limits in SDF match URDF:
   <axis>
     <limit>
       <lower>-1.57</lower>
       <upper>1.57</upper>
     </limit>
   </axis>

2. Verify initial joint positions in world spawn:
   <include>
     <uri>model://my_robot</uri>
     <pose>0 0 0.5 0 0 0</pose>
     <!-- Joint initial positions -->
     <plugin name="gz::sim::systems::JointPositionController">
       <joint_name>arm_joint</joint_name>
       <initial_position>0.0</initial_position>
     </plugin>
   </include>
```

### Issue 5: Collision Self-Intersection

```text
SYMPTOM:
─────────
Robot jitters or explodes on spawn.

CAUSE:
──────
Adjacent links' collision geometries overlap in initial pose.

SOLUTION:
─────────
1. Visualize collisions in Gazebo (View → Collisions)

2. Shrink collision geometry slightly:
   <!-- Visual geometry -->
   <visual>
     <geometry><cylinder radius="0.05" length="0.3"/></geometry>
   </visual>
   <!-- Collision geometry (slightly smaller) -->
   <collision>
     <geometry><cylinder radius="0.048" length="0.29"/></geometry>
   </collision>

3. Adjust link origins to prevent overlap:
   <link name="forearm">
     <pose>0 0 0.002 0 0 0</pose>  <!-- 2mm gap -->
     ...
   </link>

4. Disable self-collision (last resort):
   <model name="robot">
     <self_collide>false</self_collide>
     ...
   </model>
```

## Plugin Migration Table

| Gazebo Classic Plugin | Gazebo Harmonic Plugin |
|----------------------|------------------------|
| `libgazebo_ros_joint_state_publisher.so` | `gz-sim-joint-state-publisher-system` |
| `libgazebo_ros_diff_drive.so` | `gz-sim-diff-drive-system` |
| `libgazebo_ros_ray_sensor.so` | `gz-sim-sensors-system` + `<sensor type="gpu_lidar">` |
| `libgazebo_ros_camera.so` | `gz-sim-sensors-system` + `<sensor type="camera">` |
| `libgazebo_ros_imu_sensor.so` | `gz-sim-imu-system` + `<sensor type="imu">` |
| `libgazebo_ros_p3d.so` | `gz-sim-odometry-publisher-system` |
| `libgazebo_ros_ft_sensor.so` | `gz-sim-forcetorque-system` |
| `libgazebo_ros_control.so` | `gz_ros2_control` (separate package) |

## Workflow Decision Tree

```text
START: Have robot.urdf
         │
         ▼
┌─────────────────────┐
│ Is URDF valid?      │──────── NO ────► Fix URDF errors first
│ (check_urdf)        │
└─────────┬───────────┘
          │ YES
          ▼
┌─────────────────────┐
│ Using xacro?        │──────── YES ───► Expand: xacro robot.urdf.xacro > robot.urdf
└─────────┬───────────┘
          │ NO / Done
          ▼
┌─────────────────────┐
│ gz sdf -p robot.urdf│
│ > robot.sdf         │
└─────────┬───────────┘
          ▼
┌─────────────────────┐
│ Is SDF valid?       │──────── NO ────► Fix conversion warnings
│ (gz sdf -k)         │
└─────────┬───────────┘
          │ YES
          ▼
┌─────────────────────┐
│ Need sensors?       │──────── YES ───► Add <sensor> elements to SDF
└─────────┬───────────┘
          │ NO / Done
          ▼
┌─────────────────────┐
│ Need plugins?       │──────── YES ───► Replace <gazebo> tags with SDF plugins
└─────────┬───────────┘
          │ NO / Done
          ▼
┌─────────────────────┐
│ Test in Gazebo      │──────── FAIL ──► Debug using issue solutions above
│ (gz sim world.sdf)  │
└─────────┬───────────┘
          │ PASS
          ▼
       ✓ DONE
```

## Usage in Book

- **Referenced in**: Chapter 3 (Core Concept 1: URDF vs SDF)
- **Purpose**: Provide step-by-step guidance for the most common workflow students will need
- **Learning Goal**: Successfully convert existing URDF robots to work in Gazebo Harmonic

## Key Takeaways

1. **Validate before converting**: Use `check_urdf` and `gz sdf -k` to catch errors early
2. **One command conversion**: `gz sdf -p robot.urdf > robot.sdf` handles most cases
3. **Plugins require migration**: Gazebo Classic plugins don't work in Harmonic; use new names
4. **Mesh paths differ**: Replace `package://` with `model://` or absolute paths
5. **Add Gazebo features post-conversion**: Sensors, plugins, surface properties added to SDF
6. **Test iteratively**: Spawn, verify joints, check collisions, confirm sensors publish
