# ROS 2 Package Structure Diagram Specification

**Purpose**: Illustrate the standard directory structure and files of a ROS 2 Python package

**Diagram Type**: Directory tree / File hierarchy diagram

## Package Directory Structure

```
my_robot_package/                  # Root package directory
├── package.xml                    # Package metadata and dependencies
├── setup.py                       # Python package setup (entry points, install rules)
├── setup.cfg                      # Python setup configuration
├── resource/                      # Package marker files
│   └── my_robot_package           # Empty marker file (required)
├── my_robot_package/              # Python module directory (same name as package)
│   ├── __init__.py                # Makes directory a Python module
│   ├── publisher_node.py          # Publisher node implementation
│   ├── subscriber_node.py         # Subscriber node implementation
│   └── ai_agent_node.py           # AI agent node implementation
├── launch/                        # Launch files (optional but recommended)
│   ├── robot_bringup.launch.py    # Launch file to start all nodes
│   └── simulation.launch.py       # Launch file for simulation mode
├── config/                        # Configuration files (optional)
│   └── robot_params.yaml          # Default parameter values
├── test/                          # Unit tests (optional but recommended)
│   ├── test_publisher.py
│   └── test_subscriber.py
└── README.md                      # Package documentation (optional)
```

## Key Files Explained

### 1. `package.xml` (Required)

**Purpose**: Package metadata, dependencies, and build information

**Content Example**:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>ROS 2 package for humanoid robot control</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Key Fields**:
- **`<name>`**: Package name (must match directory name)
- **`<exec_depend>`**: Runtime dependencies (rclpy, message packages)
- **`<test_depend>`**: Testing dependencies
- **`<build_type>`**: `ament_python` for Python packages, `ament_cmake` for C++

### 2. `setup.py` (Required)

**Purpose**: Python package installation and entry point definitions

**Content Example**:
```python
from setuptools import setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/robot_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 package for humanoid robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = my_robot_package.publisher_node:main',
            'subscriber_node = my_robot_package.subscriber_node:main',
            'ai_agent_node = my_robot_package.ai_agent_node:main',
        ],
    },
)
```

**Key Sections**:
- **`packages`**: List of Python modules to install
- **`data_files`**: Non-Python files to install (launch files, config files, etc.)
- **`entry_points`**: Executables that can be run with `ros2 run <package> <node>`

### 3. `my_robot_package/__init__.py` (Required)

**Purpose**: Makes the directory a Python module

**Content**: Usually empty or contains package-level imports

```python
# Empty file (required for Python modules)
```

### 4. Launch Files (`launch/*.launch.py`) (Optional but Recommended)

**Purpose**: Start multiple nodes with parameters and configuration

**See**: `simple_launch.py` example for complete launch file structure

### 5. Configuration Files (`config/*.yaml`) (Optional)

**Purpose**: Store default parameter values for nodes

**Example**: `robot_params.yaml`
```yaml
/my_robot_package:
  ai_agent_node:
    ros__parameters:
      decision_threshold: 50.0
      max_motor_speed: 100.0
      enable_safety_check: true
```

## Package Creation Workflow

### Step 1: Create Package

```bash
# Navigate to your ROS 2 workspace src directory
cd ~/ros2_ws/src

# Create a new Python package
ros2 pkg create --build-type ament_python my_robot_package --dependencies rclpy std_msgs sensor_msgs

# This creates the basic package structure with package.xml and setup.py
```

### Step 2: Add Node Code

```bash
# Create Python files in the package module directory
cd my_robot_package/my_robot_package
touch publisher_node.py
touch subscriber_node.py
touch ai_agent_node.py

# Edit files and implement your nodes
```

### Step 3: Update setup.py Entry Points

```python
entry_points={
    'console_scripts': [
        'publisher_node = my_robot_package.publisher_node:main',
        'subscriber_node = my_robot_package.subscriber_node:main',
        'ai_agent_node = my_robot_package.ai_agent_node:main',
    ],
},
```

### Step 4: Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select my_robot_package

# Source the workspace
source install/setup.bash
```

### Step 5: Run Nodes

```bash
# Run individual nodes
ros2 run my_robot_package publisher_node
ros2 run my_robot_package ai_agent_node

# Or use launch file
ros2 launch my_robot_package robot_bringup.launch.py
```

## Package Dependencies

### Common ROS 2 Message Packages

- **`std_msgs`**: Basic message types (String, Float32, Int32, Bool, etc.)
- **`sensor_msgs`**: Sensor data (Image, LaserScan, Imu, JointState, etc.)
- **`geometry_msgs`**: Geometric primitives (Point, Pose, Twist, Transform, etc.)
- **`nav_msgs`**: Navigation (Odometry, Path, OccupancyGrid, etc.)
- **`trajectory_msgs`**: Trajectories (JointTrajectory for arm motion)
- **`control_msgs`**: Control actions (FollowJointTrajectory, GripperCommand)

### Adding Dependencies

To add a dependency to your package:

1. **In `package.xml`**, add:
   ```xml
   <exec_depend>geometry_msgs</exec_depend>
   ```

2. **In `setup.py`**, update the package creation command (or modify manually if already created):
   ```bash
   ros2 pkg create --build-type ament_python my_robot_package \
     --dependencies rclpy std_msgs sensor_msgs geometry_msgs
   ```

3. **Rebuild the package**:
   ```bash
   colcon build --packages-select my_robot_package
   ```

## Best Practices

### Package Naming
- Use lowercase with underscores: `my_robot_package` (not `MyRobotPackage` or `my-robot-package`)
- Be descriptive: `humanoid_vision`, `balance_controller`, `arm_kinematics`

### Code Organization
- **One node per file**: `publisher_node.py`, `subscriber_node.py` (not `all_nodes.py`)
- **Shared utilities**: Create `utils.py` for common functions used by multiple nodes
- **Keep launch files simple**: Complex logic belongs in nodes, not launch files

### Version Control
- **Include**: `package.xml`, `setup.py`, Python code, launch files, config files, README
- **Exclude (gitignore)**: `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`

## Visual Layout

```
┌──────────────────────────────────────────────┐
│  ROS 2 Package: my_robot_package             │
├──────────────────────────────────────────────┤
│  Configuration Layer                         │
│  ├── package.xml (metadata, dependencies)    │
│  ├── setup.py (entry points, install rules)  │
│  └── setup.cfg (Python config)               │
├──────────────────────────────────────────────┤
│  Python Module Layer                         │
│  ├── my_robot_package/ (module directory)    │
│  │   ├── __init__.py                         │
│  │   ├── publisher_node.py (executable)      │
│  │   ├── subscriber_node.py (executable)     │
│  │   └── ai_agent_node.py (executable)       │
├──────────────────────────────────────────────┤
│  Launch & Config Layer                       │
│  ├── launch/ (launch files)                  │
│  │   └── robot_bringup.launch.py             │
│  └── config/ (YAML parameter files)          │
│      └── robot_params.yaml                   │
├──────────────────────────────────────────────┤
│  Testing Layer (optional)                    │
│  └── test/ (unit tests)                      │
│      ├── test_publisher.py                   │
│      └── test_subscriber.py                  │
└──────────────────────────────────────────────┘
```

## Usage in Book

- **Referenced in**: Chapter 4 (Core Concept 1: ROS 2 Package Anatomy)
- **Purpose**: Help learners understand how to organize ROS 2 Python code into packages for reusability and distribution
- **Key Question**: "Why not just run standalone Python scripts?" → Answer: Packages enable dependency management, installation, discovery, and sharing with the community
