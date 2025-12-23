---
sidebar_position: 4
title: Chapter 4 - ROS 2 Packages and Launch Files
---

# Chapter 4: ROS 2 Packages and Launch Files

## Introduction

In Chapter 3, you learned to write individual ROS 2 Python nodes (publishers, subscribers, parameter handlers). While running standalone Python scripts works for simple experiments, real robot systems require organizing code into **packages** and starting multiple nodes simultaneously with **launch files**.

This chapter teaches you how to structure ROS 2 projects for scalability and reusability. You'll learn the anatomy of a ROS 2 package (`package.xml`, `setup.py`, module structure), how to write Python launch files to configure multi-node systems, and most importantly - how to bridge Python-based AI decision logic (neural networks, vision models, planners) into ROS 2 nodes that interact with robot hardware.

By the end of this chapter, you'll be able to integrate AI "brains" (e.g., a PyTorch policy network, a YOLO vision model, or an LLM task planner) with robotic "bodies" via ROS 2, creating complete physical AI systems.

**Prerequisites**:
- Chapter 3 (Python nodes, publishers, subscribers, parameters)
- Understanding of Python packages and modules
- Familiarity with AI frameworks (PyTorch, TensorFlow, etc.) is helpful but not required

---

## Core Concept 1: ROS 2 Package Anatomy

### What is a ROS 2 Package?

A **ROS 2 package** is a unit of organization for robot software. Packages:
- Contain related nodes, launch files, and configuration
- Declare dependencies on other packages (message types, libraries)
- Can be built, installed, and shared with the community
- Enable version control and dependency management

Think of a package as a Python module with extra robotics-specific metadata.

### Package Structure

See the [Package Structure Diagram](/docs/diagrams/module-1/package-structure.md) for the complete directory layout.

**Essential Files**:

#### 1. `package.xml` (Required)

Package metadata and dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>ROS 2 package for humanoid robot control</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Key Fields**:
- **`name`**: Package name (must match directory)
- **`exec_depend`**: Runtime dependencies (rclpy, message types)
- **`build_type`**: `ament_python` for Python packages

#### 2. `setup.py` (Required)

Python package setup and entry points:

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'publisher_node = my_robot_package.publisher_node:main',
            'ai_agent_node = my_robot_package.ai_agent_node:main',
        ],
    },
)
```

**`entry_points`**: Maps executable names to Python functions. After building, you can run:
```bash
ros2 run my_robot_package publisher_node
```

This executes `main()` from `my_robot_package/publisher_node.py`.

#### 3. Python Module Directory

```
my_robot_package/
├── __init__.py           # Required (can be empty)
├── publisher_node.py     # Node implementations
├── subscriber_node.py
└── ai_agent_node.py
```

### Creating a Package

```bash
# Navigate to ROS 2 workspace src/ directory
cd ~/ros2_ws/src

# Create package with dependencies
ros2 pkg create --build-type ament_python my_robot_package \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs

# This generates: package.xml, setup.py, module directory, resource/
```

### Building and Installing

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build the package
colcon build --packages-select my_robot_package

# Source the workspace (makes package discoverable)
source install/setup.bash

# Run a node from the package
ros2 run my_robot_package publisher_node
```

---

## Core Concept 2: Launch Files and Configuration

### What are Launch Files?

**Launch files** are Python scripts that configure and start multiple ROS 2 nodes simultaneously. Launch files:
- Start nodes with specific parameters
- Remap topic names (e.g., `/camera` → `/front_camera`)
- Set namespaces for multi-robot systems
- Configure logging levels and output destinations
- Enable coordinated startup/shutdown

**Why Launch Files?** Instead of running:
```bash
ros2 run pkg node1 &
ros2 run pkg node2 --ros-args -p param:=value &
ros2 run pkg node3 &
```

You write one launch file and run:
```bash
ros2 launch pkg robot_bringup.launch.py
```

All nodes start with correct configuration, and Ctrl+C stops all nodes cleanly.

### Launch File Structure

Launch files use the Python `launch` API:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='publisher_node',
            name='sensor_publisher',
            output='screen',
            parameters=[{
                'update_rate': 10.0,
            }]
        ),
        Node(
            package='my_robot_package',
            executable='subscriber_node',
            name='data_logger',
            output='screen',
        ),
    ])
```

**`LaunchDescription`**: Container for launch actions (nodes, includes, parameters, etc.)

**`Node` action fields**:
- **`package`**: ROS 2 package name
- **`executable`**: Entry point name (from `setup.py`)
- **`name`**: Node name (overrides default from code)
- **`output`**: `'screen'` = print to terminal, `'log'` = write to file
- **`parameters`**: List of dicts with parameter key-value pairs
- **`remappings`**: List of tuples for topic renaming
- **`namespace`**: Prefix for all node topics (e.g., `'robot1'` → `/robot1/sensor_data`)

---

## Hands-On Example 1: Launch File

Let's create a launch file that starts a publisher and two subscribers with different configurations.

**File**: `simple_launch.py`

```python
#!/usr/bin/env python3
"""
ROS 2 Python Launch File Example

Starts multiple nodes with parameters and remapping.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Node 1: Publisher with custom update rate
        Node(
            package='my_robot_package',
            executable='publisher_node',
            name='sensor_publisher',
            output='screen',
            parameters=[{
                'update_rate': 5.0,  # 5 Hz instead of default 10 Hz
            }]
        ),

        # Node 2: First subscriber (default namespace)
        Node(
            package='my_robot_package',
            executable='subscriber_node',
            name='data_logger',
            output='screen',
        ),

        # Node 3: Second subscriber (in separate namespace)
        Node(
            package='my_robot_package',
            executable='subscriber_node',
            name='data_processor',
            namespace='processing',  # Node becomes /processing/data_processor
            output='screen',
        ),
    ])
```

### Running the Launch File

```bash
# Save launch file to launch/ directory in your package
cp simple_launch.py ~/ros2_ws/src/my_robot_package/launch/

# Update setup.py to install launch file (add to data_files)
# Then rebuild:
colcon build --packages-select my_robot_package
source install/setup.bash

# Run the launch file
ros2 launch my_robot_package simple_launch.py

# Expected output:
# [INFO] [sensor_publisher]: Publishing at 5.0 Hz
# [INFO] [data_logger]: Subscribed to /sensor_data
# [INFO] [processing.data_processor]: Subscribed to /processing/sensor_data
```

### Advanced: Launch Arguments

Allow users to override parameters when launching:

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare command-line argument
        DeclareLaunchArgument(
            'update_rate',
            default_value='10.0',
            description='Publisher update rate in Hz'
        ),

        # Use the argument in a node
        Node(
            package='my_robot_package',
            executable='publisher_node',
            name='sensor_publisher',
            parameters=[{
                'update_rate': LaunchConfiguration('update_rate'),
            }]
        ),
    ])
```

**Usage**:
```bash
ros2 launch my_robot_package robot_bringup.launch.py update_rate:=20.0
```

---

## Core Concept 3: Multi-Node Coordination

### Namespaces

**Namespaces** prevent topic name conflicts in multi-robot or multi-component systems:

```python
# Robot 1 in namespace 'robot1'
Node(
    package='control_pkg',
    executable='controller',
    namespace='robot1',
)
# Topics become: /robot1/sensor_data, /robot1/cmd_vel

# Robot 2 in namespace 'robot2'
Node(
    package='control_pkg',
    executable='controller',
    namespace='robot2',
)
# Topics become: /robot2/sensor_data, /robot2/cmd_vel
```

Both robots run the same node code but communicate on separate topics.

### Topic Remapping

**Remapping** changes topic names without modifying code:

```python
Node(
    package='vision_pkg',
    executable='object_detector',
    remappings=[
        ('camera/image', 'front_camera/image_raw'),  # Subscribe to different camera
        ('detections', 'obstacles'),                  # Publish with different name
    ]
)
```

**Use Case**: Swap sensor sources (front camera → rear camera) without changing detector code.

### Lifecycle Management

For safety-critical systems, use lifecycle nodes in launch files:

```python
from launch_ros.actions import LifecycleNode
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition

# Create lifecycle node
lifecycle_node = LifecycleNode(
    package='control_pkg',
    executable='motor_controller',
    name='motor_ctrl',
    namespace='',
    output='screen',
)

# Trigger transitions: configure → activate
# (Advanced topic - covered in ROS 2 documentation)
```

This ensures nodes start in a controlled sequence (sensors before controllers).

---

## Core Concept 4: Bridging AI Agents to ROS

### The AI-ROS Integration Pattern

Integrating AI decision logic into ROS 2 involves:

- **Step 1 - Wrap AI logic as a ROS 2 node**: Create a `Node` subclass with AI model in `__init__()`
- **Step 2 - Subscribe to sensor data**: Camera images, LiDAR scans, joint states
- **Step 3 - Run inference in callback**: Preprocess → model.predict() → postprocess
- **Step 4 - Publish commands**: Joint trajectories, velocity commands, grasp poses
- **Step 5 - Add safety checks**: Clamp outputs, handle errors, timeout detection

See the [AI-ROS Integration Diagram](/docs/diagrams/module-1/ai-ros-integration.md) for the complete data flow.

### Pattern: Sensor → AI Decision → Command

**Conceptual Flow**:
```
Sensor Hardware → Sensor Node → /sensor_data topic
                                      ↓
                                 AI Agent Node
                                 - Callback receives sensor data
                                 - Preprocesses input
                                 - Runs model.predict()
                                 - Generates motor command
                                      ↓
                      /motor_commands topic → Motor Controller → Actuators
```

**Key Insight**: The AI agent is **just another ROS 2 node**. It subscribes to topics, publishes to topics, and integrates seamlessly with the rest of the robot system.

---

## Hands-On Example 2: AI Agent Node

Let's create an AI agent node that processes sensor input and publishes motor commands.

**File**: `ai_agent_node.py`

```python
#!/usr/bin/env python3
"""
AI Agent Node - Bridging AI Decision Logic to ROS 2

Demonstrates integrating Python AI logic as a ROS 2 node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class AIAgentNode(Node):
    """AI Agent: Processes sensor input and publishes control commands."""

    def __init__(self):
        super().__init__('ai_agent_node')

        # Parameters for AI configuration
        self.declare_parameter('decision_threshold', 50.0)
        self.declare_parameter('max_motor_speed', 100.0)

        self.threshold = self.get_parameter('decision_threshold').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_motor_speed').get_parameter_value().double_value

        # Subscribe to sensor input
        self.sensor_subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Publish motor commands
        self.command_publisher = self.create_publisher(Float32, 'motor_commands', 10)

        # Publish status for monitoring
        self.status_publisher = self.create_publisher(String, 'ai_status', 10)

        self.get_logger().info(f'AI Agent started (threshold={self.threshold})')

    def sensor_callback(self, msg):
        """AI decision logic: sensor input → motor command."""
        sensor_value = msg.data

        # AI Decision (simplified threshold-based logic)
        # In a real system: motor_command = self.model.predict(sensor_value)
        error = sensor_value - self.threshold
        motor_command = error * 2.0  # Proportional control

        # Safety limits
        motor_command = self.apply_safety_limits(motor_command)

        # Publish command
        cmd_msg = Float32()
        cmd_msg.data = motor_command
        self.command_publisher.publish(cmd_msg)

        # Publish status
        status_msg = String()
        status_msg.data = f'Sensor: {sensor_value:.2f} | Command: {motor_command:.2f}'
        self.status_publisher.publish(status_msg)

        self.get_logger().info(f'Sensor={sensor_value:.2f} → Motor={motor_command:.2f}')

    def apply_safety_limits(self, command):
        """Clamp commands to safe range."""
        if command > self.max_speed:
            return self.max_speed
        elif command < -self.max_speed:
            return -self.max_speed
        return command


def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('AI Agent shutting down...')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the AI Agent with Mock Sensor

```bash
# Terminal 1: Start mock sensor (publisher from Chapter 3)
python3 publisher_example.py

# Terminal 2: Start AI agent
python3 ai_agent_node.py

# Terminal 3: Monitor motor commands
ros2 topic echo /motor_commands

# Expected flow:
# Publisher emits 0.0, 1.0, 2.0, ... on /sensor_data
# AI Agent receives values, computes commands, publishes to /motor_commands
# Motor commands: -100.0 (clamped), -98.0, -96.0, ..., 0.0 (at threshold 50), 2.0, 4.0, ...
```

### Integrating Real AI Models

To integrate a PyTorch neural network:

```python
import torch

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Load trained model
        self.model = torch.load('path/to/model.pth')
        self.model.eval()

        # ... (rest of init)

    def sensor_callback(self, msg):
        # Preprocess sensor data to tensor
        input_tensor = torch.tensor([[msg.data]], dtype=torch.float32)

        # Run inference
        with torch.no_grad():
            output = self.model(input_tensor)

        # Extract motor command
        motor_command = output.item()

        # Apply safety and publish
        motor_command = self.apply_safety_limits(motor_command)
        self.command_publisher.publish(Float32(data=motor_command))
```

**For Vision Models** (e.g., YOLO object detection):
- Subscribe to `sensor_msgs/Image` instead of `Float32`
- Convert ROS Image to numpy/OpenCV format using `cv_bridge`
- Run `results = model(image)`
- Extract bounding boxes, publish target positions

---

## Diagrams

### Package Structure

See the [Package Structure Diagram](/docs/diagrams/module-1/package-structure.md) for:
- Complete directory tree (`package.xml`, `setup.py`, `launch/`, `config/`, `test/`)
- File organization best practices
- Dependency management workflow
- Build and installation process

**Key Takeaway**: Packages provide structure for organizing nodes, launch files, and config. Proper package structure enables code reuse and community sharing.

### AI-ROS Integration Architecture

See the [AI-ROS Integration Diagram](/docs/diagrams/module-1/ai-ros-integration.md) for:
- Complete data flow from sensor → AI agent → motor controller
- AI decision logic patterns (threshold, neural network, vision, LLM)
- Safety layer and error handling
- Feedback loop (motor encoders → joint states → AI agent)
- Timing and performance considerations

**Example Flow**:
1. Sensor publishes data at 30 Hz
2. AI agent callback triggered, preprocesses input (~5 ms)
3. Model inference (10-100 ms depending on model size)
4. Post-process and safety checks (~5 ms)
5. Publish motor commands
6. Motor controller executes at 100 Hz

**Key Insight**: AI inference can be slower than sensor rate. The AI agent publishes commands when ready; motor controller uses latest available command.

---

## Summary

This chapter taught ROS 2 package organization and AI integration:

1. **ROS 2 Package Anatomy**: Packages contain `package.xml` (metadata, dependencies), `setup.py` (entry points), and a Python module directory. Create with `ros2 pkg create`, build with `colcon build`, run with `ros2 run`.

2. **Launch Files**: Python scripts using `LaunchDescription` and `Node` actions to start multiple nodes with parameters, remapping, and namespaces. Enable coordinated multi-node startup and configuration without code changes.

3. **Multi-Node Coordination**: Namespaces prevent topic conflicts (`/robot1/cmd_vel` vs `/robot2/cmd_vel`). Remapping allows reusing nodes with different sensors (`camera/image` → `front_camera/image`). Lifecycle nodes enable controlled startup (configure → activate).

4. **AI-ROS Integration**: Wrap AI models (PyTorch, TensorFlow, YOLO, LLMs) as ROS 2 nodes. Subscribe to sensor topics, run inference in callbacks, publish commands. Add safety limits to clamp outputs. Use feedback loops (subscribe to `/joint_states`) for closed-loop control.

5. **Real-World Pattern**: Sensor nodes → AI agent node (decision logic) → motor controller node → actuators. Each component is a ROS 2 node communicating via topics. AI agent can be swapped (rule-based → neural network → LLM) without changing other nodes.

6. **Practical Skills**: Organize code into packages for reusability. Use launch files to manage multi-node systems. Integrate existing AI code (Python libraries) into ROS 2 seamlessly. Build complete physical AI systems where intelligent algorithms control physical robots.

---

## Self-Assessment Checklist

After completing this chapter, you should be able to:

- [ ] **I can create** a ROS 2 Python package with `package.xml`, `setup.py`, and module structure using `ros2 pkg create`.
- [ ] **I can write** a Python launch file that starts multiple nodes with parameters and remapping using `LaunchDescription` and `Node` actions.
- [ ] **I can run** a launch file with `ros2 launch` and verify all nodes start correctly using `ros2 node list`.
- [ ] **I can integrate** Python AI decision logic (threshold-based, neural network, or other) as a ROS 2 node that subscribes to sensors and publishes commands.
- [ ] **I can explain** the pattern for bridging AI "brains" to robotic "bodies": Sensor input → AI callback → Model inference → Safety checks → Command output.
- [ ] **I can modify** launch file parameters and namespaces to configure multi-robot systems without changing node code.

**Next Chapter**: With AI integration skills in place, Chapter 5 explores URDF (Unified Robot Description Format) - how to describe a humanoid robot's physical structure (links, joints, coordinate frames) for simulation and control.
