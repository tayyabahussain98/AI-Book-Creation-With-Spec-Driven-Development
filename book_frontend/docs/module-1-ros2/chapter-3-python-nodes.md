---
sidebar_position: 3
title: Chapter 3 - Building ROS 2 Nodes with Python
---

# Chapter 3: Building ROS 2 Nodes with Python

## Introduction

In Chapters 1 and 2, you learned **why** robots need middleware (ROS 2) and **how** ROS 2's architecture works (nodes, topics, services, actions, DDS). This chapter transitions from theory to practice: you'll write your first ROS 2 nodes in Python using the `rclpy` library.

By the end of this chapter, you'll be able to create publisher nodes that send data on topics, subscriber nodes that receive and process messages, configure nodes with parameters, and debug using ROS 2's logging system. These are the foundational skills for all ROS 2 development - every complex robot system builds on these patterns.

**Prerequisites**:
- Chapter 1 (Physical AI concepts, ROS 2 role)
- Chapter 2 (Nodes, topics, publishers, subscribers)
- ROS 2 Humble or Iron installed on Ubuntu 22.04 (or compatible Linux distribution)
- Python 3.10+ programming experience

**What You'll Build**: Three working Python nodes (publisher, subscriber, parameter logger) that communicate via ROS 2 topics.

---

## Core Concept 1: rclpy Fundamentals

### What is rclpy?

**rclpy** (ROS Client Library for Python) is the Python API for ROS 2. It provides Python classes and functions to:

- Create and manage nodes (`rclpy.node.Node`)
- Publish messages (`node.create_publisher()`)
- Subscribe to topics (`node.create_subscription()`)
- Declare and use parameters (`node.declare_parameter()`, `node.get_parameter()`)
- Log messages at different severity levels (`node.get_logger().info()`)
- Manage node lifecycles and executors (`rclpy.spin()`)

rclpy wraps the underlying C++ ROS 2 client library (rclcpp) and DDS layer, providing a Pythonic interface without sacrificing performance for most use cases.

### Basic Node Structure

Every ROS 2 Python node follows this pattern:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization: create publishers, subscribers, timers, etc.
        self.get_logger().info('Node started')

def main(args=None):
    rclpy.init(args=args)           # Initialize ROS 2 Python library
    node = MyNode()                  # Create node instance
    rclpy.spin(node)                 # Keep node running (event loop)
    node.destroy_node()              # Cleanup
    rclpy.shutdown()                 # Shutdown ROS 2

if __name__ == '__main__':
    main()
```

**Key Components**:
1. **`rclpy.init()`**: Initializes the ROS 2 context. Must be called before creating nodes.
2. **`Node` class**: Base class for all ROS 2 nodes. Inherit from it and override `__init__()`.
3. **`rclpy.spin()`**: Starts the executor that processes callbacks (subscriptions, timers, services). Blocks until interrupted (Ctrl+C).
4. **`destroy_node()` and `shutdown()`**: Cleanup resources when the node exits.

### The Node Lifecycle (Simple)

For unmanaged nodes (most common), the lifecycle is:
1. **Initialize**: `rclpy.init()` sets up ROS 2 context
2. **Create Node**: Instantiate your `Node` subclass
3. **Spin**: `rclpy.spin()` enters event loop, processing callbacks
4. **Shutdown**: User presses Ctrl+C → `spin()` exits → cleanup code runs

**Note**: Managed (lifecycle) nodes have explicit states (Unconfigured, Inactive, Active, Finalized) as discussed in Chapter 2. We'll use simple unmanaged nodes in this chapter.

---

## Core Concept 2: Publisher Pattern

### What is a Publisher?

A **publisher** is a ROS 2 component that sends messages to a topic. Publishers:
- Create messages of a specific type (e.g., `std_msgs/Float32`, `sensor_msgs/Image`)
- Publish messages at a desired frequency (e.g., 10 Hz for sensor data)
- Don't wait for subscribers - fire-and-forget pattern

### Creating a Publisher

To create a publisher in rclpy:

```python
self.publisher_ = self.create_publisher(
    MessageType,    # e.g., Float32, String, Image
    'topic_name',   # e.g., '/sensor_data'
    queue_size      # e.g., 10 (buffer last 10 messages)
)
```

**Parameters**:
- **`MessageType`**: The ROS 2 message type (import from `std_msgs.msg`, `sensor_msgs.msg`, etc.)
- **`topic_name`**: String name of the topic (should start with `/` for global topics)
- **`queue_size`**: QoS history depth - number of messages to buffer if subscribers are slow

### Publishing Messages

Once the publisher is created, send messages with:

```python
msg = MessageType()
msg.field = value
self.publisher_.publish(msg)
```

### Timer-Based Publishing

For periodic publishing (e.g., sensor data at 10 Hz), use timers:

```python
timer_period = 0.1  # seconds (10 Hz = 1/10 = 0.1s)
self.timer = self.create_timer(timer_period, self.timer_callback)

def timer_callback(self):
    msg = Float32()
    msg.data = self.sensor_value
    self.publisher_.publish(msg)
```

The `create_timer()` method creates a timer that calls `timer_callback()` every 0.1 seconds (10 times per second).

---

## Hands-On Example 1: Publisher Node

Let's create a publisher node that sends Float32 messages at 10 Hz.

**File**: `publisher_example.py`

```python
#!/usr/bin/env python3
"""
ROS 2 Publisher Example - Basic Publisher Node

This example demonstrates how to create a ROS 2 node that publishes
Float32 messages to a topic at 10 Hz (10 times per second).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SimplePublisher(Node):
    """A basic ROS 2 publisher node that sends Float32 messages."""

    def __init__(self):
        super().__init__('simple_publisher')

        # Create a publisher for Float32 messages on the 'sensor_data' topic
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)

        # Create a timer that calls timer_callback every 0.1 seconds (10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.counter = 0.0
        self.get_logger().info('Simple Publisher node started! Publishing to /sensor_data at 10 Hz')

    def timer_callback(self):
        """Called every 0.1 seconds to publish a message."""
        msg = Float32()
        msg.data = self.counter

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data:.2f}')

        self.counter += 1.0


def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()

    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        simple_publisher.get_logger().info('Publisher interrupted. Shutting down...')
    finally:
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Publisher

```bash
# Terminal 1: Run the publisher
python3 publisher_example.py

# Expected Output:
# [INFO] [simple_publisher]: Simple Publisher node started! Publishing to /sensor_data at 10 Hz
# [INFO] [simple_publisher]: Publishing: 0.00
# [INFO] [simple_publisher]: Publishing: 1.00
# [INFO] [simple_publisher]: Publishing: 2.00
# ... (continues every 100ms)
```

### Verifying Messages with ROS 2 Tools

While the publisher is running, open a new terminal and use ROS 2 command-line tools:

```bash
# List all active topics
ros2 topic list
# Output: /sensor_data, /rosout, /parameter_events

# Show details about the topic
ros2 topic info /sensor_data
# Output: Type: std_msgs/msg/Float32, Publishers: 1, Subscribers: 0

# Echo messages in real-time
ros2 topic echo /sensor_data
# Output: data: 0.0
#         ---
#         data: 1.0
#         ---
#         ... (live stream)
```

### Code Walkthrough

1. **Line 14**: Inherit from `Node` base class
2. **Line 18**: Call `super().__init__('simple_publisher')` to initialize node with name
3. **Line 21**: Create publisher for Float32 messages on `/sensor_data` topic (queue size 10)
4. **Line 25**: Create timer with 0.1s period (10 Hz frequency)
5. **Line 28**: Initialize counter to track message number
6. **Line 32-39**: Timer callback publishes Float32 message with counter value, then increments
7. **Line 47**: `rclpy.spin()` enters event loop - timer callback will be invoked every 0.1s

---

## Core Concept 3: Subscriber Pattern

### What is a Subscriber?

A **subscriber** is a ROS 2 component that receives messages from a topic. Subscribers:
- Register a callback function that runs whenever a message arrives
- Process messages asynchronously (non-blocking)
- Can have multiple subscribers on the same topic (all receive messages)

### Creating a Subscriber

To create a subscriber in rclpy:

```python
self.subscription = self.create_subscription(
    MessageType,        # e.g., Float32, String, Image
    'topic_name',       # e.g., '/sensor_data'
    self.callback,      # Function to call when message arrives
    queue_size          # e.g., 10 (buffer last 10 messages)
)
```

**Parameters**:
- **`MessageType`**: Must match the publisher's message type
- **`topic_name`**: Must match the publisher's topic name
- **`callback`**: Function with signature `callback(self, msg)` where `msg` is the received message
- **`queue_size`**: QoS history depth

### Callback Function

The callback is invoked automatically by the ROS 2 executor when a message arrives:

```python
def callback(self, msg):
    # Process the received message
    self.get_logger().info(f'Received: {msg.data}')
```

**Key Point**: Callbacks should be fast (< 10ms for real-time systems). Long computations should be offloaded to separate threads or action servers.

---

## Hands-On Example 2: Subscriber Node

Let's create a subscriber node that receives Float32 messages and logs them.

**File**: `subscriber_example.py`

```python
#!/usr/bin/env python3
"""
ROS 2 Subscriber Example - Basic Subscriber Node

This example demonstrates how to create a ROS 2 node that subscribes
to Float32 messages from a topic and processes them via a callback.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SimpleSubscriber(Node):
    """A basic ROS 2 subscriber node that receives Float32 messages."""

    def __init__(self):
        super().__init__('simple_subscriber')

        # Create a subscription to Float32 messages on the 'sensor_data' topic
        self.subscription = self.create_subscription(
            Float32,
            'sensor_data',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('Simple Subscriber node started! Listening to /sensor_data')

    def listener_callback(self, msg):
        """Called automatically when a message is received."""
        self.get_logger().info(f'I heard: {msg.data:.2f}')


def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()

    try:
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        simple_subscriber.get_logger().info('Subscriber interrupted. Shutting down...')
    finally:
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running Publisher and Subscriber Together

```bash
# Terminal 1: Run the publisher
python3 publisher_example.py
# Output: [INFO] Publishing: 0.00, 1.00, 2.00, ...

# Terminal 2: Run the subscriber
python3 subscriber_example.py
# Output: [INFO] I heard: 0.00
#         [INFO] I heard: 1.00
#         [INFO] I heard: 2.00
#         ... (continues)
```

### Code Walkthrough

1. **Line 21-26**: Create subscription to `/sensor_data` topic with `listener_callback` as the handler
2. **Line 31-32**: Callback receives `msg` parameter (Float32 message) and logs the `data` field
3. **Line 42**: `rclpy.spin()` keeps node alive and invokes `listener_callback` whenever a message arrives

**Key Insight**: The subscriber and publisher run independently. You can start/stop either one without affecting the other. If the publisher starts first, the subscriber will receive messages as soon as it connects. If the subscriber starts first, it waits until messages arrive.

---

## Core Concept 4: Parameters and Configuration

### What are Parameters?

**Parameters** are configuration values that nodes can declare, read, and modify at runtime. Parameters enable:
- **Configuration without code changes**: Set robot speed, PID gains, sensor offsets via command-line or launch files
- **Runtime reconfiguration**: Change parameters while the node is running (e.g., tune controller gains during testing)
- **Default values**: Provide sensible defaults that users can override

### Declaring Parameters

Declare parameters in `__init__()` with default values:

```python
self.declare_parameter('robot_name', 'MyRobot')      # String
self.declare_parameter('max_speed', 2.0)              # Float
self.declare_parameter('enable_logging', True)        # Boolean
```

### Reading Parameter Values

Get parameter values:

```python
robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
enable_logging = self.get_parameter('enable_logging').get_parameter_value().bool_value
```

### Overriding Parameters via Command-Line

```bash
# Override robot_name parameter
python3 my_node.py --ros-args -p robot_name:="Atlas"

# Override multiple parameters
python3 my_node.py --ros-args -p robot_name:="Spot" -p max_speed:=3.5
```

### Parameter Types

ROS 2 supports:
- `string_value` (string)
- `integer_value` (int64)
- `double_value` (float64)
- `bool_value` (boolean)
- `string_array_value`, `integer_array_value`, etc. (arrays)

---

## Core Concept 5: Logging and Debugging

### Logging Levels

ROS 2 provides five logging levels (in order of severity):

| Level | Method | Use Case |
|-------|--------|----------|
| **DEBUG** | `self.get_logger().debug('...')` | Detailed debug information (disabled by default) |
| **INFO** | `self.get_logger().info('...')` | General informational messages |
| **WARN** | `self.get_logger().warn('...')` | Warnings for non-critical issues |
| **ERROR** | `self.get_logger().error('...')` | Errors that don't stop execution |
| **FATAL** | `self.get_logger().fatal('...')` | Critical errors that may crash the node |

### Setting Log Level

```bash
# Run node with DEBUG level (shows all messages)
python3 my_node.py --ros-args --log-level DEBUG

# Run with WARN level (only warnings, errors, fatal)
python3 my_node.py --ros-args --log-level WARN
```

### Common ROS 2 Debugging Commands

```bash
# List all active nodes
ros2 node list

# Get info about a specific node
ros2 node info /my_node

# List all topics
ros2 topic list

# Echo messages on a topic (real-time stream)
ros2 topic echo /sensor_data

# Show topic publication rate
ros2 topic hz /sensor_data

# List node parameters
ros2 param list

# Get a parameter value
ros2 param get /my_node robot_name

# Set a parameter at runtime
ros2 param set /my_node robot_name "NewName"
```

---

## Hands-On Example 3: Parameters and Logging

Let's create a node that uses parameters for configuration and demonstrates all logging levels.

**File**: `param_logger_example.py`

```python
#!/usr/bin/env python3
"""
ROS 2 Parameters and Logging Example

This example demonstrates how to declare parameters and use logging levels.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParamLoggerNode(Node):
    """A node demonstrating parameters and logging levels."""

    def __init__(self):
        super().__init__('param_logger_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'MyRobot')
        self.declare_parameter('update_rate', 2.0)  # Hz

        # Read parameter values
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # Log startup info
        self.get_logger().info('='*50)
        self.get_logger().info(f'Node: {self.get_name()} started')
        self.get_logger().info(f'Robot Name: {self.robot_name}')
        self.get_logger().info(f'Update Rate: {self.update_rate} Hz')
        self.get_logger().info('='*50)

        # Create publisher and timer
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Timer callback demonstrating different logging levels."""
        self.counter += 1

        msg = String()
        msg.data = f'{self.robot_name} - Cycle {self.counter}'
        self.publisher_.publish(msg)

        # Demonstrate different logging levels
        if self.counter % 10 == 0:
            self.get_logger().info(f'Status update: {msg.data}')
        elif self.counter % 7 == 0:
            self.get_logger().warn(f'Warning: High cycle count: {self.counter}')
        elif self.counter % 15 == 0:
            self.get_logger().error(f'Error: Simulated error at cycle {self.counter}')
        else:
            self.get_logger().debug(f'Debug: Processing cycle {self.counter}')


def main(args=None):
    rclpy.init(args=args)
    param_logger_node = ParamLoggerNode()

    try:
        rclpy.spin(param_logger_node)
    except KeyboardInterrupt:
        param_logger_node.get_logger().info('Node interrupted. Shutting down...')
    finally:
        param_logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running with Parameter Overrides

```bash
# Run with default parameters
python3 param_logger_example.py
# Output: Robot Name: MyRobot, Update Rate: 2.0 Hz

# Override robot_name
python3 param_logger_example.py --ros-args -p robot_name:="Atlas"
# Output: Robot Name: Atlas, Update Rate: 2.0 Hz

# Override both parameters
python3 param_logger_example.py --ros-args -p robot_name:="Optimus" -p update_rate:=5.0
# Output: Robot Name: Optimus, Update Rate: 5.0 Hz

# Run with DEBUG log level
python3 param_logger_example.py --ros-args --log-level DEBUG
# Output: Shows debug messages every cycle
```

### Checking Parameters at Runtime

```bash
# Terminal 1: Run the node
python3 param_logger_example.py

# Terminal 2: Query parameters
ros2 param list
# Output: /param_logger_node:
#   robot_name
#   update_rate

ros2 param get /param_logger_node robot_name
# Output: String value is: MyRobot

# Change parameter at runtime
ros2 param set /param_logger_node robot_name "DynamicName"
# Output: Set parameter successful
```

---

## Diagrams

### Publisher-Subscriber Interaction

See the [Pub-Sub Interaction Diagram](/docs/diagrams/module-1/pubsub-interaction.md) for the complete message flow visualization:

**Key Points**:
- **Publisher Node** creates messages and publishes to `/sensor_data` topic at 10 Hz
- **DDS Middleware** routes messages from publisher to all subscribers
- **Subscriber Node** receives messages via callback function
- **Decoupling**: Publisher and subscriber don't know about each other directly - they only know the topic name

**Timing Example**:
- Time 0ms: Publisher publishes Float32(0.0)
- Time 0.5ms: DDS routes message to subscriber
- Time 1ms: Subscriber's callback invoked with Float32(0.0)
- Time 100ms: Publisher publishes Float32(1.0)
- Time 100.5ms: DDS routes message
- Time 101ms: Subscriber's callback invoked with Float32(1.0)

---

## Summary

This chapter introduced hands-on ROS 2 Python development with rclpy:

1. **rclpy Fundamentals**: ROS 2 Python nodes follow a standard pattern: `rclpy.init()` → create `Node` subclass → `rclpy.spin()` → cleanup. The executor handles callbacks asynchronously.

2. **Publisher Pattern**: Create publishers with `create_publisher(MessageType, topic_name, queue_size)`. Use timers (`create_timer()`) for periodic publishing. Messages are fire-and-forget (no acknowledgment).

3. **Subscriber Pattern**: Create subscribers with `create_subscription(MessageType, topic_name, callback, queue_size)`. Callbacks are invoked automatically when messages arrive. Keep callbacks fast (< 10ms).

4. **Parameters**: Declare parameters with `declare_parameter()` for runtime configuration. Override via command-line (`--ros-args -p param:=value`) or launch files. Read with `get_parameter()`.

5. **Logging**: Use five logging levels (DEBUG, INFO, WARN, ERROR, FATAL) via `get_logger()`. Set log level with `--ros-args --log-level LEVEL`. Use ROS 2 CLI tools (`ros2 topic echo`, `ros2 param list`, `ros2 node info`) for debugging.

6. **Decoupling**: Publishers and subscribers communicate via topics without direct dependencies. This enables distributed systems, fault isolation, and flexible component composition.

---

## Self-Assessment Checklist

After completing this chapter, you should be able to:

- [ ] **I can create** a basic ROS 2 Python node by inheriting from `rclpy.node.Node` and implementing `__init__()`.
- [ ] **I can write** a publisher node that sends messages to a topic at a specified frequency using timers.
- [ ] **I can write** a subscriber node that receives messages from a topic and processes them in a callback function.
- [ ] **I can run** both publisher and subscriber nodes simultaneously and verify message flow using `ros2 topic echo`.
- [ ] **I can declare** parameters with default values and override them via command-line arguments (`--ros-args -p param:=value`).
- [ ] **I can use** different logging levels (INFO, WARN, ERROR) to output messages at appropriate severity levels.

**Next Chapter**: With foundational Python node skills in place, Chapter 4 explores how to organize nodes into packages, use launch files to start multiple nodes simultaneously, and integrate AI decision logic as ROS 2 nodes.
