# AI-ROS Integration Diagram Specification

**Purpose**: Illustrate how Python-based AI decision logic integrates with ROS 2 as a node

**Diagram Type**: Component diagram / Data flow diagram

## System Components

### 1. Sensor Input (Left Side)
- **Mock Sensor Node** or **Real Sensor Driver**
- **Publishes**: `/sensor_data` topic (Float32, sensor_msgs/Image, etc.)
- **Frequency**: 10-30 Hz (depending on sensor type)
- **Examples**: Distance sensor, camera, IMU, joint encoders

### 2. AI Agent Node (Center)
- **Type**: ROS 2 Node wrapping AI decision logic
- **Subscribes to**: `/sensor_data` (input from sensors)
- **Publishes to**: `/motor_commands` (output to controllers)
- **Also publishes**: `/ai_status` (debugging/monitoring)
- **Internal Components**:
  - **Sensor Callback**: Receives sensor data, preprocesses
  - **AI Decision Logic**: Model inference or algorithm (e.g., neural network, policy, planner)
  - **Safety Layer**: Clamps commands to safe limits
  - **Command Publisher**: Sends motor commands

### 3. Motor Controller (Right Side)
- **Type**: ROS 2 Node (often C++ for real-time performance)
- **Subscribes to**: `/motor_commands` (from AI agent)
- **Controls**: Physical motors or simulation actuators
- **Publishes**: `/joint_states` (feedback to AI agent)
- **Frequency**: 100-1000 Hz (high-frequency control loop)

### 4. Feedback Loop (Bottom)
- **Motor Encoders** publish current state to `/joint_states`
- **AI Agent** subscribes to `/joint_states` for closed-loop control
- Enables error correction: "Did motor reach target position?"

## Data Flow Sequence

### Perception → Decision → Action → Feedback

```
┌─────────────┐         ┌──────────────┐         ┌─────────────┐
│   Sensor    │─publish→│ /sensor_data │─sub────→│  AI Agent   │
│   Node      │         │   (topic)    │         │    Node     │
└─────────────┘         └──────────────┘         └──────┬──────┘
                                                         │
                                                    [AI Logic]
                                                   - Preprocess
                                                   - Inference
                                                   - Post-process
                                                         │
                                                         ▼
                                         ┌──────────────────────┐
                                         │  /motor_commands     │
                                         │     (topic)          │
                                         └──────────┬───────────┘
                                                    │
                                              sub   │
                                                    ▼
                                         ┌──────────────────────┐
                                         │  Motor Controller    │
                                         │      Node            │
                                         └──────────┬───────────┘
                                                    │
                                              pub   │
                                                    ▼
                                         ┌──────────────────────┐
                                         │   /joint_states      │
                                         │     (feedback)       │
                                         └──────────┬───────────┘
                                                    │
                                              sub   │
                                                    │
                                  ┌─────────────────┘
                                  │
                                  ▼
                           [Back to AI Agent]
                          (Closed-loop control)
```

## AI Decision Logic Integration Patterns

### Pattern 1: Simple Threshold-Based Decision

```python
def sensor_callback(self, msg):
    sensor_value = msg.data
    if sensor_value > self.threshold:
        motor_command = self.max_speed  # Move forward
    else:
        motor_command = 0.0              # Stop
    self.command_publisher.publish(Float32(data=motor_command))
```

**Use Case**: Simple reactive behaviors (obstacle avoidance, line following)

### Pattern 2: Neural Network Inference

```python
import torch

def sensor_callback(self, msg):
    # Preprocess sensor data
    input_tensor = self.preprocess(msg.data)

    # Run neural network inference
    with torch.no_grad():
        output = self.model(input_tensor)

    # Post-process to motor commands
    motor_command = self.postprocess(output)

    # Publish
    self.command_publisher.publish(Float32(data=motor_command))
```

**Use Case**: Learned policies (reinforcement learning for locomotion, imitation learning for manipulation)

### Pattern 3: Vision-Based Decision

```python
from cv_bridge import CvBridge
from ultralytics import YOLO

def image_callback(self, msg):
    # Convert ROS Image to OpenCV format
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Run object detection
    results = self.vision_model(cv_image)

    # Extract target object position
    if len(results[0].boxes) > 0:
        box = results[0].boxes[0]  # First detected object
        x, y = box.xywh[0][:2].tolist()

        # Compute arm target position
        target = Point(x=x, y=y, z=0.5)
        self.target_publisher.publish(target)
```

**Use Case**: Vision-guided manipulation (picking objects, grasping)

### Pattern 4: LLM-Based Task Planning

```python
import openai

def task_callback(self, msg):
    # Receive high-level task from user
    task_description = msg.data  # e.g., "Pick up the red cup"

    # Query LLM for step-by-step plan
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": f"Generate robot plan: {task_description}"}]
    )
    plan = response.choices[0].message.content

    # Parse plan into robot actions
    actions = self.parse_plan(plan)

    # Publish actions to robot controller
    for action in actions:
        self.action_publisher.publish(action)
```

**Use Case**: High-level task planning (natural language robot control)

## Timing and Performance

### Typical Processing Pipeline

| Stage | Time | Frequency | Component |
|-------|------|-----------|-----------|
| **Sensor Capture** | 1-5 ms | 30 Hz | Camera/sensor hardware |
| **Sensor Publish** | 0.1 ms | 30 Hz | Sensor node |
| **DDS Transport** | 1-5 ms | 30 Hz | Middleware |
| **AI Preprocessing** | 5-20 ms | 30 Hz | AI agent node |
| **Model Inference** | 10-100 ms | 10-30 Hz | AI agent (GPU) |
| **Post-processing** | 1-5 ms | 10-30 Hz | AI agent |
| **Command Publish** | 0.1 ms | 10-30 Hz | AI agent |
| **Motor Control** | 1-10 ms | 100-1000 Hz | Motor controller |

**Total Latency**: Sensor capture → Motor execution: ~50-200 ms (depending on AI model complexity)

### Performance Considerations

1. **Heavy AI models (greater than 100ms inference)**:
   - Run on separate thread/process to avoid blocking ROS callbacks
   - Use GPU acceleration (CUDA, TensorRT)
   - Consider model quantization or pruning

2. **Real-time requirements (less than 10ms)**:
   - Use lightweight models or rule-based logic
   - Offload to C++ nodes for critical paths
   - Use real-time executors

3. **Multi-sensor fusion**:
   - Synchronize sensor topics with `message_filters`
   - Handle different frequencies (camera 30Hz, IMU 200Hz)

## Safety and Error Handling

### Safety Checks in AI Agent

```python
def apply_safety_limits(self, command):
    # Check 1: Clamp to max speed
    if abs(command) > self.max_speed:
        self.get_logger().warn('Command exceeds max speed, clamping')
        return math.copysign(self.max_speed, command)

    # Check 2: Verify sensor freshness
    if (self.get_clock().now() - self.last_sensor_time).nanoseconds > 1e9:  # 1 second
        self.get_logger().error('Sensor timeout, stopping motor')
        return 0.0

    # Check 3: E-stop signal
    if self.emergency_stop:
        self.get_logger().fatal('Emergency stop active!')
        return 0.0

    return command
```

### Failure Modes

**Scenario 1: AI Agent Crashes**
- **Result**: Motor controller stops receiving commands, enters safe state (stops motors)
- **Recovery**: Restart AI agent node, re-subscribes to topics

**Scenario 2: Sensor Failure**
- **Detection**: AI agent callback stops being invoked (no new sensor data)
- **Response**: AI agent times out, publishes zero command (stop)

**Scenario 3: Model Inference Error**
- **Detection**: Exception in `model.predict()`
- **Response**: Catch exception, log error, publish safe fallback command

## Visual Layout

```
┌─────────────────────────────────────────────────────────────┐
│                      AI Agent Node                          │
│  ┌──────────────┐   ┌───────────────┐   ┌──────────────┐   │
│  │  Sensor      │   │  AI Decision  │   │  Command     │   │
│  │  Callback    │──→│     Logic     │──→│  Publisher   │   │
│  │              │   │               │   │              │   │
│  │ - Receive    │   │ - Preprocess  │   │ - Safety     │   │
│  │ - Validate   │   │ - Inference   │   │ - Clamp      │   │
│  │              │   │ - Post-process│   │ - Publish    │   │
│  └──────▲───────┘   └───────────────┘   └──────┬───────┘   │
│         │                                       │           │
└─────────┼───────────────────────────────────────┼───────────┘
          │                                       │
   ┌──────┴─────────┐                   ┌────────▼────────┐
   │ /sensor_data   │                   │ /motor_commands │
   │   (input)      │                   │    (output)     │
   └────────────────┘                   └─────────────────┘
          ▲                                       │
          │                                       ▼
   ┌──────┴─────────┐                   ┌─────────────────┐
   │  Sensor Node   │                   │ Motor Controller│
   └────────────────┘                   └────────┬────────┘
                                                 │
                                          ┌──────▼─────────┐
                                          │ /joint_states  │
                                          │  (feedback)    │
                                          └────────────────┘
                                                 │
                                                 └──→ (back to AI Agent)
```

## Real-World Examples

### Example 1: Humanoid Balance Controller
- **Input**: IMU data (orientation, angular velocity)
- **AI Logic**: Neural network trained on balance task
- **Output**: Joint torque commands to maintain upright posture
- **Frequency**: 100-200 Hz (critical for stability)

### Example 2: Object Manipulation
- **Input**: RGB-D camera images
- **AI Logic**: Vision model (YOLO) + grasp pose estimator
- **Output**: Arm trajectory (6D pose targets)
- **Frequency**: 10-30 Hz (vision processing)

### Example 3: Natural Language Robot Control
- **Input**: Speech recognition text
- **AI Logic**: LLM (GPT-4) generates motion plan
- **Output**: High-level action goals
- **Frequency**: On-demand (user-triggered)

## Usage in Book

- **Referenced in**: Chapter 4 (Core Concept 4: Bridging AI Agents to ROS)
- **Purpose**: Show learners the complete pattern for integrating AI models (PyTorch, TensorFlow, YOLO, LLMs) into ROS 2 nodes
- **Learning Goal**: Learners can wrap existing AI code as ROS nodes and integrate into multi-node robot systems
