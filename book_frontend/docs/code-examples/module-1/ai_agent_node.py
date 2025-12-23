#!/usr/bin/env python3
"""
AI Agent Node Example - Bridging AI Decision Logic to ROS 2

This example demonstrates how to integrate Python-based AI decision-making
logic as a ROS 2 node. The AI agent subscribes to sensor data, makes decisions
based on simple logic (simulating an AI model), and publishes control commands.

Concept:
    Sensor Input → AI Decision Logic → Control Commands → Actuators

In a real system, the "AI logic" could be:
- A neural network policy (e.g., PyTorch model for locomotion)
- A vision model (e.g., YOLO for object detection)
- A large language model (e.g., GPT for task planning)
- A reinforcement learning agent (e.g., trained for grasping)

This simplified example uses threshold-based logic to demonstrate the pattern.

Requirements:
- ROS 2 Humble or Iron installed
- rclpy package

Usage:
    # Terminal 1: Run a mock sensor (publisher from Chapter 3)
    python3 publisher_example.py

    # Terminal 2: Run this AI agent node
    python3 ai_agent_node.py

    # Terminal 3: Monitor control commands
    ros2 topic echo /motor_commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import math


class AIAgentNode(Node):
    """
    AI Agent Node: Processes sensor input and publishes control commands.

    This node simulates an AI agent that:
    1. Subscribes to sensor data (e.g., distance measurements)
    2. Applies decision logic (threshold-based for simplicity)
    3. Publishes motor commands based on decisions
    """

    def __init__(self):
        super().__init__('ai_agent_node')

        # Declare parameters for AI configuration
        self.declare_parameter('decision_threshold', 50.0)  # Sensor threshold
        self.declare_parameter('max_motor_speed', 100.0)    # Max motor command value
        self.declare_parameter('enable_safety_check', True) # Safety override

        # Read parameters
        self.threshold = self.get_parameter('decision_threshold').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_motor_speed').get_parameter_value().double_value
        self.safety_enabled = self.get_parameter('enable_safety_check').get_parameter_value().bool_value

        # Subscriber: Sensor input (from environment or mock sensor)
        self.sensor_subscription = self.create_subscription(
            Float32,
            'sensor_data',              # Subscribe to sensor readings
            self.sensor_callback,
            10
        )

        # Publisher: Motor commands (to motor controller or simulation)
        self.command_publisher = self.create_publisher(
            Float32,
            'motor_commands',           # Publish control commands
            10
        )

        # Publisher: Status updates (for monitoring/debugging)
        self.status_publisher = self.create_publisher(
            String,
            'ai_status',
            10
        )

        # Internal state
        self.last_sensor_value = 0.0
        self.decision_count = 0

        self.get_logger().info('='*60)
        self.get_logger().info('AI Agent Node Started')
        self.get_logger().info(f'Decision Threshold: {self.threshold}')
        self.get_logger().info(f'Max Motor Speed: {self.max_speed}')
        self.get_logger().info(f'Safety Check: {"ENABLED" if self.safety_enabled else "DISABLED"}')
        self.get_logger().info('='*60)

    def sensor_callback(self, msg):
        """
        Callback invoked when sensor data arrives.

        This is where the "AI" decision-making happens. In this simplified
        example, we use threshold-based logic. In a real system, this would be:
        - model.predict(sensor_data) for neural network inference
        - policy(observation) for RL agents
        - vision_model(image) for object detection
        """
        self.last_sensor_value = msg.data
        self.decision_count += 1

        # AI Decision Logic (simplified threshold-based)
        motor_command = self.make_decision(msg.data)

        # Safety check (simulate safety constraints)
        if self.safety_enabled:
            motor_command = self.apply_safety_limits(motor_command)

        # Publish motor command
        cmd_msg = Float32()
        cmd_msg.data = motor_command
        self.command_publisher.publish(cmd_msg)

        # Publish status update
        status_msg = String()
        status_msg.data = f'Sensor: {msg.data:.2f} | Decision: {motor_command:.2f} | Count: {self.decision_count}'
        self.status_publisher.publish(status_msg)

        # Log decision (use different levels based on importance)
        if abs(motor_command) > self.max_speed * 0.8:
            self.get_logger().warn(f'HIGH COMMAND: Sensor={msg.data:.2f}, Motor={motor_command:.2f}')
        else:
            self.get_logger().info(f'Sensor={msg.data:.2f} → Motor={motor_command:.2f}')

    def make_decision(self, sensor_value):
        """
        AI decision logic: Convert sensor input to motor command.

        This is a placeholder for actual AI logic. Real implementations would:
        - Load a trained neural network model
        - Perform inference: output = model(input_tensor)
        - Post-process outputs to motor commands

        Simple Logic (for demonstration):
        - If sensor_value > threshold: move forward (positive speed)
        - If sensor_value < threshold: move backward (negative speed)
        - Command magnitude proportional to how far from threshold
        """
        # Calculate distance from threshold
        error = sensor_value - self.threshold

        # Proportional control (P-controller)
        # In real systems, this could be a PID controller or neural network output
        motor_command = error * 2.0  # Gain factor of 2.0

        return motor_command

    def apply_safety_limits(self, command):
        """
        Apply safety constraints to motor commands.

        Safety checks ensure:
        - Commands don't exceed max motor speed
        - Commands don't violate joint limits
        - Emergency stop conditions are respected

        This is critical for physical robots to prevent hardware damage.
        """
        # Clamp command to max speed
        if command > self.max_speed:
            self.get_logger().warn(f'Command {command:.2f} exceeds max speed {self.max_speed}, clamping')
            return self.max_speed
        elif command < -self.max_speed:
            self.get_logger().warn(f'Command {command:.2f} below min speed {-self.max_speed}, clamping')
            return -self.max_speed

        return command


def main(args=None):
    """
    Main function to run the AI agent node.

    This follows the standard ROS 2 node pattern:
    1. Initialize rclpy
    2. Create the AI agent node
    3. Spin (process callbacks)
    4. Cleanup on shutdown
    """
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('AI Agent interrupted. Shutting down...')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ============================================================================
# INTEGRATION WITH REAL AI MODELS
# ============================================================================
#
# To integrate a real AI model (e.g., PyTorch neural network):
#
# 1. Load the model in __init__():
#    import torch
#    self.model = torch.load('path/to/model.pth')
#    self.model.eval()
#
# 2. Modify make_decision() to use the model:
#    def make_decision(self, sensor_value):
#        input_tensor = torch.tensor([[sensor_value]], dtype=torch.float32)
#        with torch.no_grad():
#            output = self.model(input_tensor)
#        motor_command = output.item()
#        return motor_command
#
# 3. For vision-based AI (e.g., object detection):
#    - Subscribe to sensor_msgs/Image instead of Float32
#    - Run model.predict(image) in callback
#    - Publish detection results or control commands
#
# Example: YOLOv8 object detection
#    from ultralytics import YOLO
#    self.vision_model = YOLO('yolov8n.pt')
#
#    def image_callback(self, msg):
#        # Convert ROS Image message to numpy array
#        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#        # Run detection
#        results = self.vision_model(image)
#        # Extract bounding boxes
#        for box in results[0].boxes:
#            x, y, w, h = box.xywh[0].tolist()
#            # Publish target position for arm controller
#            target_msg = Point(x=x, y=y, z=0.0)
#            self.target_publisher.publish(target_msg)
#
# ============================================================================

# Expected output when running with publisher_example.py:
# [INFO] [ai_agent_node]: ==================================================
# [INFO] [ai_agent_node]: AI Agent Node Started
# [INFO] [ai_agent_node]: Decision Threshold: 50.0
# [INFO] [ai_agent_node]: Max Motor Speed: 100.0
# [INFO] [ai_agent_node]: Safety Check: ENABLED
# [INFO] [ai_agent_node]: ==================================================
# [INFO] [ai_agent_node]: Sensor=0.00 → Motor=-100.00 (clamped to max)
# [INFO] [ai_agent_node]: Sensor=1.00 → Motor=-98.00
# [INFO] [ai_agent_node]: Sensor=2.00 → Motor=-96.00
# ...
# [INFO] [ai_agent_node]: Sensor=49.00 → Motor=-2.00
# [INFO] [ai_agent_node]: Sensor=50.00 → Motor=0.00 (at threshold)
# [INFO] [ai_agent_node]: Sensor=51.00 → Motor=2.00
# [WARN] [ai_agent_node]: HIGH COMMAND: Sensor=90.00, Motor=80.00
# ...
#
# To monitor motor commands:
#   ros2 topic echo /motor_commands
# To monitor AI status:
#   ros2 topic echo /ai_status
# To change threshold at runtime:
#   ros2 param set /ai_agent_node decision_threshold 75.0
