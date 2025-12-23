#!/usr/bin/env python3
"""
ROS 2 Publisher Example - Basic Publisher Node

This example demonstrates how to create a ROS 2 node that publishes
Float32 messages to a topic at 10 Hz (10 times per second).

Requirements:
- ROS 2 Humble or Iron installed
- rclpy package (part of ROS 2 installation)

Usage:
    ros2 run <package_name> publisher_example
    # Or directly:
    python3 publisher_example.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SimplePublisher(Node):
    """
    A basic ROS 2 publisher node that sends Float32 messages.

    This node publishes incrementing counter values to demonstrate
    periodic message publishing. In a real robot system, this could
    be sensor data (e.g., temperature, distance, joint angle).
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('simple_publisher')

        # Create a publisher for Float32 messages on the 'sensor_data' topic
        # Queue size of 10 means the last 10 messages are buffered if subscribers are slow
        self.publisher_ = self.create_publisher(Float32, 'sensor_data', 10)

        # Create a timer that calls timer_callback every 0.1 seconds (10 Hz)
        timer_period = 0.1  # seconds (10 Hz = 1/0.1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track published messages
        self.counter = 0.0

        self.get_logger().info('Simple Publisher node started! Publishing to /sensor_data at 10 Hz')

    def timer_callback(self):
        """
        Callback function invoked every 0.1 seconds by the timer.

        This method creates a message, populates it with data, and publishes
        it to the 'sensor_data' topic.
        """
        # Create a new Float32 message
        msg = Float32()
        msg.data = self.counter

        # Publish the message to the topic
        self.publisher_.publish(msg)

        # Log the published value (visible in terminal)
        self.get_logger().info(f'Publishing: {msg.data:.2f}')

        # Increment counter for next message
        self.counter += 1.0


def main(args=None):
    """
    Main function to initialize ROS 2 and run the publisher node.

    This function:
    1. Initializes the ROS 2 Python client library (rclpy)
    2. Creates the publisher node
    3. Spins the node (keeps it running and processing callbacks)
    4. Cleans up when interrupted (Ctrl+C)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the publisher node
    simple_publisher = SimplePublisher()

    try:
        # Spin the node - this blocks and processes callbacks until interrupted
        # The timer callback will be invoked every 0.1 seconds
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        simple_publisher.get_logger().info('Publisher interrupted by user. Shutting down...')
    finally:
        # Cleanup: destroy the node and shutdown rclpy
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# Expected Output (when running):
# [INFO] [<timestamp>] [simple_publisher]: Simple Publisher node started! Publishing to /sensor_data at 10 Hz
# [INFO] [<timestamp>] [simple_publisher]: Publishing: 0.00
# [INFO] [<timestamp>] [simple_publisher]: Publishing: 1.00
# [INFO] [<timestamp>] [simple_publisher]: Publishing: 2.00
# ...
# (continues every 100ms until Ctrl+C)
#
# To verify messages are being published:
# Open a new terminal and run:
#   ros2 topic echo /sensor_data
# This will display all messages published to the topic in real-time.
