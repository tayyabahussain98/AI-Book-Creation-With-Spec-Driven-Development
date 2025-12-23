#!/usr/bin/env python3
"""
ROS 2 Subscriber Example - Basic Subscriber Node

This example demonstrates how to create a ROS 2 node that subscribes
to Float32 messages from a topic and processes them via a callback.

Requirements:
- ROS 2 Humble or Iron installed
- rclpy package (part of ROS 2 installation)

Usage:
    # Terminal 1: Start the publisher
    python3 publisher_example.py

    # Terminal 2: Start this subscriber
    python3 subscriber_example.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SimpleSubscriber(Node):
    """
    A basic ROS 2 subscriber node that receives Float32 messages.

    This node listens to the 'sensor_data' topic and logs received values.
    In a real robot system, the callback would process sensor data and
    trigger control actions (e.g., adjust motor speed based on distance sensor).
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('simple_subscriber')

        # Create a subscription to Float32 messages on the 'sensor_data' topic
        # The callback function 'listener_callback' is invoked whenever a message arrives
        # Queue size of 10 means the node buffers up to 10 unprocessed messages
        self.subscription = self.create_subscription(
            Float32,                    # Message type
            'sensor_data',              # Topic name (must match publisher)
            self.listener_callback,     # Callback function
            10                          # Queue size (QoS history depth)
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('Simple Subscriber node started! Listening to /sensor_data')

    def listener_callback(self, msg):
        """
        Callback function invoked when a message is received on the subscribed topic.

        Args:
            msg (Float32): The received message containing a float value

        This method is called automatically by the ROS 2 executor whenever
        a new message arrives on the 'sensor_data' topic.
        """
        # Log the received value
        self.get_logger().info(f'I heard: {msg.data:.2f}')

        # In a real system, you would process the data here:
        # - Update robot state
        # - Trigger control actions
        # - Store data for analysis
        # Example:
        # if msg.data > 50.0:
        #     self.get_logger().warn('Sensor value exceeded threshold!')


def main(args=None):
    """
    Main function to initialize ROS 2 and run the subscriber node.

    This function:
    1. Initializes the ROS 2 Python client library (rclpy)
    2. Creates the subscriber node
    3. Spins the node (keeps it running and processing callbacks)
    4. Cleans up when interrupted (Ctrl+C)
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the subscriber node
    simple_subscriber = SimpleSubscriber()

    try:
        # Spin the node - this blocks and processes callbacks until interrupted
        # Each time a message arrives on 'sensor_data', listener_callback is invoked
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        simple_subscriber.get_logger().info('Subscriber interrupted by user. Shutting down...')
    finally:
        # Cleanup: destroy the node and shutdown rclpy
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# Expected Output (when publisher is running):
# [INFO] [<timestamp>] [simple_subscriber]: Simple Subscriber node started! Listening to /sensor_data
# [INFO] [<timestamp>] [simple_subscriber]: I heard: 0.00
# [INFO] [<timestamp>] [simple_subscriber]: I heard: 1.00
# [INFO] [<timestamp>] [simple_subscriber]: I heard: 2.00
# ...
# (continues receiving messages every 100ms until Ctrl+C)
#
# If the publisher is NOT running, you'll only see:
# [INFO] [<timestamp>] [simple_subscriber]: Simple Subscriber node started! Listening to /sensor_data
# (and no "I heard" messages until the publisher starts)
#
# This demonstrates ROS 2's decoupling: subscriber can start before publisher,
# and they automatically connect when both are running.
