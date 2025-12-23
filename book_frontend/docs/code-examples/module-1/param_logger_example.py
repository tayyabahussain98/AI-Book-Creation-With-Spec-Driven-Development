#!/usr/bin/env python3
"""
ROS 2 Parameters and Logging Example

This example demonstrates how to:
1. Declare and use ROS 2 parameters (configuration values)
2. Use different logging levels (DEBUG, INFO, WARN, ERROR, FATAL)
3. Override parameters via command-line arguments

Requirements:
- ROS 2 Humble or Iron installed
- rclpy package (part of ROS 2 installation)

Usage:
    # Run with default parameters
    python3 param_logger_example.py

    # Override parameters via command-line
    python3 param_logger_example.py --ros-args -p update_rate:=5.0 -p robot_name:="Optimus"

    # Set log level to DEBUG to see all messages
    python3 param_logger_example.py --ros-args --log-level DEBUG
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParamLoggerNode(Node):
    """
    A ROS 2 node demonstrating parameter management and logging levels.

    This node shows how to:
    - Declare parameters with default values and types
    - Read parameter values at runtime
    - Use different log levels for different message severities
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('param_logger_node')

        # Declare parameters with default values
        # Parameters can be overridden via command-line or launch files
        self.declare_parameter('robot_name', 'MyRobot')  # String parameter
        self.declare_parameter('update_rate', 2.0)       # Float parameter (Hz)
        self.declare_parameter('enable_debug', False)    # Boolean parameter

        # Read parameter values
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.enable_debug = self.get_parameter('enable_debug').get_parameter_value().bool_value

        # Log startup information at INFO level
        self.get_logger().info('='*50)
        self.get_logger().info(f'Node: {self.get_name()} started')
        self.get_logger().info(f'Robot Name: {self.robot_name}')
        self.get_logger().info(f'Update Rate: {self.update_rate} Hz')
        self.get_logger().info(f'Debug Mode: {self.enable_debug}')
        self.get_logger().info('='*50)

        # Create a publisher (for demonstration)
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)

        # Create a timer based on the update_rate parameter
        timer_period = 1.0 / self.update_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.counter = 0

    def timer_callback(self):
        """
        Timer callback demonstrating different logging levels.

        Logging Levels (in order of severity):
        - DEBUG: Detailed information for debugging (typically disabled in production)
        - INFO: General informational messages
        - WARN: Warning messages for potentially problematic situations
        - ERROR: Error messages for serious issues that don't stop execution
        - FATAL: Critical errors that may cause node to crash
        """
        self.counter += 1

        # Create and publish a status message
        msg = String()
        msg.data = f'{self.robot_name} - Cycle {self.counter}'
        self.publisher_.publish(msg)

        # Demonstrate different logging levels based on counter value
        if self.counter % 10 == 0:
            # Every 10th cycle: INFO level
            self.get_logger().info(f'Status update: {msg.data}')

        elif self.counter % 7 == 0:
            # Every 7th cycle: WARN level
            self.get_logger().warn(f'Warning: High cycle count detected: {self.counter}')

        elif self.counter % 15 == 0:
            # Every 15th cycle: ERROR level (simulate error condition)
            self.get_logger().error(f'Error: Simulated error at cycle {self.counter}')

        else:
            # All other cycles: DEBUG level
            # NOTE: DEBUG messages only appear if log level is set to DEBUG
            # Run with: --ros-args --log-level DEBUG
            if self.enable_debug:
                self.get_logger().debug(f'Debug: Processing cycle {self.counter}')

        # Demonstrate conditional logging based on parameters
        if self.counter == 5:
            self.get_logger().info(f'Reached cycle 5! Robot: {self.robot_name}')

        # Simulate a critical error (uncomment to test FATAL level)
        # if self.counter == 50:
        #     self.get_logger().fatal('FATAL: Critical system failure! Shutting down...')
        #     rclpy.shutdown()


def main(args=None):
    """
    Main function to initialize ROS 2 and run the parameter/logger node.

    Command-line parameter override examples:
    1. Change robot name:
       python3 param_logger_example.py --ros-args -p robot_name:="Atlas"

    2. Change update rate to 5 Hz:
       python3 param_logger_example.py --ros-args -p update_rate:=5.0

    3. Enable debug mode:
       python3 param_logger_example.py --ros-args -p enable_debug:=true

    4. Combine multiple parameters:
       python3 param_logger_example.py --ros-args -p robot_name:="Spot" -p update_rate:=1.0

    5. Set log level to DEBUG (to see debug messages):
       python3 param_logger_example.py --ros-args --log-level DEBUG
    """
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)

    # Create the parameter/logger node
    param_logger_node = ParamLoggerNode()

    try:
        # Spin the node - this blocks and processes callbacks until interrupted
        rclpy.spin(param_logger_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        param_logger_node.get_logger().info('Node interrupted by user. Shutting down...')
    finally:
        # Cleanup: destroy the node and shutdown rclpy
        param_logger_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# Expected Output (default parameters, INFO log level):
# [INFO] [<timestamp>] [param_logger_node]: ==================================================
# [INFO] [<timestamp>] [param_logger_node]: Node: param_logger_node started
# [INFO] [<timestamp>] [param_logger_node]: Robot Name: MyRobot
# [INFO] [<timestamp>] [param_logger_node]: Update Rate: 2.0 Hz
# [INFO] [<timestamp>] [param_logger_node]: Debug Mode: False
# [INFO] [<timestamp>] [param_logger_node]: ==================================================
# [INFO] [<timestamp>] [param_logger_node]: Reached cycle 5! Robot: MyRobot
# [WARN] [<timestamp>] [param_logger_node]: Warning: High cycle count detected: 7
# [INFO] [<timestamp>] [param_logger_node]: Status update: MyRobot - Cycle 10
# [WARN] [<timestamp>] [param_logger_node]: Warning: High cycle count detected: 14
# [ERROR] [<timestamp>] [param_logger_node]: Error: Simulated error at cycle 15
# ...
#
# To check parameters while the node is running (in another terminal):
#   ros2 param list
#   ros2 param get /param_logger_node robot_name
#   ros2 param set /param_logger_node robot_name "NewName"
