#!/usr/bin/env python3
"""
ROS 2 Python Launch File Example

This launch file demonstrates how to start multiple ROS 2 nodes simultaneously
with parameters, remapping, and namespaces.

Requirements:
- ROS 2 Humble or Iron installed
- Publisher and subscriber example nodes from Chapter 3

Usage:
    ros2 launch <package_name> simple_launch.py
    # Or directly:
    python3 simple_launch.py

What this launch file does:
1. Starts a publisher node with custom parameters
2. Starts two subscriber nodes in different namespaces
3. Demonstrates parameter passing and topic remapping
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description with multiple nodes.

    This function is called by the ROS 2 launch system to create
    the node configuration. It returns a LaunchDescription containing
    all nodes to be launched.
    """
    return LaunchDescription([
        # Node 1: Publisher node with custom parameters
        Node(
            package='my_robot_package',  # Replace with your package name
            executable='publisher_node',  # Name of the executable (from setup.py)
            name='sensor_publisher',      # Node name (appears in ros2 node list)
            output='screen',              # Print output to terminal
            parameters=[{
                'update_rate': 5.0,       # Override default publishing rate to 5 Hz
                'sensor_id': 'lidar_front',
            }]
        ),

        # Node 2: First subscriber node (default namespace)
        Node(
            package='my_robot_package',
            executable='subscriber_node',
            name='data_logger',
            output='screen',
            # Remap the topic this subscriber listens to
            remappings=[
                ('sensor_data', 'sensors/lidar')  # Subscribe to /sensors/lidar instead of /sensor_data
            ]
        ),

        # Node 3: Second subscriber node (in a separate namespace)
        Node(
            package='my_robot_package',
            executable='subscriber_node',
            name='data_processor',
            namespace='processing',       # This node will be /processing/data_processor
            output='screen',
            parameters=[{
                'enable_filtering': True,
                'filter_threshold': 0.5,
            }]
        ),

        # Node 4: A simple timer node (optional - for demonstration)
        Node(
            package='my_robot_package',
            executable='timer_node',
            name='heartbeat',
            output='screen',
            parameters=[{
                'timer_period': 1.0,      # 1 Hz heartbeat
            }]
        ),
    ])


# Alternative: Launch with command-line arguments
# This allows overriding parameters from the command line when launching
def generate_launch_description_with_args():
    """
    Advanced example: Launch description with command-line arguments.

    Usage:
        ros2 launch simple_launch.py use_sim_time:=true
    """
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    return LaunchDescription([
        # Declare command-line arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),

        # Publisher node using launch arguments
        Node(
            package='my_robot_package',
            executable='publisher_node',
            name='sensor_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),

        # Subscriber node
        Node(
            package='my_robot_package',
            executable='subscriber_node',
            name='data_logger',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
    ])


# Multi-robot example: Launch the same node multiple times
def generate_launch_description_multi_robot():
    """
    Example: Launch multiple instances of the same node for multi-robot systems.

    Each robot gets its own namespace to avoid topic name conflicts.
    """
    return LaunchDescription([
        # Robot 1
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='controller',
            namespace='robot1',
            output='screen',
            parameters=[{
                'robot_id': 1,
                'x_offset': 0.0,
                'y_offset': 0.0,
            }]
        ),

        # Robot 2
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='controller',
            namespace='robot2',
            output='screen',
            parameters=[{
                'robot_id': 2,
                'x_offset': 2.0,
                'y_offset': 0.0,
            }]
        ),

        # Robot 3
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='controller',
            namespace='robot3',
            output='screen',
            parameters=[{
                'robot_id': 3,
                'x_offset': 4.0,
                'y_offset': 0.0,
            }]
        ),
    ])


# Expected output when running this launch file:
# [INFO] [launch]: All log files can be found below /home/user/.ros/log/...
# [INFO] [launch]: Default logging verbosity is set to INFO
# [INFO] [sensor_publisher]: Node started with update_rate=5.0
# [INFO] [data_logger]: Subscribed to /sensors/lidar
# [INFO] [processing.data_processor]: Node started in namespace 'processing'
# [INFO] [heartbeat]: Timer node started with period=1.0s

# To check running nodes:
#   ros2 node list
# Output:
#   /sensor_publisher
#   /data_logger
#   /processing/data_processor
#   /heartbeat

# To check topics:
#   ros2 topic list
# Output:
#   /sensors/lidar (published by sensor_publisher, subscribed by data_logger)
#   /processing/sensor_data (subscribed by data_processor)

# To stop all nodes:
#   Ctrl+C in the terminal running the launch file
