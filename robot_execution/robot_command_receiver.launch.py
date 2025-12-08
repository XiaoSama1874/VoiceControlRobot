#!/usr/bin/env python3

"""
Launch file for Robot Command Receiver node.

This launch file starts the robot_command_receiver node that listens for
commands from ROSBridge and executes them.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    
    robot_command_receiver_node = Node(
        package='voice_control_robot',  # Adjust package name as needed
        executable='robot_command_receiver',
        name='robot_command_receiver',
        output='screen',
        parameters=[
            {'command_frequency': 5},
            {'endpoint_speed': 0.05}
        ]
    )
    
    return LaunchDescription([
        robot_command_receiver_node
    ])

