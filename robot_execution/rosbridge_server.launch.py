#!/usr/bin/env python3

"""
Launch file for ROSBridge WebSocket Server

This launch file starts the ROSBridge WebSocket server that allows
external clients (like the Mac computer) to communicate with ROS2
via WebSocket protocol.

Usage:
    ros2 launch <package_name> rosbridge_server.launch.py port:=9090
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for ROSBridge server."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port number for ROSBridge WebSocket server'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address to bind ROSBridge server (0.0.0.0 for all interfaces)'
    )
    
    # ROSBridge WebSocket server node
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
        }],
        output='screen',
        respawn=True,  # Automatically restart if node crashes
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        port_arg,
        address_arg,
        rosbridge_websocket_node
    ])

