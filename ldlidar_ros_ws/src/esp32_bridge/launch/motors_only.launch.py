#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ESP32 Bridge node for motor control and odometry
    esp32_bridge_node = Node(
        package='esp32_bridge',
        executable='esp32_bridge_node',
        name='esp32_bridge',
        output='screen'
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(esp32_bridge_node)
    return ld
