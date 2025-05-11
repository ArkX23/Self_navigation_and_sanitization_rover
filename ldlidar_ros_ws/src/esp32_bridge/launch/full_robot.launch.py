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

    # LDROBOT LiDAR publisher node
    ldlidar_node = Node(
        package='ldlidar_sl_ros2',
        executable='ldlidar_sl_ros2_node',
        name='ldlidar_publisher_ld14',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD14P'},
            {'laser_scan_topic_name': 'scan'},
            {'point_cloud_2d_topic_name': 'pointcloud2d'},
            {'frame_id': 'laser'},
            {'port_name': '/dev/ttyUSB0'},
            {'serial_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 135.0},
            {'angle_crop_max': 225.0}
        ]
    )

    # Robot State Publisher for URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open('/home/arjun/ldlidar_ros2_ws/src/esp32_bridge/urdf/assem2.urdf').read()}]
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()
    ld.add_action(esp32_bridge_node)
    ld.add_action(ldlidar_node)
    ld.add_action(robot_state_publisher_node)
    return ld
