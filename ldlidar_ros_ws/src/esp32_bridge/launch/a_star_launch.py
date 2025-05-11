from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('esp32_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[os.path.join(pkg_dir, 'urdf', 'assem2.urdf')]
        ),

        Node(
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
                {'range_min': 0.12},
                {'range_max': 12.0}
            ]
        ),

        Node(
            package='esp32_bridge',
            executable='esp32_bridge_node',
            name='esp32_bridge',
            output='screen'
        ),

        ExecuteProcess(
            cmd=['python3', os.path.join(pkg_dir, 'scripts', 'a_star_navigation.py')],
            output='screen',
            name='a_star_navigation'
        )
    ])
