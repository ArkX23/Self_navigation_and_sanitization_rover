from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_nav2_mapless = get_package_share_directory('nav2_mapless')
    pkg_esp32_bridge = get_package_share_directory('esp32_bridge')
    
    # Nav2 parameters
    params_file = os.path.join(pkg_nav2_mapless, 'config', 'nav2_mapless_params.yaml')
    
    return LaunchDescription([
        # ESP32 Bridge (odometry, cmd_vel)
        Node(
            package='esp32_bridge',
            executable='esp32_bridge_node',
            name='esp32_bridge_node',
            output='screen'
        ),
        # LiDAR
        Node(
            package='ldlidar_sl_ros2',
            executable='ldlidar_sl_ros2_node',
            name='ldlidar_node',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD14P'},
                {'laser_scan_topic_name': 'scan'},
                {'point_cloud_2d_topic_name': 'none'},
                {'frame_id': 'laser'},
                {'port_name': '/dev/ttyUSB0'},
                {'serial_baudrate': 230400},
                {'laser_scan_dir': True},
                {'range_min': 0.12},
                {'range_max': 12.0}
            ]
        ),
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(os.path.join(pkg_esp32_bridge, 'urdf', 'assem2.urdf')).read()
            }]
        ),
        # Nav2 Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'False',
                'params_file': params_file
            }.items()
        )
    ])
