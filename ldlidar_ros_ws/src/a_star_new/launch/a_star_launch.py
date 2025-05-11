from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_a_star_new = get_package_share_directory('a_star_new')
    
    return LaunchDescription([
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
                'robot_description': open(os.path.join(pkg_a_star_new, 'urdf', 'assem2.urdf')).read()
            }]
        ),
        # A* Node
        Node(
            package='a_star_new',
            executable='simple_a_star',
            name='simple_a_star',
            output='screen'
        )
    ])
