import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config_path = "/home/arjun/ldlidar_ros2_ws/src/esp32_bridge/config/slam_toolbox.yaml"

    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    esp32_bridge_node = Node(
        package="esp32_bridge",
        executable="esp32_bridge_node",
        name="esp32_bridge",
        output="screen"
    )

    ldlidar_node = Node(
        package="ldlidar_sl_ros2",
        executable="ldlidar_sl_ros2_node",
        name="ldlidar_publisher_ld14",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD14P"},
            {"laser_scan_topic_name": "scan"},
            {"point_cloud_2d_topic_name": "pointcloud2d"},
            {"frame_id": "laser"},
            {"port_name": "/dev/ttyUSB0"},
            {"serial_baudrate": 230400}
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": open("/home/arjun/ldlidar_ros2_ws/src/esp32_bridge/urdf/assem2.urdf").read()}]
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[config_path, {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"autostart": True},
            {"node_names": ["slam_toolbox"]}
        ]
    )

    return LaunchDescription([
        use_sim_time,
        esp32_bridge_node,
        ldlidar_node,
        robot_state_publisher_node,
        slam_toolbox_node,
        lifecycle_manager_node
    ])
