cmake_minimum_required(VERSION 3.5)
project(esp32_bridge)

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

add_executable(esp32_bridge_node src/esp32_bridge.cpp)
ament_target_dependencies(esp32_bridge_node
rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
)

install(TARGETS
esp32_bridge_node
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
simple_a_star.py
DESTINATION lib/${PROJECT_NAME}
RENAME simple_a_star
PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

install(DIRECTORY
launch
urdf
meshes
config
rviz
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
