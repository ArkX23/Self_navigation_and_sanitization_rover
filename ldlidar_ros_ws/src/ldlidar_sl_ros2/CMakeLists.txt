cmake_minimum_required(VERSION 3.5)
project(ldlidar_sl_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Threads REQUIRED)  # Added for pthread support

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# user add
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/include/
)

file(GLOB LDLIDAR_DRI ${CMAKE_CURRENT_SOURCE_DIR}/ldlidar_driver/src/*.cpp)

add_executable(${PROJECT_NAME}_node  
  ${CMAKE_CURRENT_SOURCE_DIR}/src/demo.cpp 
  ${LDLIDAR_DRI} 
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME}_node Threads::Threads)  # Updated to use Threads::Threads

# Install 
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz2 scripts
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
