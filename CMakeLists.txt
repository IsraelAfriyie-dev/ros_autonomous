cmake_minimum_required(VERSION 3.5)
project(ros_autonomous)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# For message generation
find_package(rosidl_default_generators REQUIRED)

# Declare custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include headers
include_directories(include)

# Add executables (if any — example below)
add_executable(global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(global_rrt_detector rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs)

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(local_rrt_detector rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs)

# Link messages with executables (optional if they use them)
rosidl_target_interfaces(global_rrt_detector
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(local_rrt_detector
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install executables
install(TARGETS
  global_rrt_detector
  local_rrt_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files if any
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Final package
ament_export_dependencies(rosidl_default_runtime)
ament_package()

## Generate added messages and services with any dependencies listed here
cmake_minimum_required(VERSION 3.5)
project(ros_autonomous)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# Add executables
add_executable(global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(global_rrt_detector rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros tf2)

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(local_rrt_detector rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros tf2)

# Install executables
install(TARGETS
  global_rrt_detector
  local_rrt_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, and other assets if needed
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Package export
ament_package()
