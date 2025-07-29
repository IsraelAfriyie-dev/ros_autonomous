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

# For message generation (only if you have custom messages)
find_package(rosidl_default_generators REQUIRED)

# Declare custom messages (only if you have msg/PointArray.msg file)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Include headers
include_directories(include)

# Add executables
add_executable(global_rrt_detector src/global_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(global_rrt_detector 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  visualization_msgs
)

add_executable(local_rrt_detector src/local_rrt_detector.cpp src/functions.cpp src/mtrand.cpp)
ament_target_dependencies(local_rrt_detector 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  visualization_msgs
)

# Link custom messages with executables (only if using custom messages)
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

# Install launch and config files if they exist
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py" PATTERN "*.xml" PATTERN "*.yaml"
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.json" PATTERN "*.xml"
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Final package
ament_package()
