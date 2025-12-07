# At least 3.10 so that IMPORTED targets can be used
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Specify the project the used language and the version
project(cpm_lab_commonroad_msgs VERSION 0.1)

# Configure compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CommonroadDDSInterval.msg
  msg/CommonroadDDSPositionInterval.msg
  msg/CommonroadDDSGoalState.msg
  msg/CommonroadDDSShape.msg
  msg/CommonroadDDSPoint.msg
  msg/CommonroadDDSRectangle.msg
  msg/CommonroadDDSCircle.msg
  msg/CommonroadDDSPolygon.msg
  msg/CommonroadObstacle.msg
  msg/CommonroadObstacleList.msg
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
)
ament_package()
