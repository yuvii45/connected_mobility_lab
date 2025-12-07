cmake_minimum_required(VERSION 3.8)
project(cpm_lab_indoor_positioning_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 20)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cpm_lab_lab_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(cpm_lab_lab_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# Allow CMake to locate our script for the local third party library
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package(PYLON)

if(PYLON_FOUND)

# =============================================================================
# Core library (logic, controllers, simulation)
add_library(${PROJECT_NAME}_core SHARED
  src/IndoorPositioningSystem.cpp
  src/IndoorPositioningSystemParameter.cpp
  src/LedDetection.cpp
  src/PointUndistortion.cpp
  src/PoseCalculation.cpp
  src/VehicleIdentification.cpp
  src/VehicleDetection.cpp
  src/PointProjection.cpp
  src/VehicleTracking.cpp
)

target_compile_features(${PROJECT_NAME}_core PUBLIC c_std_99 cxx_std_20)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
ament_target_dependencies(${PROJECT_NAME}_core
  cv_bridge
)
target_compile_definitions(${PROJECT_NAME}_core PRIVATE "INDOOR_POSITIONING_SYSTEM_BUILDING_LIBRARY")

# =============================================================================
# ROS component node library (composable node)
add_library(${PROJECT_NAME}_component SHARED
  component/indoor_positioning_system_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_component
  "rclcpp" "rclcpp_components" "std_msgs" "cv_bridge" "cpm_lab_lab_msgs" "tf2" "tf2_ros" "tf2_geometry_msgs"
)
target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_core
  Pylon::pylon
  ${OpenCV_LIBS}
)

# Register component and standalone executable
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN "indoor_positioning_system_ros::VehicleDetectionNode"
  EXECUTABLE indoor_positioning_system_node
)

# =============================================================================
# Installation rules
install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_component
  indoor_positioning_system_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
else()
message(WARNING "Pylon not found. Skipping build of indoor positioning system. The message"
  "does not prevent a successful build, but you will not be able to use the CPM lab physical"
  "environment. Please install Pylon manually from the website of Basler.")
endif()