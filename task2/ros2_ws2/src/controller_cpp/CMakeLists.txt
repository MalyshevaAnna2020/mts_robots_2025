cmake_minimum_required(VERSION 3.8)

set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

project(controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map_building src/map_building.cpp)
ament_target_dependencies(map_building
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  laser_geometry
  pcl_conversions
  tf2
  tf2_eigen
)

add_executable(telemetry_bridge src/telemetry_bridge.cpp)
ament_target_dependencies(telemetry_bridge
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  laser_geometry
  pcl_conversions
  tf2
  tf2_eigen
)

target_link_libraries(map_building
  ${PCL_LIBRARIES}
)
install(TARGETS map_building telemetry_bridge
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(telemetry_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(map_building PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
