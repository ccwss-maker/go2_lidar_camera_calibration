cmake_minimum_required(VERSION 3.8)
project(go2_user_tools)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)

add_executable(pointcloud_filter_forward
  src/pointcloud_filter_forward.cpp
)
target_include_directories(pointcloud_filter_forward PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pointcloud_filter_forward
  ${PCL_LIBRARIES}
)
ament_target_dependencies(pointcloud_filter_forward
  rclcpp
  sensor_msgs
  pcl_conversions
)

add_executable(icp_extrinsic_match
  src/icp_extrinsic_match.cpp
)
target_include_directories(icp_extrinsic_match PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(icp_extrinsic_match
  ${PCL_LIBRARIES}
)
ament_target_dependencies(icp_extrinsic_match
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  pointcloud_filter_forward
  icp_extrinsic_match
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/lowstate_to_jointstate.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
