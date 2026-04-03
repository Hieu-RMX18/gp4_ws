cmake_minimum_required(VERSION 3.8)
project(motion_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME}_components
  src/planner_router.cpp
  src/ik_selector.cpp
  src/seed_manager.cpp
  src/wrist_flip_guard.cpp
  src/trajectory_post_processor.cpp
  src/quality_gate.cpp
  src/orientation_filter.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_action
  interfaces
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2_geometry_msgs
  pluginlib
)

add_executable(motion_core_node
  src/motion_core_node.cpp
)

target_link_libraries(motion_core_node
  ${PROJECT_NAME}_components
)

ament_target_dependencies(motion_core_node
  rclcpp
  rclcpp_action
  interfaces
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2_geometry_msgs
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_planner_router
    test/test_planner_router.cpp
  )
  if(TARGET test_planner_router)
    target_link_libraries(test_planner_router
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_planner_router PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_orientation_filter
    test/test_orientation_filter.cpp
  )
  if(TARGET test_orientation_filter)
    target_link_libraries(test_orientation_filter
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_orientation_filter PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_wrist_flip_guard
    test/test_wrist_flip_guard.cpp
  )
  if(TARGET test_wrist_flip_guard)
    target_link_libraries(test_wrist_flip_guard
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_wrist_flip_guard PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_quality_gate
    test/test_quality_gate.cpp
  )
  if(TARGET test_quality_gate)
    target_link_libraries(test_quality_gate
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_quality_gate PRIVATE
      include
    )
  endif()
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_components
  motion_core_node
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  interfaces
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2_geometry_msgs
  pluginlib
)

ament_package()
