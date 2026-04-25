cmake_minimum_required(VERSION 3.8)
project(motion_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME}_components SHARED
  src/angle_branch_utils.cpp
  src/execution_orchestrator.cpp
  src/planner_router.cpp
  src/ik_selector.cpp
  src/seed_manager.cpp
  src/wrist_flip_guard.cpp
  src/trajectory_post_processor.cpp
  src/quality_gate.cpp
  src/orientation_filter.cpp
  src/planning_scene_manager.cpp
  src/execute_motion_action_support.cpp
  src/non_motion_primitive_executor.cpp
  src/dispatch_trajectory_executor.cpp
  src/motion_primitive_executor.cpp
  src/primitive_router_dispatch.cpp
  src/query_handler.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  builtin_interfaces
  rclcpp
  rclcpp_action
  control_msgs
  interfaces
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  angles
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometric_shapes
  tf2_geometry_msgs
  pluginlib
)

target_link_libraries(${PROJECT_NAME}_components
  yaml-cpp
)

add_executable(motion_core_node
  src/motion_core_node.cpp
)

target_link_libraries(motion_core_node
  ${PROJECT_NAME}_components
)

ament_target_dependencies(motion_core_node
  builtin_interfaces
  rclcpp
  rclcpp_action
  control_msgs
  interfaces
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  angles
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometric_shapes
  tf2_geometry_msgs
  pluginlib
)

find_package(ruckig QUIET)
if(ruckig_FOUND)
  target_link_libraries(motion_core_node ruckig::ruckig)
  target_compile_definitions(motion_core_node PRIVATE MOTION_CORE_HAS_RUCKIG=1)
endif()

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

  ament_add_gtest(test_move_rel_validator
    test/test_move_rel_validator.cpp
  )
  if(TARGET test_move_rel_validator)
    target_link_libraries(test_move_rel_validator
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_move_rel_validator PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_angle_branch_utils
    test/test_angle_branch_utils.cpp
  )
  if(TARGET test_angle_branch_utils)
    target_link_libraries(test_angle_branch_utils
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_angle_branch_utils PRIVATE
      include
    )
    ament_target_dependencies(test_angle_branch_utils
      angles
      moveit_core
    )
  endif()

  ament_add_gtest(test_execution_orchestrator
    test/test_execution_orchestrator.cpp
  )
  if(TARGET test_execution_orchestrator)
    target_link_libraries(test_execution_orchestrator
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_execution_orchestrator PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_execute_motion_action_support
    test/test_execute_motion_action_support.cpp
  )
  if(TARGET test_execute_motion_action_support)
    target_link_libraries(test_execute_motion_action_support
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_execute_motion_action_support PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_seed_manager
    test/test_seed_manager.cpp
  )
  if(TARGET test_seed_manager)
    target_link_libraries(test_seed_manager
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_seed_manager PRIVATE
      include
    )
    ament_target_dependencies(test_seed_manager
      rclcpp
      sensor_msgs
      builtin_interfaces
    )
  endif()

  ament_add_gtest(test_query_handler
    test/test_query_handler.cpp
  )
  if(TARGET test_query_handler)
    target_link_libraries(test_query_handler
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_query_handler PRIVATE
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
  builtin_interfaces
  rclcpp
  rclcpp_action
  control_msgs
  interfaces
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  angles
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometric_shapes
  tf2_geometry_msgs
  pluginlib
)

ament_package()
