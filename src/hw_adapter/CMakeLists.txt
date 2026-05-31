cmake_minimum_required(VERSION 3.8)
project(hw_adapter)

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
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(industrial_msgs REQUIRED)
find_package(motion_core REQUIRED)
find_package(motoros2_interfaces QUIET)

add_library(${PROJECT_NAME}_components SHARED
  src/monitoring/backend_capabilities.cpp
  src/monitoring/joint_state_monitor.cpp
  src/monitoring/robot_status_monitor.cpp
  src/session/motoros2_session_manager.cpp
  src/dispatch/trajectory_executor.cpp
  src/monitoring/tool_state_monitor.cpp
  src/session/recovery_state_machine.cpp
  src/node/hw_adapter_node.cpp
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
  trajectory_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  interfaces
  moveit_msgs
  pluginlib
  industrial_msgs
)
# Link motion_core library only (not the node executable which is also in the export set)
target_link_libraries(${PROJECT_NAME}_components
  motion_core::motion_core_components
)

if(motoros2_interfaces_FOUND)
  ament_target_dependencies(${PROJECT_NAME}_components motoros2_interfaces)
endif()

add_executable(hw_adapter_node
  src/node/main.cpp
)

target_link_libraries(hw_adapter_node
  ${PROJECT_NAME}_components
)

ament_target_dependencies(hw_adapter_node
  rclcpp
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_components
  hw_adapter_node
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
  trajectory_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  interfaces
  moveit_msgs
  pluginlib
  industrial_msgs
  motion_core
)

if(motoros2_interfaces_FOUND)
  ament_export_dependencies(motoros2_interfaces)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(ROS_TEST_LOG_DIR "${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
  file(MAKE_DIRECTORY "${ROS_TEST_LOG_DIR}")

  function(configure_ros_test_environment test_name)
    if(TARGET ${test_name})
      set_tests_properties(${test_name} PROPERTIES
        ENVIRONMENT "ROS_LOG_DIR=${ROS_TEST_LOG_DIR};ROS_LOCALHOST_ONLY=1;RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
      )
    endif()
  endfunction()

  ament_add_gtest(test_backend_capabilities
    test/test_backend_capabilities.cpp
  )
  if(TARGET test_backend_capabilities)
    target_link_libraries(test_backend_capabilities
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_backend_capabilities PRIVATE
      include
    )
    ament_target_dependencies(test_backend_capabilities
      rclcpp
      sensor_msgs
    )
    configure_ros_test_environment(test_backend_capabilities)
  endif()

  ament_add_gtest(test_robot_status_monitor
    test/test_robot_status_monitor.cpp
  )
  if(TARGET test_robot_status_monitor)
    target_link_libraries(test_robot_status_monitor
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_robot_status_monitor PRIVATE
      include
    )
    ament_target_dependencies(test_robot_status_monitor
      rclcpp
      industrial_msgs
      interfaces
    )
    configure_ros_test_environment(test_robot_status_monitor)
  endif()

  ament_add_gtest(test_joint_state_monitor
    test/test_joint_state_monitor.cpp
  )
  if(TARGET test_joint_state_monitor)
    target_link_libraries(test_joint_state_monitor
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_joint_state_monitor PRIVATE
      include
    )
    ament_target_dependencies(test_joint_state_monitor
      rclcpp
      sensor_msgs
    )
    configure_ros_test_environment(test_joint_state_monitor)
  endif()

  ament_add_gtest(test_motoros2_session_manager
    test/test_motoros2_session_manager.cpp
  )
  if(TARGET test_motoros2_session_manager)
    target_link_libraries(test_motoros2_session_manager
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_motoros2_session_manager PRIVATE
      include
    )
    ament_target_dependencies(test_motoros2_session_manager
      rclcpp
      rclcpp_action
      control_msgs
      motoros2_interfaces
    )
    configure_ros_test_environment(test_motoros2_session_manager)
  endif()

  ament_add_gtest(test_trajectory_executor
    test/test_trajectory_executor.cpp
  )
  if(TARGET test_trajectory_executor)
    target_link_libraries(test_trajectory_executor
      ${PROJECT_NAME}_components
    )
    target_include_directories(test_trajectory_executor PRIVATE
      include
    )
    ament_target_dependencies(test_trajectory_executor
      rclcpp
      rclcpp_action
      control_msgs
      trajectory_msgs
    )
    configure_ros_test_environment(test_trajectory_executor)
  endif()

  if(motoros2_interfaces_FOUND)
    ament_add_gtest(test_tool_state_monitor
      test/test_tool_state_monitor.cpp
    )
    if(TARGET test_tool_state_monitor)
      target_link_libraries(test_tool_state_monitor
        ${PROJECT_NAME}_components
      )
      target_include_directories(test_tool_state_monitor PRIVATE
        include
      )
      ament_target_dependencies(test_tool_state_monitor
        rclcpp
        motoros2_interfaces
      )
      configure_ros_test_environment(test_tool_state_monitor)
    endif()

    ament_add_gtest(test_hw_adapter_node
      test/test_hw_adapter_node.cpp
    )
    if(TARGET test_hw_adapter_node)
      target_link_libraries(test_hw_adapter_node
        ${PROJECT_NAME}_components
      )
      target_include_directories(test_hw_adapter_node PRIVATE
        include
      )
      ament_target_dependencies(test_hw_adapter_node
        rclcpp
        rclcpp_action
        control_msgs
        industrial_msgs
        interfaces
        motoros2_interfaces
        sensor_msgs
        trajectory_msgs
      )
      configure_ros_test_environment(test_hw_adapter_node)
    endif()
  endif()
endif()

ament_package()
