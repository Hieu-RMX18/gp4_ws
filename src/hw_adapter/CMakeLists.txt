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
find_package(motoros2_interfaces QUIET)

add_library(${PROJECT_NAME}_components SHARED
  src/backend_capabilities.cpp
  src/joint_state_monitor.cpp
  src/robot_status_monitor.cpp
  src/motoros2_session_manager.cpp
  src/trajectory_executor.cpp
  src/tool_state_monitor.cpp
  src/recovery_state_machine.cpp
  src/hw_adapter_node.cpp
  src/hw_adapter_dispatch.cpp
  src/hw_adapter_services.cpp
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

if(motoros2_interfaces_FOUND)
  ament_target_dependencies(${PROJECT_NAME}_components motoros2_interfaces)
endif()

add_executable(hw_adapter_node
  src/main.cpp
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
)

if(motoros2_interfaces_FOUND)
  ament_export_dependencies(motoros2_interfaces)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

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
    endif()
  endif()
endif()

ament_package()
