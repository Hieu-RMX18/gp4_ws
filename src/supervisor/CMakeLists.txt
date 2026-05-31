cmake_minimum_required(VERSION 3.8)
project(supervisor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_aggregator QUIET)
find_package(action_msgs REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

add_library(audit_logger SHARED
  src/audit_logger.cpp
)

target_include_directories(audit_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(audit_logger
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  interfaces
  diagnostic_msgs
  action_msgs
)
target_link_libraries(audit_logger JsonCpp::JsonCpp)

add_library(execution_monitor SHARED
  src/execution_monitor.cpp
)

target_include_directories(execution_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(execution_monitor
  rclcpp
  interfaces
  diagnostic_msgs
  action_msgs
)

add_executable(supervisor_node
  src/supervisor_node.cpp
)

target_link_libraries(supervisor_node
  audit_logger
  execution_monitor
)

ament_target_dependencies(supervisor_node
  rclcpp
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  audit_logger
  execution_monitor
  supervisor_node
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  interfaces
  diagnostic_msgs
  action_msgs
)

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

  ament_add_gtest(test_audit_logger
    test/test_audit_logger.cpp
  )
  if(TARGET test_audit_logger)
    target_link_libraries(test_audit_logger
      audit_logger
      JsonCpp::JsonCpp
    )
    target_include_directories(test_audit_logger PRIVATE
      include
    )
    ament_target_dependencies(test_audit_logger
      rclcpp
      rosbag2_cpp
      rosbag2_storage
      interfaces
      action_msgs
    )
    configure_ros_test_environment(test_audit_logger)
  endif()

  ament_add_gtest(test_execution_monitor
    test/test_execution_monitor.cpp
  )
  if(TARGET test_execution_monitor)
    target_link_libraries(test_execution_monitor
      execution_monitor
    )
    target_include_directories(test_execution_monitor PRIVATE
      include
    )
    ament_target_dependencies(test_execution_monitor
      rclcpp
      interfaces
      diagnostic_msgs
      action_msgs
    )
    configure_ros_test_environment(test_execution_monitor)
  endif()
endif()

ament_package()
