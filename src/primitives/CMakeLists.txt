cmake_minimum_required(VERSION 3.8)
project(primitives)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(motion_core REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/primitive_dispatcher.cpp
  src/primitive_ptp.cpp
  src/primitive_lin.cpp
  src/primitive_home.cpp
  src/primitive_approach.cpp
  src/primitive_retract.cpp
  src/primitive_circ.cpp
  src/primitive_blended_sequence.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  interfaces
  tf2_geometry_msgs
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_ros_planning_interface::moveit_planning_scene_interface
  moveit_ros_planning_interface::moveit_common_planning_interface_objects
  motion_core::motion_core_components
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

install(TARGETS
  ${PROJECT_NAME}
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
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  interfaces
  motion_core
  tf2_geometry_msgs
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_primitive_home
    test/test_primitive_home.cpp
  )
  if(TARGET test_primitive_home)
    target_link_libraries(test_primitive_home
      ${PROJECT_NAME}
    )
    target_include_directories(test_primitive_home PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_primitive_ptp
    test/test_primitive_ptp.cpp
  )
  if(TARGET test_primitive_ptp)
    target_link_libraries(test_primitive_ptp
      ${PROJECT_NAME}
    )
    target_include_directories(test_primitive_ptp PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_primitive_lin
    test/test_primitive_lin.cpp
  )
  if(TARGET test_primitive_lin)
    target_link_libraries(test_primitive_lin
      ${PROJECT_NAME}
    )
    target_include_directories(test_primitive_lin PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_primitive_approach_retract
    test/test_primitive_approach_retract.cpp
  )
  if(TARGET test_primitive_approach_retract)
    target_link_libraries(test_primitive_approach_retract
      ${PROJECT_NAME}
    )
    target_include_directories(test_primitive_approach_retract PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_primitive_circ
    test/test_primitive_circ.cpp
  )
  if(TARGET test_primitive_circ)
    target_link_libraries(test_primitive_circ
      ${PROJECT_NAME}
    )
    target_include_directories(test_primitive_circ PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_primitive_blended_sequence
    test/test_primitive_blended_sequence.cpp
  )
  if(TARGET test_primitive_blended_sequence)
    target_link_libraries(test_primitive_blended_sequence
      ${PROJECT_NAME}
    )
    target_include_directories(test_primitive_blended_sequence PRIVATE
      include
    )
  endif()

  ament_add_gtest(test_primitive_types_dispatcher
    test/test_primitive_types_dispatcher.cpp
  )
  if(TARGET test_primitive_types_dispatcher)
    target_link_libraries(test_primitive_types_dispatcher
      ${PROJECT_NAME}
    )
    target_include_directories(test_primitive_types_dispatcher PRIVATE
      include
    )
  endif()
endif()

ament_package()
