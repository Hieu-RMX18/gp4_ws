cmake_minimum_required(VERSION 3.8)
project(jog_pendant)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Local: interfaces (generated messages)
find_package(interfaces REQUIRED)
# MotoROS2 point-queue interfaces
find_package(motoros2_interfaces REQUIRED)
# Industrial robot status
find_package(industrial_msgs REQUIRED)

# ── C++ node: servo_bridge_node ────────────────────────────────────────
add_executable(servo_bridge_node src/servo_bridge_node.cpp)
ament_target_dependencies(servo_bridge_node
  rclcpp
  std_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  interfaces
  motoros2_interfaces
  industrial_msgs
)

# ── Install ────────────────────────────────────────────────────────────
install(TARGETS servo_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/jog_input_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
