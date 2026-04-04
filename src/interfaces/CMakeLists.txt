cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(msg_files
  "msg/MotionFeedback.msg"
  "msg/MotionRequest.msg"
  "msg/MotionResult.msg"
  "msg/RobotReadiness.msg"
)

set(srv_files
  "srv/ValidateCommand.srv"
)

set(action_files
  "action/ExecuteMotion.action"
  "action/DispatchTrajectory.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs trajectory_msgs
)

ament_package()
