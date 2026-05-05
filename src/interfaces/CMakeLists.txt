cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(msg_files
  "msg/MotionFeedback.msg"
  "msg/MotionRequest.msg"
  "msg/MotionResult.msg"
  "msg/RobotReadiness.msg"
  # Experimental jog pendant interfaces
  "msg/JogCommand.msg"
  "msg/ServoBridgeStatus.msg"
  # BLENDED_SEQUENCE typed step (W2.T0)
  "msg/SequenceStep.msg"
  # W4.T0 — Perception interfaces
  "msg/PerceptionStatus.msg"
)

set(srv_files
  "srv/ValidateCommand.srv"
  "srv/GetCurrentPose.srv"
  "srv/AlarmReset.srv"
  "srv/IoSet.srv"
  # W4.T0 — Perception services
  "srv/CalibrateHandEye.srv"
  "srv/GetObjectPositions.srv"
  "srv/CheckCamera.srv"
  # W5.T2 — HMI consolidation services
  "srv/HydrateWorkplane.srv"
  "srv/GetPrimitiveConstants.srv"
  "srv/ConfirmExecution.srv"
)

set(action_files
  "action/ExecuteMotion.action"
  "action/DispatchTrajectory.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs trajectory_msgs vision_msgs
)

ament_package()
