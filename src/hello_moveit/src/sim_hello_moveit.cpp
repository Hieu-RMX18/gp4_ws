#include <memory>

#include "motoros2_interfaces/srv/start_traj_mode.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <chrono>
#include <thread>

int main(int argc, char* argv[])
{
  //-----------------------------------------------------
  rclcpp::init(argc, argv);

  //-------------------controller enable-----------------------------
  auto drive = rclcpp::Node::make_shared("enable_client");

  auto enable_client = drive->create_client<motoros2_interfaces::srv::StartTrajMode>("yaskawa/start_traj_mode");

  if (enable_client->wait_for_service(std::chrono::seconds(2))) {
    auto request = std::make_shared<motoros2_interfaces::srv::StartTrajMode::Request>();
    auto future = enable_client->async_send_request(request);
    RCLCPP_INFO(drive->get_logger(), "Sending start_traj_mode request...");
    if (rclcpp::spin_until_future_complete(drive, future) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(drive->get_logger(), "Successfully enabled trajectory mode.");
    } else {
      RCLCPP_ERROR(drive->get_logger(), "Failed to enable trajectory mode.");
    }
  } else {
    RCLCPP_WARN(drive->get_logger(), "Service yaskawa/start_traj_mode not available (simulation mode?).");
  }

  //-----------------------------------------------------

  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "gp4_arm");

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.0;
    msg.orientation.y = 1.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    msg.position.x = 0.25;
    msg.position.y = 0.0;
    msg.position.z = 0.25;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  std::this_thread::sleep_for(std::chrono::seconds(5));
  // Execute the plan
  if (success)
  {
    RCLCPP_INFO(logger, "Planning OK!");
    // move_group_interface.execute(plan);
    move_group_interface.asyncExecute(plan);
    std::this_thread::sleep_for(std::chrono::seconds(10));
    RCLCPP_INFO(logger, "end of waiting");
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Execute the plan
  // if (success)
  // {
  //   move_group_interface.asyncExecute(plan);
  //   std::cout << "executing second time "
  //             << "\n";
  //   move_group_interface.asyncExecute(plan);

  //   std::this_thread::sleep_for(std::chrono::seconds(10));
  // }
  // else
  // {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }

  // if (success)
  // {
  //     // Execute the trajectory
  //     moveit::planning_interface::MoveItErrorCode execute_result = move_group_interface.asyncExecute(plan);

  //     if (execute_result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  //     {
  //         RCLCPP_INFO(logger, "Execution succeeded");
  //     }
  //     else
  //     {
  //         RCLCPP_ERROR(logger,"Execution failed");
  //     }
  //     moveit::planning_interface::MoveItErrorCode execute_result = move_group_interface.asyncExecute(plan);

  // }
  // else
  // {
  //     RCLCPP_ERROR(logger, "Planning failed");
  // }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
