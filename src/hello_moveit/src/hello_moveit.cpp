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

  if (enable_client->wait_for_service(std::chrono::seconds(2)))
  {
    auto request = std::make_shared<motoros2_interfaces::srv::StartTrajMode::Request>();
    auto future = enable_client->async_send_request(request);
    RCLCPP_INFO(drive->get_logger(), "Sending start_traj_mode request...");
    if (rclcpp::spin_until_future_complete(drive, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(drive->get_logger(), "Successfully enabled trajectory mode.");
    }
    else
    {
      RCLCPP_ERROR(drive->get_logger(), "Failed to enable trajectory mode.");
    }
  }
  else
  {
    RCLCPP_WARN(drive->get_logger(), "Service yaskawa/start_traj_mode not available (simulation mode?).");
  }

  //-----------------------------------------------------

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.arguments({ "--ros-args", "-r", "/joint_states:=/yaskawa/joint_states" });
  auto const node = std::make_shared<rclcpp::Node>("hello_moveit", node_options);

  // Tạo spinner để Node có thể liên tục lắng nghe Topic dưới nền
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  spinner.detach();

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "gp4_arm");

  // Lấy vị trí hiện tại làm điểm xuất phát
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose start_pose = move_group_interface.getCurrentPose().pose;
  waypoints.push_back(start_pose);

  // Set a target Pose (cùng hướng hiện tại, chỉ thay đổi x, y, z)
  geometry_msgs::msg::Pose target_pose = start_pose;
  target_pose.position.x = 0.25;
  target_pose.position.y = 0.20;
  target_pose.position.z = 0.4;
  // KHÔNG đổi orientation để tránh lỗi Singularity/kẹt khớp khi đang di chuyển
  waypoints.push_back(target_pose);

  // Đặt giới hạn tốc độ và gia tốc (0.1 = 10%)
  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;  // Tắt jump threshold
  const double eef_step = 0.05;       // Bước nội suy 5cm để đỡ gắt và giật

  RCLCPP_INFO(logger, "Đang tính toán Cartesian Path...");
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.9)
  {
    RCLCPP_INFO(logger, "Planning OK! (%.2f%% completed).", fraction * 100.0);

    // Làm chậm và mượt hóa quỹ đạo tuyến tính bằng cách scale giãn thời gian nội suy
    double scale_factor = 4.0;  // Chạy chậm đi 4 lần
    for (auto& point : trajectory.joint_trajectory.points)
    {
      double time_in_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      time_in_sec *= scale_factor;
      point.time_from_start.sec = std::floor(time_in_sec);
      point.time_from_start.nanosec = (time_in_sec - point.time_from_start.sec) * 1e9;

      for (size_t i = 0; i < point.velocities.size(); ++i)
      {
        point.velocities[i] /= scale_factor;
        if (!point.accelerations.empty() && i < point.accelerations.size())
        {
          point.accelerations[i] /= (scale_factor * scale_factor);
        }
      }
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    RCLCPP_INFO(logger, "Thực thi Cartesian Path (đã được làm mượt)...");

    auto result = move_group_interface.execute(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Execution completed successfully.");
    }
    else
    {
      RCLCPP_ERROR(logger, "Execution failed with error code: %d", result.val);
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed! Chỉ hoàn thành %.2f%% đường đi.", fraction * 100.0);
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
