#pragma once

#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/srv/get_current_pose.hpp"

namespace motion_core {
class QueryHandler {
public:
  using EnsureMoveGroupFn = std::function<bool(std::string &)>;
  using ReadCurrentTcpPoseFn = std::function<bool(
      geometry_msgs::msg::PoseStamped &, std::string &, double)>;

  QueryHandler(rclcpp::Logger logger, EnsureMoveGroupFn ensure_move_group,
               ReadCurrentTcpPoseFn read_current_tcp_pose);

  void handle_get_current_pose(
      const std::shared_ptr<interfaces::srv::GetCurrentPose::Request> request,
      std::shared_ptr<interfaces::srv::GetCurrentPose::Response> response);

private:
  rclcpp::Logger logger_;
  EnsureMoveGroupFn ensure_move_group_;
  ReadCurrentTcpPoseFn read_current_tcp_pose_;
};
} // namespace motion_core
