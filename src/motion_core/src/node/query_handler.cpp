#include "motion_core/query_handler.hpp"

#include <utility>

namespace motion_core {
QueryHandler::QueryHandler(rclcpp::Logger logger,
                           EnsureMoveGroupFn ensure_move_group,
                           ReadCurrentTcpPoseFn read_current_tcp_pose)
    : logger_(std::move(logger)),
      ensure_move_group_(std::move(ensure_move_group)),
      read_current_tcp_pose_(std::move(read_current_tcp_pose)) {}

void QueryHandler::handle_get_current_pose(
    const std::shared_ptr<interfaces::srv::GetCurrentPose::Request> request,
    std::shared_ptr<interfaces::srv::GetCurrentPose::Response> response) {
  std::string frame = request->reference_frame;
  if (frame.empty()) {
    frame = "base_link";
  }

  if (frame != "base_link") {
    response->success = false;
    response->message = "unsupported reference_frame '" + frame +
                        "'; only 'base_link' is supported";
    RCLCPP_WARN(logger_, "GET_POSE rejected: %s", response->message.c_str());
    return;
  }

  std::string move_group_reason;
  if (!ensure_move_group_(move_group_reason)) {
    response->success = false;
    response->message = "cannot read current pose: MoveGroup unavailable — " +
                        move_group_reason;
    RCLCPP_ERROR(logger_, "GET_POSE failed: %s", response->message.c_str());
    return;
  }

  geometry_msgs::msg::PoseStamped current_stamped;
  std::string current_pose_reason;
  if (!read_current_tcp_pose_(current_stamped, current_pose_reason, 5.0)) {
    response->success = false;
    response->message =
        "failed to read current TCP pose: " + current_pose_reason;
    RCLCPP_ERROR(logger_, "GET_POSE failed: %s", response->message.c_str());
    return;
  }

  if (current_stamped.header.frame_id != frame) {
    response->success = false;
    response->message = "current TCP pose is available in frame '" +
                        current_stamped.header.frame_id + "'; expected '" +
                        frame + "'";
    RCLCPP_ERROR(logger_, "GET_POSE failed: %s", response->message.c_str());
    return;
  }

  const auto &q = current_stamped.pose.orientation;
  const double qnorm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (qnorm_sq <= 1e-12) {
    response->success = false;
    response->message = "current pose has invalid/zero orientation; robot "
                        "state may be unavailable";
    RCLCPP_ERROR(logger_, "GET_POSE failed: %s", response->message.c_str());
    return;
  }

  response->success = true;
  response->message = "current TCP pose in frame: " + frame;
  response->current_pose = current_stamped.pose;

  RCLCPP_INFO(logger_,
              "GET_POSE success: position=(%.4f, %.4f, %.4f), "
              "orientation=(%.4f, %.4f, %.4f, %.4f), frame=%s",
              current_stamped.pose.position.x, current_stamped.pose.position.y,
              current_stamped.pose.position.z, q.x, q.y, q.z, q.w,
              frame.c_str());
}
} // namespace motion_core
