#include <string>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "motion_core/query_handler.hpp"

namespace motion_core {
using GetCurrentPose = interfaces::srv::GetCurrentPose;

TEST(QueryHandlerTest, RejectsUnsupportedFrameWithoutInvokingCallbacks) {
  bool ensure_called = false;
  bool read_called = false;

  QueryHandler handler(
      rclcpp::get_logger("query_handler_test"),
      [&](std::string &) {
        ensure_called = true;
        return true;
      },
      [&](geometry_msgs::msg::PoseStamped &, std::string &, double) {
        read_called = true;
        return true;
      });

  auto request = std::make_shared<GetCurrentPose::Request>();
  auto response = std::make_shared<GetCurrentPose::Response>();
  request->reference_frame = "tool0";

  handler.handle_get_current_pose(request, response);

  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("unsupported reference_frame"),
            std::string::npos);
  EXPECT_FALSE(ensure_called);
  EXPECT_FALSE(read_called);
}

TEST(QueryHandlerTest, FailsClosedWhenMoveGroupIsUnavailable) {
  QueryHandler handler(
      rclcpp::get_logger("query_handler_test"),
      [&](std::string &reason) {
        reason = "MoveGroup init failed";
        return false;
      },
      [&](geometry_msgs::msg::PoseStamped &, std::string &, double) {
        return true;
      });

  auto request = std::make_shared<GetCurrentPose::Request>();
  auto response = std::make_shared<GetCurrentPose::Response>();
  request->reference_frame = "base_link";

  handler.handle_get_current_pose(request, response);

  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("MoveGroup unavailable"), std::string::npos);
}

TEST(QueryHandlerTest, FailsClosedWhenPoseReadFails) {
  QueryHandler handler(
      rclcpp::get_logger("query_handler_test"),
      [&](std::string &) { return true; },
      [&](geometry_msgs::msg::PoseStamped &, std::string &reason, double) {
        reason = "latest /yaskawa/joint_states unavailable";
        return false;
      });

  auto request = std::make_shared<GetCurrentPose::Request>();
  auto response = std::make_shared<GetCurrentPose::Response>();
  request->reference_frame = "base_link";

  handler.handle_get_current_pose(request, response);

  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("failed to read current TCP pose"),
            std::string::npos);
}

TEST(QueryHandlerTest, ReturnsCurrentPoseOnSuccess) {
  QueryHandler handler(
      rclcpp::get_logger("query_handler_test"),
      [&](std::string &) { return true; },
      [&](geometry_msgs::msg::PoseStamped &current_stamped, std::string &,
          double) {
        current_stamped.header.frame_id = "base_link";
        current_stamped.pose.position.x = 0.1;
        current_stamped.pose.position.y = 0.2;
        current_stamped.pose.position.z = 0.3;
        current_stamped.pose.orientation.x = 0.0;
        current_stamped.pose.orientation.y = 0.0;
        current_stamped.pose.orientation.z = 0.0;
        current_stamped.pose.orientation.w = 1.0;
        return true;
      });

  auto request = std::make_shared<GetCurrentPose::Request>();
  auto response = std::make_shared<GetCurrentPose::Response>();
  request->reference_frame = "base_link";

  handler.handle_get_current_pose(request, response);

  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "current TCP pose in frame: base_link");
  EXPECT_DOUBLE_EQ(response->current_pose.position.x, 0.1);
  EXPECT_DOUBLE_EQ(response->current_pose.position.y, 0.2);
  EXPECT_DOUBLE_EQ(response->current_pose.position.z, 0.3);
  EXPECT_DOUBLE_EQ(response->current_pose.orientation.w, 1.0);
}
} // namespace motion_core
