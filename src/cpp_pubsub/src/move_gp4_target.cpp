#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cmath>

#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <motoros2_interfaces/srv/start_traj_mode.hpp>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using JointState = sensor_msgs::msg::JointState;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using SetTrajectoryMode = motoros2_interfaces::srv::StartTrajMode;

class SimpleTrajectoryActionClient : public rclcpp::Node
{
private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Client<SetTrajectoryMode>::SharedPtr traj_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<JointState>::SharedPtr joint_sub_;

  std::vector<double> current_positions_{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  bool got_joint_state_ = false;

public:
  SimpleTrajectoryActionClient() : Node("move_gp4_target")
  {
    // 1. Create service client for start_traj_mode
    traj_client_ = this->create_client<SetTrajectoryMode>("yaskawa/start_traj_mode");

    // 2. Create action client for follow_joint_trajectory
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "yaskawa/follow_joint_trajectory");  // Changed to manipulator/ if necessary based on real robot.

    // 3. Subscribe to joint states
    joint_sub_ = this->create_subscription<JointState>(
        "yaskawa/joint_states",
        rclcpp::QoS(10),  // Reliable QoS for joint states
        std::bind(&SimpleTrajectoryActionClient::joint_state_callback, this, std::placeholders::_1));

    // 4. Start a timer to run the sequence once after node spins
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&SimpleTrajectoryActionClient::start_sequence, this));
  }

private:
  void joint_state_callback(const JointState::SharedPtr msg)
  {
    if (got_joint_state_)
      return;

    std::vector<std::string> target_names = { "joint_1_s", "joint_2_l", "joint_3_u",
                                              "joint_4_r", "joint_5_b", "joint_6_t" };

    for (size_t i = 0; i < target_names.size(); ++i)
    {
      for (size_t j = 0; j < msg->name.size(); ++j)
      {
        if (msg->name[j] == target_names[i])
        {
          current_positions_[i] = msg->position[j];
          break;
        }
      }
    }
    got_joint_state_ = true;
  }

  void start_sequence()
  {
    if (!got_joint_state_)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for /joint_states from the robot...");
      return;
    }

    timer_->cancel();  // Only run once!

    if (!traj_client_->wait_for_service(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(this->get_logger(), "Service yaskawa/start_traj_mode not available.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending StartTrajMode request...");
    auto request = std::make_shared<SetTrajectoryMode::Request>();

    // Send async request with a callback
    traj_client_->async_send_request(
        request, std::bind(&SimpleTrajectoryActionClient::on_traj_mode_response, this, std::placeholders::_1));
  }

  void on_traj_mode_response(rclcpp::Client<SetTrajectoryMode>::SharedFuture future)
  {
    if (future.valid())
    {
      RCLCPP_INFO(this->get_logger(), "start_traj_mode SUCCESS. Proceeding to send trajectory goal...");
      send_goal();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "start_traj_mode FAILED to get valid response.");
      rclcpp::shutdown();
    }
  }

  void send_goal()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available. Terminating...");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory = JointTrajectory();

    goal_msg.trajectory.joint_names = { "joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t" };

    std::vector<double> q0 = current_positions_;
    std::vector<double> q1 = q0;
    std::vector<double> q2 = q0;
    std::vector<double> q3 = q0;

    q1[5] -= M_PI / 9.0;
    q1[0] -= M_PI / 18.0;

    q2[0] += M_PI / 9.0;
    q2[1] += M_PI / 18.0;
    q2[2] += M_PI / 18.0;

    q3[0] -= M_PI / 18.0;
    q3[1] -= M_PI / 18.0;
    q3[2] -= M_PI / 18.0;

    std::vector<double> qdot(goal_msg.trajectory.joint_names.size(), 0.0);
    goal_msg.trajectory.header.stamp = this->now();

    // Add points ...
    goal_msg.trajectory.points.push_back(JointTrajectoryPoint());
    goal_msg.trajectory.points[0].positions = q0;
    goal_msg.trajectory.points[0].velocities = qdot;
    goal_msg.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);

    goal_msg.trajectory.points.push_back(JointTrajectoryPoint());
    goal_msg.trajectory.points[1].positions = q1;
    goal_msg.trajectory.points[1].velocities = qdot;
    goal_msg.trajectory.points[1].time_from_start = rclcpp::Duration(5, 0);

    goal_msg.trajectory.points.push_back(JointTrajectoryPoint());
    goal_msg.trajectory.points[2].positions = q2;
    goal_msg.trajectory.points[2].velocities = qdot;
    goal_msg.trajectory.points[2].time_from_start = rclcpp::Duration(10, 0);

    goal_msg.trajectory.points.push_back(JointTrajectoryPoint());
    goal_msg.trajectory.points[3].positions = q3;
    goal_msg.trajectory.points[3].velocities = qdot;
    goal_msg.trajectory.points[3].time_from_start = rclcpp::Duration(15, 0);

    // Options
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_goal_options.result_callback = [this](const GoalHandleFJT::WrappedResult& result) {
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Trajectory EXECUTION SUCCEEDED!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "Trajectory ABORTED");
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Trajectory FAILED/CANCELED");
          break;
      }
      rclcpp::shutdown();
    };

    send_goal_options.goal_response_callback = [this](const GoalHandleFJT::SharedPtr& goal_handle) {
      if (!goal_handle)
      {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
        rclcpp::shutdown();
      }
      else
      {
        RCLCPP_INFO(get_logger(), "Goal accepted by server, executing trajectory...");
      }
    };

    // Send
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTrajectoryActionClient>();

  // We just spin. The timer inside the node will trigger the sequence.
  rclcpp::spin(node);

  // Shutdown already handled via rclcpp::shutdown() inside callbacks
  return 0;
}
