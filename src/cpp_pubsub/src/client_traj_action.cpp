#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "motoros2_interfaces/srv/start_traj_mode.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;
using JointState = sensor_msgs::msg::JointState;
using SetTrajectoryMode = motoros2_interfaces::srv::StartTrajMode;

class RobotController : public rclcpp::Node
{
private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Client<SetTrajectoryMode>::SharedPtr traj_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<JointState>::SharedPtr joint_sub_;

    std::vector<double> current_positions_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool got_joint_state_ = false;

public:
    RobotController() : Node("client_traj_action")
    {
        // 1. Create service client for start_traj_mode
        traj_client_ = this->create_client<SetTrajectoryMode>("yaskawa/start_traj_mode");

        // 2. Create action client for follow_joint_trajectory
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "yaskawa/follow_joint_trajectory");

        // 3. Subscribe to joint states to get current robot position
        joint_sub_ = this->create_subscription<JointState>(
            "yaskawa/joint_states",
            rclcpp::QoS(10),
            std::bind(&RobotController::joint_state_callback, this, std::placeholders::_1));

        // 4. Start a timer to run the sequence once after node spins
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RobotController::start_sequence, this));
    }

private:
    void joint_state_callback(const JointState::SharedPtr msg)
    {
        if (got_joint_state_) return;

        std::vector<std::string> target_names = {
            "joint_1_s", "joint_2_l", "joint_3_u",
            "joint_4_r", "joint_5_b", "joint_6_t"};

        for (size_t i = 0; i < target_names.size(); ++i) {
            for (size_t j = 0; j < msg->name.size(); ++j) {
                if (msg->name[j] == target_names[i]) {
                    current_positions_[i] = msg->position[j];
                    break;
                }
            }
        }
        got_joint_state_ = true;
        RCLCPP_INFO(this->get_logger(), "Got joint states: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    current_positions_[0], current_positions_[1], current_positions_[2],
                    current_positions_[3], current_positions_[4], current_positions_[5]);
    }

    void start_sequence()
    {
        if (!got_joint_state_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Waiting for yaskawa/joint_states from the robot...");
            return;
        }

        timer_->cancel(); // Only run once!

        if (!traj_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service yaskawa/start_traj_mode not available.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sending StartTrajMode request...");
        auto request = std::make_shared<SetTrajectoryMode::Request>();

        traj_client_->async_send_request(
            request,
            std::bind(&RobotController::on_traj_mode_response, this, std::placeholders::_1));
    }

    void on_traj_mode_response(rclcpp::Client<SetTrajectoryMode>::SharedFuture future)
    {
        if (future.valid()) {
            RCLCPP_INFO(this->get_logger(), "start_traj_mode SUCCESS. Sending trajectory goal...");
            moveRobot();
        } else {
            RCLCPP_ERROR(this->get_logger(), "start_traj_mode FAILED.");
            rclcpp::shutdown();
        }
    }

    void moveRobot()
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available. Terminating...");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(get_logger(), "MOVE ROBOT.");

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory = JointTrajectory();

        goal.trajectory.joint_names = {"joint_1_s", "joint_2_l", "joint_3_u",
                                        "joint_4_r", "joint_5_b", "joint_6_t"};

        // Use current position as starting point
        std::vector<double> q0 = current_positions_;
        std::vector<double> q1 = q0;

        // Move joint_5 and joint_6 by 0.1 rad (~5.7 degrees) from current position
        q1[4] += 0.3;
        q1[5] += 0.3;

        std::vector<double> qdot = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        goal.trajectory.header.stamp = this->now();

        // Point 0: current position at t=0s (MUST be 0 for motoros2)
        goal.trajectory.points.push_back(JointTrajectoryPoint());
        goal.trajectory.points[0].positions = q0;
        goal.trajectory.points[0].velocities = qdot;
        goal.trajectory.points[0].time_from_start = rclcpp::Duration(0, 0);

        // Point 1: target position at t=4s
        goal.trajectory.points.push_back(JointTrajectoryPoint());
        goal.trajectory.points[1].positions = q1;
        goal.trajectory.points[1].velocities = qdot;
        goal.trajectory.points[1].time_from_start = rclcpp::Duration(4, 0);

        // Send goal with callbacks
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleFJT::SharedPtr &goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server.");
                    rclcpp::shutdown();
                } else {
                    RCLCPP_INFO(get_logger(), "Goal accepted by the action server.");
                }
            };

        send_goal_options.result_callback =
            [this](const GoalHandleFJT::WrappedResult &result) {
                // Print detailed error info from the result
                RCLCPP_INFO(get_logger(), "Result error_code: %d", result.result->error_code);
                if (!result.result->error_string.empty()) {
                    RCLCPP_INFO(get_logger(), "Result error_string: %s", result.result->error_string.c_str());
                }

                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Goal SUCCEEDED. Robot joints moved successfully.");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal ABORTED by the action server.");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(get_logger(), "Goal CANCELED.");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code.");
                    break;
                }
                rclcpp::shutdown();
            };

        action_client_->async_send_goal(goal, send_goal_options);
        RCLCPP_INFO(get_logger(), "Goal sent! Waiting for result...");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();
    rclcpp::spin(node);
    return 0;
}
