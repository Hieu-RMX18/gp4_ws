/**
 * gp4_monitor.cpp
 * ───────────────
 * Node giám sát trạng thái robot Yaskawa GP4 real-time.
 *
 * Chức năng:
 *   1. Hiển thị trạng thái robot (e-stop, drives, motion, errors)
 *   2. Hiển thị vị trí joint hiện tại (cập nhật mỗi 2 giây)
 *   3. Theo dõi phản hồi khi gửi trajectory (accepted / executing / done)
 *   4. Cảnh báo khi có alarm/error
 *
 * Chạy:
 *   ros2 run cpp_pubsub gp4_monitor
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <industrial_msgs/msg/robot_status.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <iomanip>
#include <sstream>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>

using JointState   = sensor_msgs::msg::JointState;
using RobotStatus  = industrial_msgs::msg::RobotStatus;
using FollowJT     = control_msgs::action::FollowJointTrajectory;

// ─── Helper: chuyển TriState thành chuỗi ─────────────────────────────────
static std::string tri(int8_t val)
{
    switch (val) {
        case  1: return "\033[32mTRUE\033[0m";   // green
        case  0: return "\033[31mFALSE\033[0m";  // red
        default: return "\033[33mUNKNOWN\033[0m"; // yellow
    }
}

// ─── Helper: radian → degree ──────────────────────────────────────────────
static double rad2deg(double r) { return r * 180.0 / M_PI; }

// ═══════════════════════════════════════════════════════════════════════════
class GP4Monitor : public rclcpp::Node
{
public:
    GP4Monitor() : Node("gp4_monitor")
    {
        RCLCPP_INFO(get_logger(),
            "\033[1;36m"
            "╔══════════════════════════════════════════╗\n"
            "║      GP4 Robot Status Monitor v1.0       ║\n"
            "╚══════════════════════════════════════════╝"
            "\033[0m");

        // ── 1. Subscribe: Joint States ──────────────────────────────────
        joint_sub_ = create_subscription<JointState>(
            "joint_states", rclcpp::SensorDataQoS(),
            [this](const JointState::SharedPtr msg) { on_joint_state(msg); });

        // Cũng thử subscribe namespace yaskawa/ (MotoROS2 driver publish ở đây)
        joint_sub_yaskawa_ = create_subscription<JointState>(
            "yaskawa/joint_states", rclcpp::SensorDataQoS(),
            [this](const JointState::SharedPtr msg) { on_joint_state(msg); });

        // ── 2. Subscribe: Robot Status ──────────────────────────────────
        status_sub_ = create_subscription<RobotStatus>(
            "robot_status", rclcpp::SensorDataQoS(),
            [this](const RobotStatus::SharedPtr msg) { on_robot_status(msg); });

        status_sub_yaskawa_ = create_subscription<RobotStatus>(
            "yaskawa/robot_status", rclcpp::SensorDataQoS(),
            [this](const RobotStatus::SharedPtr msg) { on_robot_status(msg); });

        // ── 3. Subscribe: FollowJointTrajectory action feedback ─────────
        // (từ controller gp4_arm_controller hoặc yaskawa namespace)
        fjt_sub_ = create_subscription<FollowJT::Impl::FeedbackMessage>(
            "gp4_arm_controller/follow_joint_trajectory/_action/feedback",
            10,
            [this](const FollowJT::Impl::FeedbackMessage::SharedPtr msg) {
                on_trajectory_feedback(msg);
            });

        fjt_sub_yaskawa_ = create_subscription<FollowJT::Impl::FeedbackMessage>(
            "yaskawa/follow_joint_trajectory/_action/feedback",
            10,
            [this](const FollowJT::Impl::FeedbackMessage::SharedPtr msg) {
                on_trajectory_feedback(msg);
            });

        // Subscribe result topic
        fjt_result_sub_ = create_subscription<FollowJT::Impl::GoalStatusMessage>(
            "gp4_arm_controller/follow_joint_trajectory/_action/status",
            10,
            [this](const FollowJT::Impl::GoalStatusMessage::SharedPtr msg) {
                on_trajectory_status(msg);
            });

        fjt_result_sub_yaskawa_ = create_subscription<FollowJT::Impl::GoalStatusMessage>(
            "yaskawa/follow_joint_trajectory/_action/status",
            10,
            [this](const FollowJT::Impl::GoalStatusMessage::SharedPtr msg) {
                on_trajectory_status(msg);
            });

        // ── 4. Timer: in joint states mỗi 2 giây ──────────────────────
        print_timer_ = create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&GP4Monitor::print_joint_summary, this));

        RCLCPP_INFO(get_logger(),
            "Đang lắng nghe: joint_states, robot_status, trajectory feedback...");
    }

private:
    // ─── Subscriptions ──────────────────────────────────────────────────
    rclcpp::Subscription<JointState>::SharedPtr   joint_sub_;
    rclcpp::Subscription<JointState>::SharedPtr   joint_sub_yaskawa_;
    rclcpp::Subscription<RobotStatus>::SharedPtr  status_sub_;
    rclcpp::Subscription<RobotStatus>::SharedPtr  status_sub_yaskawa_;
    rclcpp::Subscription<FollowJT::Impl::FeedbackMessage>::SharedPtr fjt_sub_;
    rclcpp::Subscription<FollowJT::Impl::FeedbackMessage>::SharedPtr fjt_sub_yaskawa_;
    rclcpp::Subscription<FollowJT::Impl::GoalStatusMessage>::SharedPtr fjt_result_sub_;
    rclcpp::Subscription<FollowJT::Impl::GoalStatusMessage>::SharedPtr fjt_result_sub_yaskawa_;
    rclcpp::TimerBase::SharedPtr print_timer_;

    // ─── State ──────────────────────────────────────────────────────────
    std::mutex mtx_;
    std::vector<std::string> joint_names_;
    std::vector<double>      joint_positions_;
    bool got_joints_ = false;

    // Lưu trạng thái trước đó để chỉ in khi thay đổi
    int8_t prev_e_stopped_     = -99;
    int8_t prev_drives_        = -99;
    int8_t prev_motion_        = -99;
    int8_t prev_in_motion_     = -99;
    int8_t prev_in_error_      = -99;
    int    prev_goal_status_   = -1;

    // ═══════════════════════════════════════════════════════════════════
    // CALLBACK: Joint State
    // ═══════════════════════════════════════════════════════════════════
    void on_joint_state(const JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        joint_names_    = msg->name;
        joint_positions_ = msg->position;
        got_joints_ = true;
    }

    // ═══════════════════════════════════════════════════════════════════
    // CALLBACK: Robot Status  (chỉ in khi trạng thái THAY ĐỔI)
    // ═══════════════════════════════════════════════════════════════════
    void on_robot_status(const RobotStatus::SharedPtr msg)
    {
        bool changed = false;

        if (msg->e_stopped.val     != prev_e_stopped_)  changed = true;
        if (msg->drives_powered.val != prev_drives_)     changed = true;
        if (msg->motion_possible.val != prev_motion_)    changed = true;
        if (msg->in_motion.val      != prev_in_motion_)  changed = true;
        if (msg->in_error.val       != prev_in_error_)   changed = true;

        if (!changed) return;

        prev_e_stopped_  = msg->e_stopped.val;
        prev_drives_     = msg->drives_powered.val;
        prev_motion_     = msg->motion_possible.val;
        prev_in_motion_  = msg->in_motion.val;
        prev_in_error_   = msg->in_error.val;

        std::ostringstream ss;
        ss << "\033[1;35m"
           << "\n┌─────────── ROBOT STATUS CHANGED ───────────┐\n"
           << "\033[0m"
           << "│  E-Stop        : " << tri(msg->e_stopped.val) << "\n"
           << "│  Drives Powered: " << tri(msg->drives_powered.val) << "\n"
           << "│  Motion Possible: " << tri(msg->motion_possible.val) << "\n"
           << "│  In Motion     : " << tri(msg->in_motion.val) << "\n"
           << "│  In Error      : " << tri(msg->in_error.val) << "\n";

        if (!msg->error_codes.empty()) {
            ss << "│  Error Codes   : ";
            for (auto code : msg->error_codes) ss << code << " ";
            ss << "\n";
        }

        ss << "\033[1;35m"
           << "└─────────────────────────────────────────────┘"
           << "\033[0m";

        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    }

    // ═══════════════════════════════════════════════════════════════════
    // CALLBACK: Trajectory Feedback (in tiến trình khi trajectory đang chạy)
    // ═══════════════════════════════════════════════════════════════════
    void on_trajectory_feedback(
        const FollowJT::Impl::FeedbackMessage::SharedPtr msg)
    {
        auto &fb = msg->feedback;

        std::ostringstream ss;
        ss << "\033[1;33m[TRAJECTORY FEEDBACK]\033[0m ";

        // Hiển thị desired vs actual cho mỗi joint
        if (!fb.joint_names.empty()) {
            ss << "Joints: ";
            for (size_t i = 0; i < fb.joint_names.size(); ++i) {
                double desired = (i < fb.desired.positions.size())
                                 ? rad2deg(fb.desired.positions[i]) : 0.0;
                double actual  = (i < fb.actual.positions.size())
                                 ? rad2deg(fb.actual.positions[i]) : 0.0;
                double error   = (i < fb.error.positions.size())
                                 ? rad2deg(fb.error.positions[i]) : 0.0;

                ss << fb.joint_names[i]
                   << " [D:" << std::fixed << std::setprecision(1) << desired
                   << "° A:" << actual
                   << "° E:" << error << "°] ";
            }
        }

        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    }

    // ═══════════════════════════════════════════════════════════════════
    // CALLBACK: Action Goal Status
    // ═══════════════════════════════════════════════════════════════════
    void on_trajectory_status(
        const FollowJT::Impl::GoalStatusMessage::SharedPtr msg)
    {
        for (auto &status : msg->status_list) {
            int8_t s = status.status;
            if (s == prev_goal_status_) continue;
            prev_goal_status_ = s;

            std::string label;
            std::string color;
            switch (s) {
                case 1: label = "ACCEPTED";   color = "\033[34m"; break;
                case 2: label = "EXECUTING";  color = "\033[33m"; break;
                case 4: label = "SUCCEEDED";  color = "\033[32m"; break;
                case 5: label = "CANCELED";   color = "\033[31m"; break;
                case 6: label = "ABORTED";    color = "\033[31m"; break;
                default: label = "STATUS=" + std::to_string(s); color = "\033[37m";
            }

            RCLCPP_INFO(get_logger(),
                "%s[TRAJECTORY %s]%s Goal trajectory status changed",
                color.c_str(), label.c_str(), "\033[0m");
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // TIMER: In tổng hợp vị trí joint mỗi 2 giây
    // ═══════════════════════════════════════════════════════════════════
    void print_joint_summary()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!got_joints_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "\033[33mChờ dữ liệu joint_states từ robot...\033[0m");
            return;
        }

        std::ostringstream ss;
        ss << "\033[36m[JOINTS]\033[0m ";
        for (size_t i = 0; i < joint_names_.size() && i < joint_positions_.size(); ++i) {
            ss << joint_names_[i] << "="
               << std::fixed << std::setprecision(2)
               << rad2deg(joint_positions_[i]) << "°  ";
        }
        RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
    }
};

// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GP4Monitor>());
    rclcpp::shutdown();
    return 0;
}
