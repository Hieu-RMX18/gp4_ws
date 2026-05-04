// Copyright 2026 hieu2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * servo_bridge_node.cpp -- Experimental Servo-to-MotoROS2 point-queue bridge.
 *
 * Purpose:
 *   Bridges MoveIt Servo joint trajectory output -> MotoROS2 point-queue mode
 *   via /yaskawa/queue_traj_point, so the HMI jog pendant can drive the GP4
 *   in real time.
 *
 * Execution path:
 *   MoveIt Servo (joint jogging) -> servo_bridge_node ->
 * /yaskawa/queue_traj_point
 *                                             |
 *                                             v
 *                                    MotoROS2 -> YRC1000micro -> GP4
 *
 * NOT a replacement for the FJT mainline path. Strictly exclusive with FJT
 * mode.
 *
 * State machine:
 *   IDLE -> STARTING -> ACTIVE -> HALTED
 *                    \> REJECTED_NOT_READY / REJECTED_FJT_ACTIVE / ERROR
 *
 * Activation flow (self-contained):
 *   1. handle_activate sets state=STARTING, returns success=true immediately
 *   2. If servo not active: call start_servo async -> poll servo_active_
 * (100ms, 5s timeout)
 *   3. On servo_active: call start_point_queue_mode async
 *   4. On PQM success: set state=ACTIVE
 *
 * HARD CONSTRAINTS (fail-closed):
 *   - Robot must be ready before activation
 *   - No FJT trajectory mode must be active (confirmed via result_code)
 *   - Joint limits must be validated before each point
 *   - Rate limiting: max 15 Hz forwarding
 *   - No stop service for point-queue mode: deactivation = soft stop by input
 * withdrawal
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <ratio>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// MotoROS2 point-queue interfaces
#include <motoros2_interfaces/msg/motion_ready_enum.hpp>
#include <motoros2_interfaces/msg/queue_result_enum.hpp>
#include <motoros2_interfaces/srv/queue_traj_point.hpp>
#include <motoros2_interfaces/srv/start_point_queue_mode.hpp>

// GP4 / MotoROS2 robot status
#include <industrial_msgs/msg/robot_status.hpp>

// Our own bridge status
#include <interfaces/msg/servo_bridge_status.hpp>

namespace {
constexpr uint16_t QUEUE_SUCCESS = 1;
constexpr uint16_t QUEUE_BUSY = 4;
constexpr uint16_t MOTION_READY = 1;
constexpr uint16_t MOTION_NOT_READY_OTHER_TRAJ_MODE = 114;

constexpr double MAX_FORWARD_HZ = 15.0;
constexpr double MIN_FORWARD_INTERVAL_SEC = 1.0 / MAX_FORWARD_HZ;

constexpr double JOINT_LIMIT_MARGIN_RAD = 0.05;

// Joint limits (rad) -- conservative for experimental jogging
const std::vector<std::pair<std::string, std::pair<double, double>>>
    GP4_JOINT_LIMITS = {
        {"joint_1_s", {-2.9671, 2.9671}}, {"joint_2_l", {-1.9199, 2.2689}},
        {"joint_3_u", {-1.1345, 3.4907}}, {"joint_4_r", {-3.4907, 3.4907}},
        {"joint_5_b", {-2.1468, 2.1468}}, {"joint_6_t", {-7.9412, 7.9412}},
};

struct JointBounds {
  double min_rad = 0.0;
  double max_rad = 0.0;
};

std::optional<JointBounds> get_joint_bounds(const std::string &joint_name) {
  for (const auto &[name, bounds] : GP4_JOINT_LIMITS) {
    if (name == joint_name) {
      JointBounds b;
      b.min_rad = bounds.first - JOINT_LIMIT_MARGIN_RAD;
      b.max_rad = bounds.second + JOINT_LIMIT_MARGIN_RAD;
      return b;
    }
  }
  return std::nullopt;
}

} // namespace

namespace jog_pendant {

enum class BridgeState {
  IDLE,
  STARTING,
  READY,
  ACTIVE,
  HALTING,
  HALTED,
  ERROR,
  REJECTED_NOT_READY,
  REJECTED_FJT_ACTIVE,
  TIMEOUT,
  BUSY_RETRY,
};

constexpr const char *bridge_state_str(BridgeState s) {
  switch (s) {
  case BridgeState::IDLE:
    return "IDLE";
  case BridgeState::STARTING:
    return "STARTING";
  case BridgeState::READY:
    return "READY";
  case BridgeState::ACTIVE:
    return "ACTIVE";
  case BridgeState::HALTING:
    return "HALTING";
  case BridgeState::HALTED:
    return "HALTED";
  case BridgeState::ERROR:
    return "ERROR";
  case BridgeState::REJECTED_NOT_READY:
    return "REJECTED_NOT_READY";
  case BridgeState::REJECTED_FJT_ACTIVE:
    return "REJECTED_FJT_ACTIVE";
  case BridgeState::TIMEOUT:
    return "TIMEOUT";
  case BridgeState::BUSY_RETRY:
    return "BUSY_RETRY";
  }
  return "UNKNOWN";
}

class ServoBridgeNode : public rclcpp::Node {
public:
  explicit ServoBridgeNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("servo_bridge_node", options), state_(BridgeState::IDLE),
        robot_ready_(false), servo_active_(false), bridge_active_(false),
        points_queued_(0), effective_hz_(0.0), last_error_(""),
        rejection_reason_(""), sim_mode_(false) {
    // -- Clock for stamping
    clock_ = this->get_clock();

    // -- Declare parameters
    declare_parameter("robot_status_topic", "/yaskawa/robot_status");
    declare_parameter("joint_states_topic", "/yaskawa/joint_states");
    declare_parameter("servo_status_topic", "/servo_node/status");
    declare_parameter("servo_trajectory_topic", "/servo_node/command_out");
    declare_parameter("start_point_queue_service",
                      "/yaskawa/start_point_queue_mode");
    declare_parameter("queue_traj_point_service", "/yaskawa/queue_traj_point");
    declare_parameter("status_pub_topic", "/servo_bridge/status");
    declare_parameter("activate_service", "/servo_bridge/activate");
    declare_parameter("deactivate_service", "/servo_bridge/deactivate");
    declare_parameter("forward_rate_hz", 10.0);
    declare_parameter("joint_names",
                      std::vector<std::string>{"joint_1_s", "joint_2_l",
                                               "joint_3_u", "joint_4_r",
                                               "joint_5_b", "joint_6_t"});
    declare_parameter("busy_retry_max", 3);
    declare_parameter("busy_retry_interval_ms", 50);
    declare_parameter("activation_timeout_ms", 5000);
    declare_parameter("start_servo_service",
                      "/servo_node/servo_node/start_servo");
    declare_parameter("sim_mode", false);

    robot_status_topic_ = get_parameter("robot_status_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    servo_status_topic_ = get_parameter("servo_status_topic").as_string();
    servo_trajectory_topic_ =
        get_parameter("servo_trajectory_topic").as_string();
    start_point_queue_service_ =
        get_parameter("start_point_queue_service").as_string();
    queue_traj_point_service_ =
        get_parameter("queue_traj_point_service").as_string();
    status_pub_topic_ = get_parameter("status_pub_topic").as_string();
    activate_service_ = get_parameter("activate_service").as_string();
    deactivate_service_ = get_parameter("deactivate_service").as_string();
    forward_rate_hz_ = get_parameter("forward_rate_hz").as_double();
    forward_interval_ms_ =
        static_cast<int32_t>(1000.0 / std::max(0.1, forward_rate_hz_));
    joint_names_ = get_parameter("joint_names").as_string_array();
    busy_retry_max_ = get_parameter("busy_retry_max").as_int();
    busy_retry_interval_ms_ = get_parameter("busy_retry_interval_ms").as_int();
    activation_timeout_ms_ = get_parameter("activation_timeout_ms").as_int();
    start_servo_service_ = get_parameter("start_servo_service").as_string();
    sim_mode_ = get_parameter("sim_mode").as_bool();

    RCLCPP_INFO(
        get_logger(),
        "servo_bridge_node: sim_mode=%s, robot_status=%s, joint_states=%s, "
        "servo_status=%s, servo_trajectory=%s, "
        "start_pqm=%s, queue_traj=%s, forward_rate=%.1fHz",
        sim_mode_ ? "true" : "false", robot_status_topic_.c_str(),
        joint_states_topic_.c_str(), servo_status_topic_.c_str(),
        servo_trajectory_topic_.c_str(), start_point_queue_service_.c_str(),
        queue_traj_point_service_.c_str(), forward_rate_hz_);

    // -- Subscriptions
    // MotoROS2 publishes with BEST_EFFORT reliability (UDP transport).
    // Using default RELIABLE QoS causes silent message drop -- use
    // SensorDataQoS.
    const auto sensor_qos = rclcpp::SensorDataQoS{};

    robot_status_sub_ = create_subscription<industrial_msgs::msg::RobotStatus>(
        robot_status_topic_, sensor_qos,
        [this](const industrial_msgs::msg::RobotStatus::SharedPtr msg) {
          this->on_robot_status(msg);
        });

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, sensor_qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          this->on_joint_states(msg);
        });

    // Servo status is published as std_msgs/Int8 on ~/status
    // 0 = NOMINAL (OK), non-zero = warning/problem
    servo_status_sub_ = create_subscription<std_msgs::msg::Int8>(
        servo_status_topic_, 10,
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
          this->on_servo_status(msg);
        });

    servo_trajectory_sub_ =
        create_subscription<trajectory_msgs::msg::JointTrajectory>(
            servo_trajectory_topic_, 10,
            [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
              this->on_servo_trajectory(msg);
            });

    // -- Publishers
    status_pub_ = create_publisher<interfaces::msg::ServoBridgeStatus>(
        status_pub_topic_, 10);

    // -- Services
    using std_srvs::srv::Trigger;

    activate_srv_ = create_service<Trigger>(
        activate_service_,
        [this](const std::shared_ptr<Trigger::Request> &req,
               const std::shared_ptr<Trigger::Response> &resp) {
          this->handle_activate(req, resp);
        });

    deactivate_srv_ = create_service<Trigger>(
        deactivate_service_,
        [this](const std::shared_ptr<Trigger::Request> &req,
               const std::shared_ptr<Trigger::Response> &resp) {
          this->handle_deactivate(req, resp);
        });

    // -- Service clients
    start_pqm_client_ =
        create_client<motoros2_interfaces::srv::StartPointQueueMode>(
            start_point_queue_service_);
    queue_traj_client_ =
        create_client<motoros2_interfaces::srv::QueueTrajPoint>(
            queue_traj_point_service_);
    start_servo_client_ =
        create_client<std_srvs::srv::Trigger>(start_servo_service_);

    // -- Status publishing timer
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                      [this]() { this->publish_status(); });

    RCLCPP_INFO(get_logger(),
                "servo_bridge_node initialized. "
                "Activate via service: %s",
                activate_service_.c_str());
  }

private:
  // ======================================================================
  // State management
  // ======================================================================

  void set_state(BridgeState new_state, const std::string &error_msg = "") {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    state_ = new_state;
    if (!error_msg.empty()) {
      last_error_ = error_msg;
    }
    RCLCPP_INFO(get_logger(), "bridge state -> %s",
                bridge_state_str(new_state));
  }

  BridgeState current_state() const {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);
    return state_;
  }

  rclcpp::Time now() const { return clock_->now(); }

  // ======================================================================
  // Activation / Deactivation
  // ======================================================================

  using Trigger = std_srvs::srv::Trigger;

  void handle_activate(const std::shared_ptr<Trigger::Request> & /*request*/,
                       const std::shared_ptr<Trigger::Response> &response) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);

    if (state_ == BridgeState::ACTIVE || state_ == BridgeState::STARTING) {
      response->success = false;
      response->message =
          "bridge already " + std::string(bridge_state_str(state_));
      RCLCPP_WARN(get_logger(), "activation rejected: %s",
                  response->message.c_str());
      return;
    }

    if (!robot_ready_) {
      response->success = false;
      response->message = "activation rejected: robot not ready (check drives, "
                          "e-stop, error state)";
      rejection_reason_ = response->message;
      set_state(BridgeState::REJECTED_NOT_READY, response->message);
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    set_state(BridgeState::STARTING);
    rejection_reason_.clear();
    response->success = true;
    response->message = "activation in progress";

    if (servo_active_) {
      begin_start_point_queue_mode();
    } else {
      begin_start_servo();
    }
  }

  void handle_deactivate(const std::shared_ptr<Trigger::Request> & /*request*/,
                         const std::shared_ptr<Trigger::Response> &response) {
    std::lock_guard<std::recursive_mutex> lock(state_mutex_);

    // Cancel any in-progress servo activation poll
    if (servo_poll_timer_) {
      servo_poll_timer_->cancel();
      servo_poll_timer_.reset();
    }

    if (state_ == BridgeState::IDLE) {
      response->success = true;
      response->message = "bridge already IDLE";
      RCLCPP_INFO(get_logger(), "deactivate: %s", response->message.c_str());
      return;
    }

    // Deactivation = soft stop by input withdrawal
    // No hard stop service for point-queue mode.
    bridge_active_ = false;
    points_queued_ = 0;
    effective_hz_ = 0.0;

    set_state(BridgeState::HALTED);
    response->success = true;
    response->message =
        "bridge deactivated. "
        "Note: no hard stop service exists for point-queue mode. "
        "Motion stops by input withdrawal (watchdog <=200ms on "
        "jog_input_node). "
        "Operator must verify robot has stopped.";
    rejection_reason_.clear();
    last_error_.clear();

    RCLCPP_WARN(get_logger(), "deactivated. soft stop by input withdrawal. %s",
                response->message.c_str());
  }

  // ======================================================================
  // Activation helpers -- called from handle_activate, fully async
  // ======================================================================

  void begin_start_servo() {
    if (!start_servo_client_->service_is_ready()) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      rejection_reason_ =
          "start_servo service (" + start_servo_service_ + ") not available";
      set_state(BridgeState::REJECTED_NOT_READY, rejection_reason_);
      RCLCPP_WARN(get_logger(), "%s", rejection_reason_.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "calling start_servo at %s",
                start_servo_service_.c_str());
    auto request = std::make_shared<Trigger::Request>();
    start_servo_client_->async_send_request(
        request, [this](rclcpp::Client<Trigger>::SharedFuture future) {
          auto result = future.get();
          if (!result || !result->success) {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            std::string msg = result ? result->message : "null response";
            rejection_reason_ = "start_servo failed: " + msg;
            set_state(BridgeState::REJECTED_NOT_READY, rejection_reason_);
            RCLCPP_WARN(get_logger(), "%s", rejection_reason_.c_str());
            return;
          }
          RCLCPP_INFO(get_logger(),
                      "start_servo succeeded, polling for servo_active...");
          begin_servo_active_poll();
        });
  }

  void begin_servo_active_poll() {
    servo_poll_count_ = 0;
    const int max_polls = activation_timeout_ms_ / 100;
    servo_poll_timer_ =
        create_wall_timer(std::chrono::milliseconds(100), [this, max_polls]() {
          if (servo_active_.load()) {
            servo_poll_timer_->cancel();
            servo_poll_timer_.reset();
            RCLCPP_INFO(get_logger(), "servo_active confirmed after %d polls",
                        servo_poll_count_);
            begin_start_point_queue_mode();
            return;
          }
          servo_poll_count_++;
          if (servo_poll_count_ >= max_polls) {
            servo_poll_timer_->cancel();
            servo_poll_timer_.reset();
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            rejection_reason_ =
                "timed out waiting for servo_active after start_servo (" +
                std::to_string(activation_timeout_ms_) + "ms)";
            set_state(BridgeState::REJECTED_NOT_READY, rejection_reason_);
            RCLCPP_WARN(get_logger(), "%s", rejection_reason_.c_str());
          }
        });
  }

  void begin_start_point_queue_mode() {
    if (!start_pqm_client_->service_is_ready()) {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      rejection_reason_ = start_point_queue_service_ + " is not available";
      set_state(BridgeState::ERROR, rejection_reason_);
      RCLCPP_ERROR(get_logger(), "activation failed: %s",
                   rejection_reason_.c_str());
      return;
    }

    auto request = std::make_shared<
        motoros2_interfaces::srv::StartPointQueueMode::Request>();
    start_pqm_client_->async_send_request(
        request,
        [this](rclcpp::Client<motoros2_interfaces::srv::StartPointQueueMode>::
                   SharedFuture future) {
          auto result = future.get();
          if (!result) {
            std::lock_guard<std::recursive_mutex> lock(state_mutex_);
            set_state(BridgeState::ERROR,
                      "start_point_queue_mode returned null response");
            return;
          }

          const uint16_t code = result->result_code.value;
          const std::string &msg = result->message;
          std::lock_guard<std::recursive_mutex> lock(state_mutex_);

          if (code == MOTION_READY) {
            rejection_reason_.clear();
            RCLCPP_INFO(get_logger(), "start_point_queue_mode: READY -- %s",
                        msg.c_str());
            set_state(BridgeState::ACTIVE);
            bridge_active_ = true;
            points_queued_ = 0;
            effective_hz_ = 0.0;
            last_forward_time_ = now();
            return;
          }

          if (code == MOTION_NOT_READY_OTHER_TRAJ_MODE) {
            rejection_reason_ = "FJT trajectory mode is active. "
                                "Deactivate FJT before using point-queue mode.";
            set_state(BridgeState::REJECTED_FJT_ACTIVE, rejection_reason_);
            RCLCPP_WARN(get_logger(),
                        "start_point_queue_mode: FJT ACTIVE -- %s",
                        msg.c_str());
            return;
          }

          rejection_reason_ = "start_point_queue_mode failed with code " +
                              std::to_string(code) + ": " + msg;
          set_state(BridgeState::REJECTED_NOT_READY, rejection_reason_);
          RCLCPP_WARN(get_logger(), "start_point_queue_mode: code=%d -- %s",
                      code, msg.c_str());
        });
  }

  // ======================================================================
  // Subscriptions
  // ======================================================================

  void
  on_robot_status(const industrial_msgs::msg::RobotStatus::SharedPtr &msg) {
    // industrial_msgs TriState: -1=UNKNOWN, 0=FALSE, 1=TRUE
    const bool drives_ok = msg->drives_powered.val > 0;
    const bool e_stopped = msg->e_stopped.val > 0;
    const bool in_error = msg->in_error.val > 0;
    const bool motion_ok = msg->motion_possible.val > 0;

    bool was_ready = robot_ready_.load();
    robot_ready_ = drives_ok && !e_stopped && !in_error && motion_ok;

    if (!was_ready && robot_ready_) {
      RCLCPP_INFO(get_logger(),
                  "robot became ready: drives=%d e_stop=%d error=%d motion=%d",
                  drives_ok, e_stopped, in_error, motion_ok);
    } else if (was_ready && !robot_ready_) {
      RCLCPP_WARN(
          get_logger(),
          "robot became NOT ready: drives=%d e_stop=%d error=%d motion=%d",
          drives_ok, e_stopped, in_error, motion_ok);

      BridgeState cs = current_state();
      if (cs == BridgeState::ACTIVE || cs == BridgeState::READY) {
        bridge_active_ = false;
        points_queued_ = 0;
        set_state(BridgeState::ERROR, "robot not ready during active jog");
      }
    }

    last_robot_status_time_ = now();
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr &msg) {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    current_joint_positions_.clear();
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      current_joint_positions_[msg->name[i]] = msg->position[i];
    }
    last_joint_states_time_ = now();
  }

  void on_servo_status(const std_msgs::msg::Int8::SharedPtr &msg) {
    // Servo status: 0 = NOMINAL (OK), non-zero = warning/problem
    bool was_active = servo_active_.load();
    servo_active_ = (msg->data == 0);

    if (!was_active && servo_active_) {
      RCLCPP_INFO(get_logger(), "servo became active (status_code=%d)",
                  msg->data);
    } else if (was_active && !servo_active_) {
      RCLCPP_WARN(get_logger(), "servo became inactive (status_code=%d)",
                  msg->data);

      BridgeState cs = current_state();
      if (cs == BridgeState::ACTIVE) {
        bridge_active_ = false;
        points_queued_ = 0;
        set_state(BridgeState::ERROR, "servo status became inactive");
      }
    }

    last_servo_status_time_ = now();
  }

  void on_servo_trajectory(
      const trajectory_msgs::msg::JointTrajectory::SharedPtr &msg) {
    BridgeState cs = current_state();
    if (cs != BridgeState::ACTIVE || !bridge_active_.load()) {
      return;
    }

    if (msg->points.empty()) {
      return;
    }

    const auto &point = msg->points.back();

    // Rate limiting
    auto now_ts = now();
    auto elapsed = (now_ts - last_forward_time_).seconds();
    if (elapsed < MIN_FORWARD_INTERVAL_SEC) {
      return;
    }

    forward_point_async(msg->joint_names, point);
    last_forward_time_ = now_ts;
    update_effective_hz(elapsed);
  }

  // ======================================================================
  // Point forwarding
  // ======================================================================

  bool validate_point(
      const std::vector<std::string> &joint_names,
      const trajectory_msgs::msg::JointTrajectoryPoint &point) const {
    if (joint_names.size() != joint_names_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
                           "joint_names size mismatch: got %zu, expected %zu",
                           joint_names.size(), joint_names_.size());
      return false;
    }

    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i] != joint_names_[i]) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *clock_, 1000,
            "joint_names mismatch at [%zu]: got '%s', expected '%s'", i,
            joint_names[i].c_str(), joint_names_[i].c_str());
        return false;
      }
    }

    if (point.positions.size() != joint_names_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
                           "positions size mismatch: got %zu, expected %zu",
                           point.positions.size(), joint_names_.size());
      return false;
    }

    for (size_t i = 0; i < joint_names.size(); ++i) {
      auto bounds = get_joint_bounds(joint_names[i]);
      if (bounds && (point.positions[i] < bounds->min_rad ||
                     point.positions[i] > bounds->max_rad)) {
        RCLCPP_WARN(
            get_logger(),
            "joint %s at %.4f rad out of bounds [%.4f, %.4f]; rejecting point",
            joint_names[i].c_str(), point.positions[i], bounds->min_rad,
            bounds->max_rad);
        return false;
      }
    }

    return true;
  }

  void
  forward_point_async(const std::vector<std::string> &joint_names,
                      const trajectory_msgs::msg::JointTrajectoryPoint &point) {
    if (!validate_point(joint_names, point)) {
      set_state(BridgeState::ERROR, "joint validation failed for queued point");
      bridge_active_ = false;
      return;
    }

    if (!queue_traj_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
                           "queue_traj_point service unavailable");
      return;
    }

    auto request =
        std::make_shared<motoros2_interfaces::srv::QueueTrajPoint::Request>();
    request->joint_names = joint_names;
    request->point = point;

    queue_traj_client_->async_send_request(
        request,
        [this](rclcpp::Client<
               motoros2_interfaces::srv::QueueTrajPoint>::SharedFuture future) {
          auto response = future.get();
          if (!response) {
            RCLCPP_WARN_THROTTLE(get_logger(), *clock_, 1000,
                                 "queue_traj_point returned null");
            return;
          }

          const uint16_t code = response->result_code.value;

          if (code == QUEUE_SUCCESS) {
            points_queued_.fetch_add(1, std::memory_order_relaxed);
            return;
          }

          if (code == QUEUE_BUSY) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *clock_, 500,
                "queue_traj_point BUSY, will retry on next trajectory");
            return;
          }

          RCLCPP_WARN(get_logger(), "queue_traj_point failed: code=%d msg=%s",
                      code, response->message.c_str());
          set_state(BridgeState::ERROR,
                    "queue_traj_point failed: " + response->message);
          bridge_active_ = false;
        });
  }

  void update_effective_hz(double elapsed_since_last) {
    if (elapsed_since_last > 0) {
      std::lock_guard<std::mutex> lock(effective_hz_mutex_);
      double inst_hz = 1.0 / elapsed_since_last;
      // Rolling average
      effective_hz_ = 0.7 * effective_hz_ + 0.3 * inst_hz;
    }
  }

  // ======================================================================
  // Status publishing
  // ======================================================================

  void publish_status() {
    BridgeState cs = current_state();
    bool active = bridge_active_.load();
    bool ready = robot_ready_.load();
    bool servo_ok = servo_active_.load();
    int32_t queued = points_queued_.load();
    double hz = 0.0;
    std::string error_copy;
    std::string rejection_copy;
    {
      std::lock_guard<std::recursive_mutex> lock(state_mutex_);
      error_copy = last_error_;
      rejection_copy = rejection_reason_;
    }
    {
      std::lock_guard<std::mutex> lock(effective_hz_mutex_);
      hz = effective_hz_;
    }

    interfaces::msg::ServoBridgeStatus status_msg;
    status_msg.header.stamp = now();
    status_msg.state = bridge_state_str(cs);
    status_msg.points_queued = queued;
    status_msg.effective_hz = hz;
    status_msg.robot_ready = ready;
    status_msg.servo_active = servo_ok;
    status_msg.bridge_active = active;
    status_msg.last_error = error_copy;
    status_msg.rejection_reason = rejection_copy;

    status_pub_->publish(status_msg);
  }

  // ======================================================================
  // Members
  // ======================================================================

  rclcpp::Clock::SharedPtr clock_;

  mutable std::recursive_mutex state_mutex_;
  BridgeState state_;

  std::atomic<bool> robot_ready_;
  std::atomic<bool> servo_active_;
  std::atomic<bool> bridge_active_;
  std::atomic<int32_t> points_queued_;
  std::atomic<double> effective_hz_;

  rclcpp::Time last_forward_time_;
  std::mutex effective_hz_mutex_;

  std::string last_error_;
  std::string rejection_reason_;

  // Topics / services (params)
  std::string robot_status_topic_;
  std::string joint_states_topic_;
  std::string servo_status_topic_;
  std::string servo_trajectory_topic_;
  std::string start_point_queue_service_;
  std::string queue_traj_point_service_;
  std::string status_pub_topic_;
  std::string activate_service_;
  std::string deactivate_service_;
  double forward_rate_hz_;
  int32_t forward_interval_ms_;
  std::vector<std::string> joint_names_;
  int32_t busy_retry_max_;
  int32_t busy_retry_interval_ms_;
  int32_t activation_timeout_ms_;
  std::string start_servo_service_;
  bool sim_mode_;

  // Joint state cache
  mutable std::mutex joint_mutex_;
  std::unordered_map<std::string, double> current_joint_positions_;
  rclcpp::Time last_joint_states_time_;
  rclcpp::Time last_robot_status_time_;
  rclcpp::Time last_servo_status_time_;

  // Subscriptions
  rclcpp::Subscription<industrial_msgs::msg::RobotStatus>::SharedPtr
      robot_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      servo_trajectory_sub_;

  // Publishers
  rclcpp::Publisher<interfaces::msg::ServoBridgeStatus>::SharedPtr status_pub_;

  // Services (server)
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deactivate_srv_;

  // Service clients
  rclcpp::Client<motoros2_interfaces::srv::StartPointQueueMode>::SharedPtr
      start_pqm_client_;
  rclcpp::Client<motoros2_interfaces::srv::QueueTrajPoint>::SharedPtr
      queue_traj_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr servo_poll_timer_;
  int32_t servo_poll_count_{0};
};

} // namespace jog_pendant

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<jog_pendant::ServoBridgeNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
