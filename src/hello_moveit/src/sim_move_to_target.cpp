#include <memory>
#include <cstdlib>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <thread>

/**
 * sim_move_to_target.cpp
 * ──────────────────────
 * (Bản chạy mô phỏng không kết nối qua MotoROS2 driver)
 * Di chuyển end effector của GP4 đến tọa độ Cartesian (x, y, z) yêu cầu trong RViz.
 *
 * Tính năng an toàn:
 *   - Workspace bounds: giới hạn vùng planning
 *   - Collision object: sàn nhà (z = -0.01) để robot không đâm xuống đất
 *   - Velocity/Acceleration scaling: chạy chậm 10%
 *   - Planning time: 10 giây cho planner tìm đường
 *   - Validation: kiểm tra tọa độ nằm trong workspace
 *
 * Cách dùng:
 *   ros2 run hello_moveit sim_move_to_target -- <x> <y> <z>
 *   ros2 run hello_moveit sim_move_to_target -- <x> <y> <z> <ox> <oy> <oz> <ow>
 *
 * Ví dụ:
 *   ros2 run hello_moveit sim_move_to_target -- 0.3 0.2 0.4
 *   ros2 run hello_moveit sim_move_to_target -- 0.3 0.2 0.4 0.0 0.0 0.707 0.707
 */

// ── Workspace bounds (mét) ──────────────────────────────────────────
static constexpr double WS_X_MIN = -0.8;
static constexpr double WS_X_MAX =  0.8;
static constexpr double WS_Y_MIN = -0.8;
static constexpr double WS_Y_MAX =  0.8;
static constexpr double WS_Z_MIN = -0.05;
static constexpr double WS_Z_MAX =  0.8;

// ── Velocity / Acceleration limits ─────────────────────────────────
static constexpr double VELOCITY_SCALE     = 0.1;   // 10% tối đa
static constexpr double ACCELERATION_SCALE = 0.1;   // 10% tối đa
static constexpr double PLANNING_TIME      = 10.0;  // giây

void print_usage()
{
  std::printf(
      "\n"
      "Usage:\n"
      "  ros2 run hello_moveit sim_move_to_target -- <x> <y> <z>\n"
      "  ros2 run hello_moveit sim_move_to_target -- <x> <y> <z> <ox> <oy> <oz> <ow>\n"
      "\n"
      "  x, y, z      : Tọa độ end effector (mét)\n"
      "  ox, oy, oz, ow: Orientation quaternion (mặc định: 0 0 0 1 = hướng xuống)\n"
      "\n"
      "Ví dụ:\n"
      "  ros2 run hello_moveit sim_move_to_target -- 0.3 0.2 0.4\n"
      "  ros2 run hello_moveit sim_move_to_target -- 0.3 0.2 0.4 0.0 0.0 0.707 0.707\n"
      "\n");
}

bool validate_target(double x, double y, double z, rclcpp::Logger logger)
{
  bool valid = true;
  if (x < WS_X_MIN || x > WS_X_MAX) {
    RCLCPP_ERROR(logger, "x = %.3f nằm ngoài workspace [%.2f, %.2f]!", x, WS_X_MIN, WS_X_MAX);
    valid = false;
  }
  if (y < WS_Y_MIN || y > WS_Y_MAX) {
    RCLCPP_ERROR(logger, "y = %.3f nằm ngoài workspace [%.2f, %.2f]!", y, WS_Y_MIN, WS_Y_MAX);
    valid = false;
  }
  if (z < WS_Z_MIN || z > WS_Z_MAX) {
    RCLCPP_ERROR(logger, "z = %.3f nằm ngoài workspace [%.2f, %.2f]!", z, WS_Z_MIN, WS_Z_MAX);
    valid = false;
  }
  return valid;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // ── Khởi tạo Node với NodeOptions ────────────────────────────────
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("use_sim_time", true);

  // ── Khai báo tham số kinematics cho MoveIt ──────────────────────
  // Khi chạy ros2 run (không qua launch file), MoveIt không có
  // kinematics.yaml → cần declare trực tiếp trên node.
  node_options.append_parameter_override(
      "robot_description_kinematics.gp4_arm.kinematics_solver",
      "kdl_kinematics_plugin/KDLKinematicsPlugin");
  node_options.append_parameter_override(
      "robot_description_kinematics.gp4_arm.kinematics_solver_search_resolution",
      0.005);
  node_options.append_parameter_override(
      "robot_description_kinematics.gp4_arm.kinematics_solver_timeout",
      0.005);

  auto node = rclcpp::Node::make_shared("sim_move_to_target", node_options);
  auto const logger = node->get_logger();

  // ── Quan trọng: Spin node trong một luồng riêng để nhận joint_states ──
  // MoveIt cần nhận joint_states liên tục để biết vị trí hiện tại.
  // Nếu main thread bị block bởi plan(), callback joint_states sẽ không chạy.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner_thread([&executor]() { executor.spin(); });

  // ── 1. Parse command-line arguments ───────────────────────────────
  // ROS 2 strips its own args; remaining args after "--" stay in argv
  // We expect: program_name x y z [ox oy oz ow]
  if (argc < 4) {
    RCLCPP_ERROR(logger, "Thiếu tọa độ! Cần ít nhất 3 tham số: x y z");
    print_usage();
    rclcpp::shutdown();
    return 1;
  }

  double target_x  = std::atof(argv[1]);
  double target_y  = std::atof(argv[2]);
  double target_z  = std::atof(argv[3]);

  // Orientation: mặc định quaternion (0, 0, 0, 1) = identity
  double orient_x = 0.0, orient_y = 0.0, orient_z = 0.0, orient_w = 1.0;
  if (argc >= 8) {
    orient_x = std::atof(argv[4]);
    orient_y = std::atof(argv[5]);
    orient_z = std::atof(argv[6]);
    orient_w = std::atof(argv[7]);
  }

  RCLCPP_INFO(logger, "═══════════════════════════════════════════════════");
  RCLCPP_INFO(logger, "  MOVE TO TARGET");
  RCLCPP_INFO(logger, "  Position:    (%.4f, %.4f, %.4f)", target_x, target_y, target_z);
  RCLCPP_INFO(logger, "  Orientation: (%.4f, %.4f, %.4f, %.4f)", orient_x, orient_y, orient_z, orient_w);
  RCLCPP_INFO(logger, "═══════════════════════════════════════════════════");

  // ── 2. Validate target nằm trong workspace ───────────────────────
  if (!validate_target(target_x, target_y, target_z, logger)) {
    RCLCPP_ERROR(logger, "Target nằm ngoài workspace! Hủy bỏ.");
    RCLCPP_ERROR(logger, "Workspace: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
                 WS_X_MIN, WS_X_MAX, WS_Y_MIN, WS_Y_MAX, WS_Z_MIN, WS_Z_MAX);
    executor.cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // ── 3. (Bỏ qua Bật Trajectory Mode MotoROS2 vì đây là mô phỏng) ──

  // ── 4. Khởi tạo MoveGroupInterface ──────────────────────────────
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "gp4_arm");

  // ── 5. In vị trí hiện tại của end effector ───────────────────────
  //
  // Robot biết vị trí hiện tại bằng cách:
  //   - Đọc joint_states từ /yaskawa/joint_states (encoder các khớp)
  //   - MoveIt dùng Forward Kinematics (FK) để tính pose end effector
  //   - getCurrentPose() trả về geometry_msgs::PoseStamped
  //
  auto current_pose = move_group.getCurrentPose();
  RCLCPP_INFO(logger, "───────────────────────────────────────────────────");
  RCLCPP_INFO(logger, "  VỊ TRÍ HIỆN TẠI (End Effector):");
  RCLCPP_INFO(logger, "    Position:    (%.4f, %.4f, %.4f)",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
  RCLCPP_INFO(logger, "    Orientation: (%.4f, %.4f, %.4f, %.4f)",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);
  RCLCPP_INFO(logger, "    Frame:       %s", current_pose.header.frame_id.c_str());
  RCLCPP_INFO(logger, "───────────────────────────────────────────────────");

  // ── 6. Thiết lập Workspace bounds ────────────────────────────────
  //
  // setWorkspace() giới hạn vùng mà planner được phép tìm đường.
  // Nếu end effector hoặc link nào của robot đi ra ngoài vùng này,
  // planner sẽ không chấp nhận → tránh va chạm ngoài vùng an toàn.
  //
  move_group.setWorkspace(WS_X_MIN, WS_Y_MIN, WS_Z_MIN,
                          WS_X_MAX, WS_Y_MAX, WS_Z_MAX);
  RCLCPP_INFO(logger, "[SAFETY] Workspace: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]",
              WS_X_MIN, WS_X_MAX, WS_Y_MIN, WS_Y_MAX, WS_Z_MIN, WS_Z_MAX);

  // ── 7. Thêm Collision Objects (sàn nhà) ─────────────────────────
  //
  // Tạo một hộp lớn phẳng ở z = -0.01 đại diện cho sàn nhà.
  // MoveIt sẽ plan đường đi tránh va vào sàn.
  //
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  {
    moveit_msgs::msg::CollisionObject floor_object;
    floor_object.header.frame_id = move_group.getPlanningFrame();
    floor_object.id = "floor";

    shape_msgs::msg::SolidPrimitive floor_box;
    floor_box.type = floor_box.BOX;
    floor_box.dimensions.resize(3);
    floor_box.dimensions[floor_box.BOX_X] = 2.0;   // 2m rộng
    floor_box.dimensions[floor_box.BOX_Y] = 2.0;   // 2m dài
    floor_box.dimensions[floor_box.BOX_Z] = 0.02;  // 2cm dày

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -0.01;  // Ngay dưới base robot

    floor_object.primitives.push_back(floor_box);
    floor_object.primitive_poses.push_back(floor_pose);
    floor_object.operation = floor_object.ADD;

    planning_scene_interface.applyCollisionObject(floor_object);
    RCLCPP_INFO(logger, "[SAFETY] Added floor collision object at z = -0.01");
  }

  // ── 8. Thiết lập tốc độ & planning ──────────────────────────────
  move_group.setMaxVelocityScalingFactor(VELOCITY_SCALE);
  move_group.setMaxAccelerationScalingFactor(ACCELERATION_SCALE);
  move_group.setPlanningTime(PLANNING_TIME);
  RCLCPP_INFO(logger, "[SAFETY] Velocity: %.0f%%, Acceleration: %.0f%%, Planning time: %.1fs",
              VELOCITY_SCALE * 100, ACCELERATION_SCALE * 100, PLANNING_TIME);

  // ── 9. Đặt Target Pose ──────────────────────────────────────────
  //
  // setPoseTarget() nhận geometry_msgs::Pose chứa:
  //   - position (x, y, z): vị trí end effector trong base frame
  //   - orientation (quaternion): hướng end effector
  //
  // MoveIt sẽ dùng Inverse Kinematics (IK) để tìm
  // giá trị các khớp tương ứng, rồi dùng OMPL planner
  // để tìm đường đi từ vị trí hiện tại đến vị trí đích.
  //
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = target_x;
  target_pose.position.y = target_y;
  target_pose.position.z = target_z;
  target_pose.orientation.x = orient_x;
  target_pose.orientation.y = orient_y;
  target_pose.orientation.z = orient_z;
  target_pose.orientation.w = orient_w;

  // ── 9. Tìm nghiệm IK tối ưu (tránh lật khớp) ──────────────────────────
  // Lấy trạng thái khớp hiện tại làm mốc hạt giống
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(1.0);
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup("gp4_arm");

  // Tính IK cho target_pose, xuất phát từ current_state với timeout 1.0 giây
  bool found_ik = current_state->setFromIK(joint_model_group, target_pose, 1.0);

  if (found_ik) {
    RCLCPP_INFO(logger, "Tìm thấy cấu hình khớp gần nhất. Thiết lập JointTarget.");
    move_group.setJointValueTarget(*current_state);
  } else {
    RCLCPP_ERROR(logger, "══════════════════════════════════════════════");
    RCLCPP_ERROR(logger, "Không tìm thấy nghiệm IK cho tọa độ / orientation này!");
    RCLCPP_ERROR(logger, "══════════════════════════════════════════════");
    executor.cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // Chuyển sang planner rrtstar ưu tiên đường đi ngắn nhất thay vì ngẫu nhiên
  move_group.setPlannerId("RRTstar");
  RCLCPP_INFO(logger, "Target set: (%.4f, %.4f, %.4f) orientation (%.4f, %.4f, %.4f, %.4f)",
              target_x, target_y, target_z, orient_x, orient_y, orient_z, orient_w);

  // ── 10. Planning ─────────────────────────────────────────────────
  RCLCPP_INFO(logger, "Planning path to target...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));

  if (!success) {
    RCLCPP_ERROR(logger, "══════════════════════════════════════════════");
    RCLCPP_ERROR(logger, "  PLANNING FAILED!");
    RCLCPP_ERROR(logger, "  Có thể do:");
    RCLCPP_ERROR(logger, "    - Target ngoài tầm với");
    RCLCPP_ERROR(logger, "    - Collision với vật cản");
    RCLCPP_ERROR(logger, "    - Orientation không khả thi");
    RCLCPP_ERROR(logger, "══════════════════════════════════════════════");
    executor.cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // ── 11. Execute ──────────────────────────────────────────────────
  RCLCPP_INFO(logger, "Planning OK! Executing trajectory...");
  auto exec_result = move_group.execute(plan);

  if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
    // In vị trí sau khi di chuyển
    auto final_pose = move_group.getCurrentPose();
    RCLCPP_INFO(logger, "═══════════════════════════════════════════════════");
    RCLCPP_INFO(logger, "  DI CHUYỂN THÀNH CÔNG!");
    RCLCPP_INFO(logger, "  VỊ TRÍ SAU KHI DI CHUYỂN:");
    RCLCPP_INFO(logger, "    Position:    (%.4f, %.4f, %.4f)",
                final_pose.pose.position.x,
                final_pose.pose.position.y,
                final_pose.pose.position.z);
    RCLCPP_INFO(logger, "    Orientation: (%.4f, %.4f, %.4f, %.4f)",
                final_pose.pose.orientation.x,
                final_pose.pose.orientation.y,
                final_pose.pose.orientation.z,
                final_pose.pose.orientation.w);
    RCLCPP_INFO(logger, "═══════════════════════════════════════════════════");
  } else {
    RCLCPP_ERROR(logger, "Execution FAILED! Error code: %d", exec_result.val);
  }

  // Sạch sẽ tắt node và dọn dẹp luồng con
  executor.cancel();
  if (spinner_thread.joinable()) {
    spinner_thread.join();
  }
  rclcpp::shutdown();
  return 0;
}
