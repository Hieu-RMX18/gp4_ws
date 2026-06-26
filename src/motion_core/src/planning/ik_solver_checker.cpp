#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ik_solver_checker");
    
    robot_model_loader::RobotModelLoader loader(node, "robot_description");
    auto robot_model = loader.getModel();
    if (!robot_model) {
        RCLCPP_ERROR(node->get_logger(), "No robot model");
        return 1;
    }
    
    auto jmg = robot_model->getJointModelGroup("gp4_arm");
    if (!jmg) {
        RCLCPP_ERROR(node->get_logger(), "No gp4_arm");
        return 1;
    }
    
    auto solver = jmg->getSolverInstance();
    if (solver) {
        RCLCPP_INFO(node->get_logger(), "IK Solver for gp4_arm: %s", solver->getBaseKinematicsPlugin().c_str());
    } else {
        RCLCPP_INFO(node->get_logger(), "No IK solver found for gp4_arm!");
    }
    
    rclcpp::shutdown();
    return 0;
}
