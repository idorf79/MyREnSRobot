#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "moveit/move_group_interface/move_group_interface.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "open_manipulator_controller/action/move_to_position.hpp"
#include "open_manipulator_controller/action/gripper_control.hpp"

namespace open_manipulator_controller
{

class ArmController : public rclcpp::Node
{
public:
  using MoveToPosition    = open_manipulator_controller::action::MoveToPosition;
  using GripperControl    = open_manipulator_controller::action::GripperControl;
  using MoveGoalHandle    = rclcpp_action::ServerGoalHandle<MoveToPosition>;
  using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperControl>;

  explicit ArmController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  // Called from main() AFTER the executor has started spinning.
  // MoveGroupInterface requires the node to already be receiving callbacks.
  void post_init();

private:
  // ── Parameters ────────────────────────────────────────────────────────────
  double feedback_rate_hz_{10.0};
  double goal_timeout_sec_{30.0};
  std::string arm_planning_group_;     // e.g. "arm"
  std::string gripper_planning_group_; // e.g. "gripper"

  // ── MoveIt interfaces (constructed in post_init) ───────────────────────────
  // Unique_ptr so construction can be deferred until after spinning starts.
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_mgi_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_mgi_;

  // ── Live joint state (for feedback) ───────────────────────────────────────
  std::vector<double>         current_joint_positions_{0.0, 0.0, 0.0, 0.0};
  double                      gripper_position_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // ── Action servers ─────────────────────────────────────────────────────────
  rclcpp_action::Server<MoveToPosition>::SharedPtr move_server_;
  rclcpp_action::Server<GripperControl>::SharedPtr gripper_server_;

  // ── Joint state callback ───────────────────────────────────────────────────
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ── MoveToPosition action ──────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_move_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveToPosition::Goal> goal);
  rclcpp_action::CancelResponse handle_move_cancel(
    const std::shared_ptr<MoveGoalHandle> goal_handle);
  void handle_move_accepted(const std::shared_ptr<MoveGoalHandle> goal_handle);
  void execute_move(const std::shared_ptr<MoveGoalHandle> goal_handle);

  // ── GripperControl action ──────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_gripper_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperControl::Goal> goal);
  rclcpp_action::CancelResponse handle_gripper_cancel(
    const std::shared_ptr<GripperGoalHandle> goal_handle);
  void handle_gripper_accepted(const std::shared_ptr<GripperGoalHandle> goal_handle);
  void execute_gripper(const std::shared_ptr<GripperGoalHandle> goal_handle);
};

}  // namespace open_manipulator_controller
