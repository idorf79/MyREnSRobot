#include "open_manipulator_controller/arm_controller.hpp"
#include "open_manipulator_controller/predefined_positions.hpp"

#include <chrono>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

namespace open_manipulator_controller
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor — parameters and action servers only.
// MoveGroupInterface is NOT constructed here; see post_init().
// ─────────────────────────────────────────────────────────────────────────────

ArmController::ArmController(const rclcpp::NodeOptions & options)
: Node("arm_controller", options)
{
  declare_parameter("feedback_rate_hz",       10.0);
  declare_parameter("goal_timeout_sec",       30.0);
  declare_parameter("arm_planning_group",     std::string("arm"));
  declare_parameter("gripper_planning_group", std::string("gripper"));

  feedback_rate_hz_       = get_parameter("feedback_rate_hz").as_double();
  goal_timeout_sec_       = get_parameter("goal_timeout_sec").as_double();
  arm_planning_group_     = get_parameter("arm_planning_group").as_string();
  gripper_planning_group_ = get_parameter("gripper_planning_group").as_string();

  // Live joint positions for action feedback
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&ArmController::joint_state_callback, this, std::placeholders::_1));

  // Action servers — safe to create before MoveIt is ready because
  // execute_move / execute_gripper check arm_mgi_ before using it.
  move_server_ = rclcpp_action::create_server<MoveToPosition>(
    this, "move_to_position",
    std::bind(&ArmController::handle_move_goal,     this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ArmController::handle_move_cancel,   this, std::placeholders::_1),
    std::bind(&ArmController::handle_move_accepted, this, std::placeholders::_1));

  gripper_server_ = rclcpp_action::create_server<GripperControl>(
    this, "gripper_control",
    std::bind(&ArmController::handle_gripper_goal,     this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ArmController::handle_gripper_cancel,   this, std::placeholders::_1),
    std::bind(&ArmController::handle_gripper_accepted, this, std::placeholders::_1));

  std::ostringstream oss;
  for (const auto & [name, _] : predefined_positions()) { oss << name << " "; }
  RCLCPP_INFO(get_logger(),
    "ArmController constructed. Call post_init() after executor starts. "
    "Positions: [ %s]", oss.str().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// post_init — called from main() after the executor is spinning.
// MoveGroupInterface connects to /move_group, which requires active callbacks.
// ─────────────────────────────────────────────────────────────────────────────

void ArmController::post_init()
{
  RCLCPP_INFO(get_logger(), "Connecting to MoveIt move_group...");

  arm_mgi_ = std::make_unique<MoveGroupInterface>(
    shared_from_this(), arm_planning_group_);

  gripper_mgi_ = std::make_unique<MoveGroupInterface>(
    shared_from_this(), gripper_planning_group_);

  // Apply sensible defaults — override via SRDF or parameters if needed.
  arm_mgi_->setMaxVelocityScalingFactor(0.5);
  arm_mgi_->setMaxAccelerationScalingFactor(0.5);
  arm_mgi_->setPlanningTime(5.0);

  gripper_mgi_->setMaxVelocityScalingFactor(1.0);
  gripper_mgi_->setMaxAccelerationScalingFactor(1.0);
  gripper_mgi_->setPlanningTime(2.0);

  RCLCPP_INFO(get_logger(),
    "MoveIt ready. Arm group: '%s'  Gripper group: '%s'",
    arm_planning_group_.c_str(), gripper_planning_group_.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// Joint state callback — populates feedback fields
// ─────────────────────────────────────────────────────────────────────────────

void ArmController::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const std::vector<std::string> arm_joints =
    {"joint1", "joint2", "joint3", "joint4"};

  for (size_t i = 0; i < arm_joints.size(); ++i) {
    for (size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == arm_joints[i] && i < current_joint_positions_.size()) {
        current_joint_positions_[i] = msg->position[j];
      }
    }
  }
  double gripper_left = gripper_position_, gripper_right = gripper_position_;
  bool found_left = false, found_right = false;
  for (size_t j = 0; j < msg->name.size(); ++j) {
    if (msg->name[j] == "gripper_left_joint")  { gripper_left  = msg->position[j]; found_left  = true; }
    if (msg->name[j] == "gripper_right_joint") { gripper_right = msg->position[j]; found_right = true; }
  }
  if (found_left && found_right) {
    gripper_position_ = (gripper_left + gripper_right) / 2.0;
  } else if (found_left) {
    gripper_position_ = gripper_left;
  } else if (found_right) {
    gripper_position_ = gripper_right;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// MoveToPosition action
// ─────────────────────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse ArmController::handle_move_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const MoveToPosition::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received move goal: '%s'", goal->position_name.c_str());

  if (!arm_mgi_) {
    RCLCPP_WARN(get_logger(), "MoveIt not ready yet — rejecting.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (predefined_positions().find(goal->position_name) == predefined_positions().end()) {
    RCLCPP_WARN(get_logger(), "Unknown position '%s' — rejecting.", goal->position_name.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmController::handle_move_cancel(
  const std::shared_ptr<MoveGoalHandle> /*gh*/)
{
  RCLCPP_INFO(get_logger(), "Move cancel requested — stopping MoveIt.");
  if (arm_mgi_) {
    arm_mgi_->stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmController::handle_move_accepted(const std::shared_ptr<MoveGoalHandle> gh)
{
  std::thread([this, gh]() { execute_move(gh); }).detach();
}

void ArmController::execute_move(const std::shared_ptr<MoveGoalHandle> gh)
{
  const auto   goal     = gh->get_goal();
  const auto & name     = goal->position_name;
  const double duration = (goal->duration > 0.0) ? goal->duration : DEFAULT_DURATION;

  RCLCPP_INFO(get_logger(), "Planning move to '%s'", name.c_str());

  auto result = std::make_shared<MoveToPosition::Result>();
  const auto & joint_positions = predefined_positions().at(name);

  // ── Set target ─────────────────────────────────────────────────────────────
  // MoveGroupInterface::setJointValueTarget takes a map of joint_name → value.
  const std::vector<std::string> joint_names =
    {"joint1", "joint2", "joint3", "joint4"};

  std::map<std::string, double> target_map;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    target_map[joint_names[i]] = joint_positions[i];
  }

  if (!arm_mgi_->setJointValueTarget(target_map)) {
    result->success = false;
    result->message = "setJointValueTarget failed — values may be out of bounds.";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    gh->abort(result);
    return;
  }

  // Scale velocity to honour the requested duration.
  // MoveIt doesn't take a fixed duration directly; we approximate by
  // scaling velocity proportionally (clamped to [0.05, 1.0]).
  const double default_dur = 4.0;  // rough estimate for a full-range move
  const double vel_scale   = std::clamp(default_dur / duration, 0.05, 1.0);
  arm_mgi_->setMaxVelocityScalingFactor(vel_scale);

  // ── Plan ───────────────────────────────────────────────────────────────────
  if (gh->is_canceling()) {
    result->success = false;
    result->message = "Cancelled before planning.";
    gh->canceled(result);
    return;
  }

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_mgi_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = false;
    result->message = "MoveIt planning failed (error " +
      std::to_string(plan_result.val) + ").";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    gh->abort(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Plan found, executing...");

  // ── Publish feedback in a side thread during execution ─────────────────────
  std::atomic<bool> exec_done{false};
  const auto planned_duration =
    plan.trajectory.joint_trajectory.points.empty() ? duration :
    rclcpp::Duration(
      plan.trajectory.joint_trajectory.points.back().time_from_start).seconds();

  auto fb_thread = std::thread([&]() {
    const auto sleep_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / feedback_rate_hz_));
    const auto start = std::chrono::steady_clock::now();
    while (!exec_done.load()) {
      if (gh->is_canceling()) break;
      double elapsed  = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      double progress = std::min(elapsed / planned_duration, 0.99);
      auto fb = std::make_shared<MoveToPosition::Feedback>();
      fb->progress                = progress;
      fb->current_joint_positions = current_joint_positions_;
      fb->status                  = "MOVING";
      gh->publish_feedback(fb);
      std::this_thread::sleep_for(sleep_ns);
    }
  });

  // ── Execute ────────────────────────────────────────────────────────────────
  // MoveGroupInterface::execute() blocks until the trajectory completes.
  const auto exec_result = arm_mgi_->execute(plan);

  exec_done.store(true);
  if (fb_thread.joinable()) fb_thread.join();

  // ── Result ─────────────────────────────────────────────────────────────────
  if (gh->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during execution.";
    gh->canceled(result);
    return;
  }

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = false;
    result->message = "MoveIt execution failed (error " +
      std::to_string(exec_result.val) + ").";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    gh->abort(result);
    return;
  }

  result->success               = true;
  result->message               = "Reached '" + name + "'.";
  result->final_joint_positions = joint_positions;
  gh->succeed(result);
  RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// GripperControl action
// ─────────────────────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse ArmController::handle_gripper_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GripperControl::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received gripper goal: '%s'", goal->command.c_str());
  if (!gripper_mgi_) {
    RCLCPP_WARN(get_logger(), "MoveIt not ready yet — rejecting.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->command != "open" && goal->command != "close") {
    RCLCPP_WARN(get_logger(), "Invalid command '%s' — rejecting.", goal->command.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmController::handle_gripper_cancel(
  const std::shared_ptr<GripperGoalHandle> /*gh*/)
{
  RCLCPP_INFO(get_logger(), "Gripper cancel requested — stopping MoveIt.");
  if (gripper_mgi_) {
    gripper_mgi_->stop();
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmController::handle_gripper_accepted(const std::shared_ptr<GripperGoalHandle> gh)
{
  std::thread([this, gh]() { execute_gripper(gh); }).detach();
}

void ArmController::execute_gripper(const std::shared_ptr<GripperGoalHandle> gh)
{
  const auto   goal    = gh->get_goal();
  const bool   is_open = (goal->command == "open");
  const double target  = is_open ? GRIPPER_OPEN : GRIPPER_CLOSED;

  RCLCPP_INFO(get_logger(), "Gripper %s (target=%.4f)", goal->command.c_str(), target);

  auto result = std::make_shared<GripperControl::Result>();

  // ── Set target ─────────────────────────────────────────────────────────────
  // Both gripper_left_joint and gripper_right_joint move in the same direction
  // on the OpenManipulator-X — positive values open the gripper.
  if (!gripper_mgi_->setJointValueTarget({
      {"gripper_left_joint",  target},
      {"gripper_right_joint", target}}))
  {
    result->success = false;
    result->message = "Gripper setJointValueTarget failed.";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    gh->abort(result);
    return;
  }

  if (gh->is_canceling()) {
    result->success = false;
    result->message = "Cancelled before planning.";
    gh->canceled(result);
    return;
  }

  // ── Plan ───────────────────────────────────────────────────────────────────
  MoveGroupInterface::Plan plan;
  const auto plan_result = gripper_mgi_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = false;
    result->message = "Gripper planning failed (error " +
      std::to_string(plan_result.val) + ").";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    gh->abort(result);
    return;
  }

  // ── Feedback thread ────────────────────────────────────────────────────────
  std::atomic<bool> exec_done{false};
  auto fb_thread = std::thread([&]() {
    while (!exec_done.load()) {
      if (gh->is_canceling()) break;
      auto fb = std::make_shared<GripperControl::Feedback>();
      fb->current_position = gripper_position_;
      fb->status           = "MOVING";
      gh->publish_feedback(fb);
      std::this_thread::sleep_for(100ms);
    }
  });

  // ── Execute ────────────────────────────────────────────────────────────────
  const auto exec_result = gripper_mgi_->execute(plan);

  exec_done.store(true);
  if (fb_thread.joinable()) fb_thread.join();

  if (gh->is_canceling()) {
    result->success = false;
    result->message = "Cancelled during gripper execution.";
    gh->canceled(result);
    return;
  }

  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = false;
    result->message = "Gripper execution failed (error " +
      std::to_string(exec_result.val) + ").";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    gh->abort(result);
    return;
  }

  result->success        = true;
  result->message        = "Gripper " + goal->command + "ed.";
  result->final_position = target;
  gh->succeed(result);
  RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
}

}  // namespace open_manipulator_controller
