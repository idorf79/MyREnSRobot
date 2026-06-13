#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "open_manipulator_controller/action/move_to_position.hpp"
#include "open_manipulator_controller/action/gripper_control.hpp"

using namespace std::chrono_literals;
using MoveToPosition = open_manipulator_controller::action::MoveToPosition;
using GripperControl = open_manipulator_controller::action::GripperControl;

// ─────────────────────────────────────────────────────────────────────────────

class ArmClientExample : public rclcpp::Node
{
public:
  explicit ArmClientExample(rclcpp::executors::SingleThreadedExecutor & executor)
  : Node("arm_client_example"), executor_(executor)
  {
    move_client_    = rclcpp_action::create_client<MoveToPosition>(this, "move_to_position");
    gripper_client_ = rclcpp_action::create_client<GripperControl>(this, "gripper_control");
  }

  // ── move_to ───────────────────────────────────────────────────────────────

  bool move_to(const std::string & position_name, double duration = 2.0)
  {
    if (!move_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "move_to_position server not available.");
      return false;
    }

    MoveToPosition::Goal goal;
    goal.position_name = position_name;
    goal.duration      = duration;

    RCLCPP_INFO(get_logger(), "-> move_to('%s', %.1f s)", position_name.c_str(), duration);

    bool accepted  = false;
    bool completed = false;
    bool success   = false;

    auto send_goal_options = rclcpp_action::Client<MoveToPosition>::SendGoalOptions{};

    send_goal_options.goal_response_callback =
      [&accepted](const rclcpp_action::ClientGoalHandle<MoveToPosition>::SharedPtr & gh) {
        accepted = (gh != nullptr);
      };

    send_goal_options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<MoveToPosition>::SharedPtr,
        const std::shared_ptr<const MoveToPosition::Feedback> fb)
      {
        RCLCPP_INFO(get_logger(), "  progress=%.0f%%  state=%s",
          fb->progress * 100.0, fb->status.c_str());
      };

    send_goal_options.result_callback =
      [&completed, &success](
        const rclcpp_action::ClientGoalHandle<MoveToPosition>::WrappedResult & wr)
      {
        success   = wr.result->success;
        completed = true;
      };

    move_client_->async_send_goal(goal, send_goal_options);

    // Spin until goal is accepted
    while (!accepted && rclcpp::ok()) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    if (!accepted) {
      RCLCPP_ERROR(get_logger(), "Move goal rejected.");
      return false;
    }

    // Spin until result arrives
    while (!completed && rclcpp::ok()) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }

    if (success) {
      RCLCPP_INFO(get_logger(), "  Move succeeded.");
    } else {
      RCLCPP_ERROR(get_logger(), "  Move failed.");
    }
    return success;
  }

  // ── gripper ───────────────────────────────────────────────────────────────

  bool gripper(const std::string & command, double effort = 1.0)
  {
    if (!gripper_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "gripper_control server not available.");
      return false;
    }

    GripperControl::Goal goal;
    goal.command = command;
    goal.effort  = effort;

    RCLCPP_INFO(get_logger(), "-> gripper('%s')", command.c_str());

    bool accepted  = false;
    bool completed = false;
    bool success   = false;

    auto send_goal_options = rclcpp_action::Client<GripperControl>::SendGoalOptions{};

    send_goal_options.goal_response_callback =
      [&accepted](const rclcpp_action::ClientGoalHandle<GripperControl>::SharedPtr & gh) {
        accepted = (gh != nullptr);
      };

    send_goal_options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<GripperControl>::SharedPtr,
        const std::shared_ptr<const GripperControl::Feedback> fb)
      {
        RCLCPP_INFO(get_logger(), "  pos=%.4f  status=%s",
          fb->current_position, fb->status.c_str());
      };

    send_goal_options.result_callback =
      [&completed, &success](
        const rclcpp_action::ClientGoalHandle<GripperControl>::WrappedResult & wr)
      {
        success   = wr.result->success;
        completed = true;
      };

    gripper_client_->async_send_goal(goal, send_goal_options);

    while (!accepted && rclcpp::ok()) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    if (!accepted) {
      RCLCPP_ERROR(get_logger(), "Gripper goal rejected.");
      return false;
    }

    while (!completed && rclcpp::ok()) {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }

    if (success) {
      RCLCPP_INFO(get_logger(), "  Gripper succeeded.");
    } else {
      RCLCPP_ERROR(get_logger(), "  Gripper failed.");
    }
    return success;
  }

  // ── Demo pick-and-place sequence ──────────────────────────────────────────

  void run_demo()
  {
    RCLCPP_INFO(get_logger(), "=== Starting pick-and-place demo ===");

    struct Step {
      std::string            label;
      std::function<bool()>  fn;
    };

    std::vector<Step> steps = {
      {"Move home",        [this] { return move_to("home",        2.0); }},
      {"Open gripper",     [this] { return gripper("open");             }},
      {"Move pick_left",   [this] { return move_to("pick_left",   2.5); }},
      {"Close gripper",    [this] { return gripper("close");            }},
      {"Move safe",        [this] { return move_to("safe",        2.0); }},
      {"Move place_front", [this] { return move_to("place_front", 2.5); }},
      {"Open gripper",     [this] { return gripper("open");             }},
      {"Return home",      [this] { return move_to("home",        2.0); }},
      {"Move pick_right",  [this] { return move_to("pick_right",  2.5); }},
      {"Close gripper",    [this] { return gripper("close");            }},
      {"Move safe",        [this] { return move_to("safe",        2.0); }},
      {"Move place_front", [this] { return move_to("place_front", 2.5); }},
      {"Open gripper",     [this] { return gripper("open");             }},
      {"Return home",      [this] { return move_to("home",        2.0); }},
    };

    for (const auto & step : steps) {
      RCLCPP_INFO(get_logger(), "-- %s", step.label.c_str());
      if (!step.fn()) {
        RCLCPP_ERROR(get_logger(), "Demo aborted at step: %s", step.label.c_str());
        return;
      }
    }

    RCLCPP_INFO(get_logger(), "=== Demo complete ===");
  }

private:
  rclcpp::executors::SingleThreadedExecutor &            executor_;
  rclcpp_action::Client<MoveToPosition>::SharedPtr move_client_;
  rclcpp_action::Client<GripperControl>::SharedPtr gripper_client_;
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ArmClientExample>(executor);
  executor.add_node(node);

  node->run_demo();

  rclcpp::shutdown();
  return 0;
}
