#include "open_manipulator_controller/arm_controller.hpp"
#include "rclcpp/rclcpp.hpp"

#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<open_manipulator_controller::ArmController>();

  // MoveGroupInterface requires the node to be actively receiving callbacks
  // before it is constructed, because it calls /move_group services during
  // initialisation. We therefore spin in a background thread, call post_init()
  // to build the MoveGroupInterfaces, then join and hand back to the executor.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Start spinning in the background
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Now safe to construct MoveGroupInterface
  node->post_init();

  // Wait for shutdown
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}
