#include "differential_command_sender/differential_command_sender.hpp"

int main(int argc, char * argv[])
{
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<differential_command_sender::DifferentialCommandSender>();

  // 运行节点
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
  }

  // 清理
  rclcpp::shutdown();
  return 0;
}