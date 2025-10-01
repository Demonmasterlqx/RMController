#ifndef POSITION_SPEED_SENDER_HPP_
#define POSITION_SPEED_SENDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rm_controller_interface/msg/position_speed_command.hpp>
#include <chrono>
#include <cmath>

namespace position_speed_sender
{

class PositionSpeedSender : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  PositionSpeedSender();

  /**
   * @brief 析构函数
   */
  ~PositionSpeedSender() = default;

private:
  /**
   * @brief 定时器回调函数，发送正弦波位置命令
   */
  void timer_callback();

  /**
   * @brief 计算正弦波位置
   * @param time_sec 当前时间（秒）
   * @return 正弦波位置值
   */
  double calculate_sine_position(double time_sec);

  // ROS2发布器
  rclcpp::Publisher<rm_controller_interface::msg::PositionSpeedCommand>::SharedPtr publisher_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 正弦波参数
  double amplitude_;        // 振幅
  double frequency_;        // 频率 (Hz)
  double offset_;          // 偏移量
  double phase_;           // 相位
  double fixed_speed_;     // 固定速度值
  
  // 时间相关
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  
  // 发布频率
  double publish_frequency_;
  
  // 话题名称
  std::string topic_name_;
};

} // namespace position_speed_sender

#endif // POSITION_SPEED_SENDER_HPP_
