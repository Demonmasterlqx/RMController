#ifndef DIFFERENTIAL_COMMAND_SENDER_HPP_
#define DIFFERENTIAL_COMMAND_SENDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rm_controller_interface/msg/end_differential_command.hpp>
#include <chrono>
#include <cmath>
#include <string>

namespace differential_command_sender
{

/**
 * @brief 工作模式枚举
 */
enum class WorkMode
{
  POSITION_SINE_VELOCITY_FIXED,    // 模式1: 位置正弦波，速度固定
  VELOCITY_SINE_POSITION_FIXED,    // 模式2: 速度正弦波，位置固定
  POSITION_SINE_VELOCITY_DERIVED,  // 模式3: 位置正弦波，速度求导
  ROLL_SINE_PITCH_FIXED,          // 模式4: roll正弦，pitch固定，速度固定
  ROLL_FIXED_PITCH_SINE,          // 模式5: roll固定，pitch正弦，速度固定
  ROLL_PITCH_BOTH_SINE            // 模式6: roll和pitch均为正弦，速度固定
};

class DifferentialCommandSender : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  DifferentialCommandSender();

  /**
   * @brief 析构函数
   */
  ~DifferentialCommandSender() = default;

private:
  /**
   * @brief 定时器回调函数，发送差分命令
   */
  void timer_callback();

  /**
   * @brief 计算正弦波位置
   * @param time_sec 当前时间（秒）
   * @return 正弦波位置值
   */
  double calculate_sine_position(double time_sec);

  /**
   * @brief 计算正弦波速度
   * @param time_sec 当前时间（秒）
   * @return 正弦波速度值
   */
  double calculate_sine_velocity(double time_sec);

  /**
   * @brief 计算位置的导数（速度）
   * @param time_sec 当前时间（秒）
   * @return 位置导数值
   */
  double calculate_position_derivative(double time_sec);

  /**
   * @brief 根据字符串解析工作模式
   * @param mode_str 模式字符串
   * @return 工作模式枚举
   */
  WorkMode parse_work_mode(const std::string& mode_str);

  // ROS2发布器
  rclcpp::Publisher<rm_controller_interface::msg::EndDifferentialCommand>::SharedPtr publisher_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 工作模式
  WorkMode work_mode_;
  
  // 正弦波参数
  double amplitude_;        // 振幅
  double frequency_;        // 频率 (Hz)
  double offset_;          // 偏移量
  double phase_;           // 相位
  
  // 固定值参数
  double fixed_velocity_;     // 固定速度值
  double fixed_position_;     // 固定位置值
  
  // 时间相关
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  
  // 发布频率
  double publish_frequency_;
  
  // 话题名称
  std::string topic_name_;
};

} // namespace differential_command_sender

#endif // DIFFERENTIAL_COMMAND_SENDER_HPP_