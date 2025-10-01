#include "position_speed_sender/position_speed_sender.hpp"

namespace position_speed_sender
{

PositionSpeedSender::PositionSpeedSender()
: Node("position_speed_sender")
{
  // 声明并获取参数
  this->declare_parameter("amplitude", 1.0);
  this->declare_parameter("frequency", 1.0);
  this->declare_parameter("offset", 0.0);
  this->declare_parameter("phase", 0.0);
  this->declare_parameter("fixed_speed", 0.0);
  this->declare_parameter("publish_frequency", 100.0);
  this->declare_parameter("topic_name", "position_speed_command");

  amplitude_ = this->get_parameter("amplitude").as_double();
  frequency_ = this->get_parameter("frequency").as_double();
  offset_ = this->get_parameter("offset").as_double();
  phase_ = this->get_parameter("phase").as_double();
  fixed_speed_ = this->get_parameter("fixed_speed").as_double();
  publish_frequency_ = this->get_parameter("publish_frequency").as_double();
  topic_name_ = this->get_parameter("topic_name").as_string();

  // 创建发布器
  publisher_ = this->create_publisher<rm_controller_interface::msg::PositionSpeedCommand>(
    topic_name_, 10);

  // 记录启动时间
  start_time_ = std::chrono::steady_clock::now();

  // 创建定时器
  auto timer_period = std::chrono::microseconds(
    static_cast<int64_t>(1000000.0 / publish_frequency_));
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&PositionSpeedSender::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), 
    "PositionSpeedSender initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), 
    "  - Amplitude: %.3f", amplitude_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Frequency: %.3f Hz", frequency_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Offset: %.3f", offset_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Phase: %.3f rad", phase_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Fixed speed: %.3f", fixed_speed_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Publish frequency: %.1f Hz", publish_frequency_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Topic name: %s", topic_name_.c_str());
}

void PositionSpeedSender::timer_callback()
{
  // 计算当前时间
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(
    current_time - start_time_);
  double time_sec = elapsed_time.count() / 1000000.0;

  // 计算正弦波位置
  double position = calculate_sine_position(time_sec);
  
  // 使用固定速度值
  double speed = fixed_speed_;

  // 创建并发送消息
  auto message = rm_controller_interface::msg::PositionSpeedCommand();
  message.position = static_cast<float>(position);
  message.speed = static_cast<float>(speed);

  publisher_->publish(message);

  // 每秒打印一次状态信息
  static int counter = 0;
  counter++;
  if (counter >= static_cast<int>(publish_frequency_))
  {
    RCLCPP_INFO(this->get_logger(), 
      "Time: %.3fs, Position: %.3f, Speed: %.3f", 
      time_sec, position, speed);
    counter = 0;
  }
}

double PositionSpeedSender::calculate_sine_position(double time_sec)
{
  return amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_sec + phase_) + offset_;
}

} // namespace position_speed_sender