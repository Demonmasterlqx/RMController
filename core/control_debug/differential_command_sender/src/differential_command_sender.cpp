#include "differential_command_sender/differential_command_sender.hpp"

namespace differential_command_sender
{

DifferentialCommandSender::DifferentialCommandSender()
: Node("differential_command_sender")
{
  // 声明并获取参数
  this->declare_parameter("work_mode", "position_sine_velocity_fixed");
  this->declare_parameter("amplitude", 1.0);
  this->declare_parameter("frequency", 1.0);
  this->declare_parameter("offset", 0.0);
  this->declare_parameter("phase", 0.0);
  this->declare_parameter("fixed_velocity", 0.0);
  this->declare_parameter("fixed_position", 0.0);
  this->declare_parameter("publish_frequency", 100.0);
  this->declare_parameter("topic_name", "end_differential_command");

  std::string mode_str = this->get_parameter("work_mode").as_string();
  work_mode_ = parse_work_mode(mode_str);
  amplitude_ = this->get_parameter("amplitude").as_double();
  frequency_ = this->get_parameter("frequency").as_double();
  offset_ = this->get_parameter("offset").as_double();
  phase_ = this->get_parameter("phase").as_double();
  fixed_velocity_ = this->get_parameter("fixed_velocity").as_double();
  fixed_position_ = this->get_parameter("fixed_position").as_double();
  publish_frequency_ = this->get_parameter("publish_frequency").as_double();
  topic_name_ = this->get_parameter("topic_name").as_string();

  // 创建发布器
  publisher_ = this->create_publisher<rm_controller_interface::msg::EndDifferentialCommand>(
    topic_name_, 10);

  // 记录启动时间
  start_time_ = std::chrono::steady_clock::now();

  // 创建定时器
  auto timer_period = std::chrono::microseconds(
    static_cast<int64_t>(1000000.0 / publish_frequency_));
  timer_ = this->create_wall_timer(
    timer_period,
    std::bind(&DifferentialCommandSender::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), 
    "DifferentialCommandSender initialized with parameters:");
  
  std::string mode_name;
  switch(work_mode_) {
    case WorkMode::POSITION_SINE_VELOCITY_FIXED:
      mode_name = "Position Sine + Velocity Fixed";
      break;
    case WorkMode::VELOCITY_SINE_POSITION_FIXED:
      mode_name = "Velocity Sine + Position Fixed";
      break;
    case WorkMode::POSITION_SINE_VELOCITY_DERIVED:
      mode_name = "Position Sine + Velocity Derived";
      break;
    case WorkMode::ROLL_SINE_PITCH_FIXED:
      mode_name = "Roll Sine + Pitch Fixed + Velocity Fixed";
      break;
    case WorkMode::ROLL_FIXED_PITCH_SINE:
      mode_name = "Roll Fixed + Pitch Sine + Velocity Fixed";
      break;
    case WorkMode::ROLL_PITCH_BOTH_SINE:
      mode_name = "Roll & Pitch Both Sine + Velocity Fixed";
      break;
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "  - Work mode: %s", mode_name.c_str());
  RCLCPP_INFO(this->get_logger(), 
    "  - Amplitude: %.3f", amplitude_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Frequency: %.3f Hz", frequency_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Offset: %.3f", offset_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Phase: %.3f rad", phase_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Fixed velocity: %.3f", fixed_velocity_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Fixed position: %.3f", fixed_position_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Publish frequency: %.1f Hz", publish_frequency_);
  RCLCPP_INFO(this->get_logger(), 
    "  - Topic name: %s", topic_name_.c_str());
}

void DifferentialCommandSender::timer_callback()
{
  // 计算当前时间
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(
    current_time - start_time_);
  double time_sec = elapsed_time.count() / 1000000.0;

  double roll_position, roll_velocity, pitch_position, pitch_velocity;

  // 根据工作模式计算位置和速度
  switch(work_mode_) {
    case WorkMode::POSITION_SINE_VELOCITY_FIXED:
      // 模式1: 位置正弦波，速度固定
      roll_position = calculate_sine_position(time_sec);
      roll_velocity = fixed_velocity_;
      pitch_position = calculate_sine_position(time_sec + M_PI/4); // 相位差π/4，使pitch和roll有不同的运动
      pitch_velocity = fixed_velocity_;
      break;
      
    case WorkMode::VELOCITY_SINE_POSITION_FIXED:
      // 模式2: 速度正弦波，位置固定
      roll_position = fixed_position_;
      roll_velocity = calculate_sine_velocity(time_sec);
      pitch_position = fixed_position_;
      pitch_velocity = calculate_sine_velocity(time_sec + M_PI/4);
      break;
      
    case WorkMode::POSITION_SINE_VELOCITY_DERIVED:
      // 模式3: 位置正弦波，速度求导
      roll_position = calculate_sine_position(time_sec);
      roll_velocity = calculate_position_derivative(time_sec);
      pitch_position = calculate_sine_position(time_sec + M_PI/4);
      pitch_velocity = calculate_position_derivative(time_sec + M_PI/4);
      break;
      
    case WorkMode::ROLL_SINE_PITCH_FIXED:
      // 模式4: roll正弦，pitch固定，速度固定
      roll_position = calculate_sine_position(time_sec);
      roll_velocity = fixed_velocity_;
      pitch_position = fixed_position_;
      pitch_velocity = fixed_velocity_;
      break;
      
    case WorkMode::ROLL_FIXED_PITCH_SINE:
      // 模式5: roll固定，pitch正弦，速度固定
      roll_position = fixed_position_;
      roll_velocity = fixed_velocity_;
      pitch_position = calculate_sine_position(time_sec);
      pitch_velocity = fixed_velocity_;
      break;
      
    case WorkMode::ROLL_PITCH_BOTH_SINE:
      // 模式6: roll和pitch均为正弦，速度固定
      roll_position = calculate_sine_position(time_sec);
      roll_velocity = fixed_velocity_;
      pitch_position = calculate_sine_position(time_sec + M_PI/4); // 相位差π/4
      pitch_velocity = fixed_velocity_;
      break;
      
    default:
      roll_position = pitch_position = 0.0;
      roll_velocity = pitch_velocity = 0.0;
      break;
  }

  // 创建并发送消息
  auto message = rm_controller_interface::msg::EndDifferentialCommand();
  message.roll_position = static_cast<float>(roll_position);
  message.roll_velocity = static_cast<float>(roll_velocity);
  message.pitch_position = static_cast<float>(pitch_position);
  message.pitch_velocity = static_cast<float>(pitch_velocity);

  publisher_->publish(message);

  // 每秒打印一次状态信息
  static int counter = 0;
  counter++;
  if (counter >= static_cast<int>(publish_frequency_))
  {
    RCLCPP_INFO(this->get_logger(), 
      "Time: %.3fs, Roll(pos: %.3f, vel: %.3f), Pitch(pos: %.3f, vel: %.3f)", 
      time_sec, roll_position, roll_velocity, pitch_position, pitch_velocity);
    counter = 0;
  }
}

double DifferentialCommandSender::calculate_sine_position(double time_sec)
{
  return amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_sec + phase_) + offset_;
}

double DifferentialCommandSender::calculate_sine_velocity(double time_sec)
{
  return amplitude_ * std::sin(2.0 * M_PI * frequency_ * time_sec + phase_) + offset_;
}

double DifferentialCommandSender::calculate_position_derivative(double time_sec)
{
  return amplitude_ * frequency_ * 2.0 * M_PI * 
         std::cos(2.0 * M_PI * frequency_ * time_sec + phase_);
}

WorkMode DifferentialCommandSender::parse_work_mode(const std::string& mode_str)
{
  if (mode_str == "position_sine_velocity_fixed" || mode_str == "1") {
    return WorkMode::POSITION_SINE_VELOCITY_FIXED;
  } else if (mode_str == "velocity_sine_position_fixed" || mode_str == "2") {
    return WorkMode::VELOCITY_SINE_POSITION_FIXED;
  } else if (mode_str == "position_sine_velocity_derived" || mode_str == "3") {
    return WorkMode::POSITION_SINE_VELOCITY_DERIVED;
  } else if (mode_str == "roll_sine_pitch_fixed" || mode_str == "4") {
    return WorkMode::ROLL_SINE_PITCH_FIXED;
  } else if (mode_str == "roll_fixed_pitch_sine" || mode_str == "5") {
    return WorkMode::ROLL_FIXED_PITCH_SINE;
  } else if (mode_str == "roll_pitch_both_sine" || mode_str == "6") {
    return WorkMode::ROLL_PITCH_BOTH_SINE;
  } else {
    RCLCPP_WARN(this->get_logger(), 
      "Unknown work mode '%s', using default mode 1", mode_str.c_str());
    return WorkMode::POSITION_SINE_VELOCITY_FIXED;
  }
}

} // namespace differential_command_sender