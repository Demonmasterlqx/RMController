#ifndef SPEED_EFFOR_CONTROLLER__SPEED_EFFOR_CONTROLLER_HPP_
#define SPEED_EFFOR_CONTROLLER__SPEED_EFFOR_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


#include "speed_effor_controller/speed_effor_controller_parameters.hpp"

#include "pid_controller/pid_controller.hpp"

// #define DEBUG

namespace RM_hardware_interface{

class SpeedEffortController : public controller_interface::ChainableControllerInterface{

public:
    SpeedEffortController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers() override;

    bool on_set_chained_mode(bool chained_mode) override;

private:

    const int EFFORT_COMMAND_INDEX=0;
    const int SPEED_STATE_INDEX=0;
    const int EFFORT_STATE_INDEX=1;

    // 参数获取
    std::shared_ptr<speed_effor_controller::ParamListener> param_listener_=nullptr;

    // 参数
    speed_effor_controller::Params params_;

    // 是否启用链式控制
    bool chainable_=false;

    // 回调速度命令的缓冲区
    realtime_tools::RealtimeBuffer<double> speed_command_buffer_;

    // chainable下的command接口的命令的缓冲区
    std::shared_ptr<double> speed_command_chain_=nullptr;

    // 看门狗计时器
    rclcpp::Time last_command_time_;

    // 看门狗时间
    rclcpp::Duration watchdog_timeout_=rclcpp::Duration::from_seconds(0);

    // 当前速度命令是否被使用
    std::atomic<bool> command_used_{false};

    // velocity state publisher
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> velocity_state_publisher_=nullptr;

    // effort state publisher
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> effort_state_publisher_=nullptr;

    // effort reference publisher
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> effort_reference_publisher_=nullptr;

    // velocity reference publisher
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> velocity_reference_publisher_=nullptr;

    // speed subscriber
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_command_subscriber_=nullptr;

    // front_feed
    std::atomic<double> front_feed_{0.0};

    // PID controller
    std::shared_ptr<PIDController> pid_controller_=nullptr;

    // PID config
    PID_Init_Config_s pid_config_;

    // 重置看门狗
    void reset_watchdog();
    // 看门狗是否触发
    bool is_watchdog_triggered();

    // 设置command_used_ 为true
    void set_command_used();

    void speed_command_callback(const std_msgs::msg::Float32::SharedPtr msg);

    #ifdef DEBUG

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr debug_time_interval_publisher_=nullptr;
    // 上一次接收到命令的时间，也就是调用 update_and_write_commands 的时间
    rclcpp::Time last_time_ = rclcpp::Time(0,0);
    
    #endif



}; // class SpeedEffortController



}// namespace RM_hardware_interface


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::SpeedEffortController, controller_interface::ChainableControllerInterface);

#endif 