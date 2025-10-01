#ifndef POSITION_SPEED_CONTROLLER__POSITION_SPEED_CONTROLLER_HPP_
#define POSITION_SPEED_CONTROLLER__POSITION_SPEED_CONTROLLER_HPP_

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
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rm_controller_interface/msg/position_speed_command.hpp"

#include "position_speed_controller/position_speed_controller_parameters.hpp"

#include "pid_controller/pid_controller.hpp"

#include "watchdog/watchdog.hpp"

#include "position_speed_controller/trajectory_generator.hpp"

// #define DEBUG

namespace RM_hardware_interface{

class PositionSpeedController : public controller_interface::ChainableControllerInterface{
public:

    PositionSpeedController();

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
    const int POSITION_STATE_INDEX=1;
    const int EFFORT_STATE_INDEX=2;

    // 链式控制下的命令下标

    const int CHAINED_SPEED_COMMAND_INDEX=0;
    const int CHAINED_POSITION_COMMAND_INDEX=1;

    // 参数获取
    std::shared_ptr<speed_effor_controller::ParamListener> param_listener_=nullptr;

    // 参数
    speed_effor_controller::Params params_;

    // 是否启动链式控制
    bool chainable_=false;

    // 看门狗
    std::shared_ptr<Watchdog> watchdog_=nullptr;

    // sub/pub

    // 位置命令发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> position_reference_publisher_=nullptr;
    // 速度命令发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> speed_reference_publisher_=nullptr;
    // 位置状态发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> position_state_publisher_=nullptr;
    // 速度状态发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> velocity_state_publisher_=nullptr;
    // 力矩状态发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> effort_state_publisher_=nullptr;
    // 力矩参考发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> effort_reference_publisher_=nullptr;
    // topic命令sub
    rclcpp::Subscription<rm_controller_interface::msg::PositionSpeedCommand>::SharedPtr command_subscriber_=nullptr;

    // 回调速度命令的缓冲区
    realtime_tools::RealtimeBuffer<rm_controller_interface::msg::PositionSpeedCommand> command_buffer_;


    #ifdef DEBUG
    // 调试信息发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> call_period_publisher_=nullptr;
    rclcpp::Time last_time_;
    #endif

    // 位置环PID
    std::shared_ptr<PIDController> position_pid_=nullptr;
    // 速度环PID
    std::shared_ptr<PIDController> speed_pid_=nullptr;

    // callback
    void position_speed_command_callback(const rm_controller_interface::msg::PositionSpeedCommand::SharedPtr msg);

}; // class PositionSpeedController

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::PositionSpeedController, controller_interface::ChainableControllerInterface);


#endif  // POSITION_SPEED_CONTROLLER__POSITION_SPEED_CONTROLLER_HPP_