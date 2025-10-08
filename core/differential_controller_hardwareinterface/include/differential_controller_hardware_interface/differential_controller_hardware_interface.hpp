#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <atomic>
#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#ifndef DIFFERENTIAL_CONTROLLER_HARDWARE_INTERFACE_HPP_
#define DIFFERENTIAL_CONTROLLER_HARDWARE_INTERFACE_HPP_

namespace RM_hardware_interface{

using realtime_tools::RealtimePublisher;
using StateInterface = hardware_interface::StateInterface;
using HardwareInfo = hardware_interface::HardwareInfo;
using CallbackReturn = hardware_interface::CallbackReturn;
using CommandInterface = hardware_interface::CommandInterface;
using return_type = hardware_interface::return_type;

// 用于保存每个 state 接口的信息以及订阅
struct StateInfo{
    // 状态接口名称 全称
    std::string name;
    // 暴露给ros2 control的接口
    std::shared_ptr<double> data_ptr;
    // 用于和 sub callback 连接的 buffer
    realtime_tools::RealtimeBuffer<double> buffer;
    // 没有读到数据的次数
    int no_read_count=0;
    bool new_data_available=false;
    // sub
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub;

    StateInfo(std::string name_, rclcpp::Node::SharedPtr node_){
        name = name_;
        data_ptr = std::make_shared<double>(0.0);
        buffer.initRT(0.0);
        sub = node_->create_subscription<std_msgs::msg::Float32>(
            name, 1,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                buffer.writeFromNonRT(msg->data);
                no_read_count = 0;
                new_data_available = true;
            });
    }
};


class DifferentialControllerHardwareInterface : public hardware_interface::SystemInterface{
public:
    DifferentialControllerHardwareInterface();
    ~DifferentialControllerHardwareInterface() = default;

    CallbackReturn on_init(const HardwareInfo & hardware_info) override;

    std::vector<StateInterface> export_state_interfaces() override;

    std::vector<CommandInterface> export_command_interfaces() override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);
private:

    // ros2 node
    rclcpp::Node::SharedPtr node_;

    // 构建好的 state 接口
    std::vector<StateInterface> state_interfaces_;

    // 保存每个 state 接口的 结构体
    std::vector<std::shared_ptr<StateInfo>> state_ptrs_;

    // ros2 node spin thread
    std::shared_ptr<std::thread> node_thread_;

};

} // namespace RM_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::DifferentialControllerHardwareInterface, hardware_interface::SystemInterface);

#endif  // DIFFERENTIAL_CONTROLLER_HARDWARE_INTERFACE_HPP_