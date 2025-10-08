#include "differential_controller/differential_controller.hpp"

namespace RM_hardware_interface{

controller_interface::return_type DifferentialController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){

    #ifdef DEBUG
    // 计算调用周期
    auto now = this->get_node()->now();
    auto call_period = (now - last_time_).seconds();
    last_time_ = now;
    // 发布调用周期
    if (call_period_publisher_ && call_period > 0.0) {
        auto msg = std_msgs::msg::Float32();
        msg.data = static_cast<float>(call_period);
        call_period_publisher_->publish(msg);
    }
    #endif

    return controller_interface::return_type::OK;
}


} // namespace RM_hardware_interface