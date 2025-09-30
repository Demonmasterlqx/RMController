#include "speed_effor_controller/speed_effor_controller.hpp"

namespace RM_hardware_interface{

controller_interface::return_type SpeedEffortController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){

    #ifdef DEBUG

    try{
        auto msg = std_msgs::msg::Float32();
        msg.data = (time - last_time_).seconds();
        last_time_ = time;
        debug_time_interval_publisher_->publish(msg);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing debug time interval: %s",e.what());
    }

    #endif




}

} // namespace RM_hardware_interface