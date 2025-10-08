#include "differential_controller/differential_controller.hpp"

namespace RM_hardware_interface{

void DifferentialController::zero_sub_callback_(const std_msgs::msg::Bool::SharedPtr msg){

    if(!msg->data){
        return;
    }
    zero_state_.store(ZERO_STATE::ZERO_INPROGRESS);

}

void DifferentialController::force_zero_sub_callback_(const std_msgs::msg::Bool::SharedPtr msg){
    if(!msg->data){
        return;
    }

    try{

        left_motor_position_counter_->reset(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value());
        right_motor_position_counter_->reset(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value());
        zero_state_.store(ZERO_STATE::ZERO_OK);

    }
    catch(std::exception &e){
        RCLCPP_ERROR(this->get_node()->get_logger(), "Error occurred while forcing zero: %s", e.what());
        zero_state_.store(ZERO_STATE::ZERO_FAIL);
    }

}

} // namespace RM_hardware_interface