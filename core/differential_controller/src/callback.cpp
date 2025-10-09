#include "differential_controller/differential_controller.hpp"

namespace RM_hardware_interface{

void DifferentialController::zero_sub_callback_(const std_msgs::msg::Bool::SharedPtr msg){

    if(!msg->data){
        return;
    }
    std::lock_guard<std::mutex> lock(zero_start_time_mutex_);
    zero_start_time_ = this->get_node()->get_clock()->now();
    zero_reach_limit_attempts_ = 0;
    zero_state_.store(ZERO_STATE::ZERO_INPROGRESS);

}

void DifferentialController::force_zero_sub_callback_(const std_msgs::msg::Bool::SharedPtr msg){
    if(!msg->data){
        return;
    }

    zero_state_.store(ZERO_STATE::ZERO_FORCE);

}

void DifferentialController::command_sub_callback_(const rm_controller_interface::msg::EndDifferentialCommand::SharedPtr msg){
    last_command_msg_.writeFromNonRT(*msg);
    watchdog_->reset();
}

} // namespace RM_hardware_interface