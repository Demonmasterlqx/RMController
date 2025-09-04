#include "speed_effor_controller/speed_effor_controller.hpp"

namespace RM_hardware_interface{

SpeedEffortController::SpeedEffortController():controller_interface::ChainableControllerInterface(){}

controller_interface::CallbackReturn SpeedEffortController::on_init(){
    try{
        param_listener_=std::make_shared<speed_effor_controller::ParamListener>(get_node());
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"SpeedEffortController: Could not initialize parameters: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    speed_command_buffer_.writeFromNonRT(0.0);

    last_command_time_=get_node()->now();

    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> SpeedEffortController::on_export_reference_interfaces(){
    std::vector<hardware_interface::CommandInterface> interfaces;
    hardware_interface::CommandInterface interface(params_.joint ,"velocity" , speed_command_chain_.get());
    return interfaces;
}

controller_interface::CallbackReturn SpeedEffortController::on_configure(const rclcpp_lifecycle::State &){
    params_ = param_listener_->get_params();

    // 设置看门狗
    watchdog_timeout_=rclcpp::Duration::from_seconds(params_.reference_timeout);

    chainable_ = params_.chainable;
    if(!chainable_){
        RCLCPP_INFO_STREAM(get_node()->get_logger(),"SpeedEffortController: not in chained mode, subscribing to topic " << params_.command_topic);
        get_node()->create_subscription<std_msgs::msg::Float32>(
            params_.command_topic,1,
            std::bind(&SpeedEffortController::speed_command_callback,this,std::placeholders::_1)
        );
    }
    else{
        set_chained_mode(true);
        speed_command_chain_=std::make_shared<double>(0.0);
        RCLCPP_INFO(get_node()->get_logger(),"SpeedEffortController: in chained mode, not subscribing to any topic");
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedEffortController::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"SpeedEffortController: cleaning up from state %s",previous_state.label().c_str());
    speed_command_buffer_.writeFromNonRT(0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SpeedEffortController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(params_.joint + "/" + params_.effort_command_name);

    return config;
}

controller_interface::InterfaceConfiguration SpeedEffortController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(params_.joint + "/" + params_.speed_state_name);
    config.names.push_back(params_.joint + "/" + params_.effort_state_name);

    return config;
}


controller_interface::CallbackReturn SpeedEffortController::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"SpeedEffortController: activating from state %s",previous_state.label().c_str());
    speed_command_buffer_.writeFromNonRT(0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedEffortController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"SpeedEffortController: deactivating from state %s",previous_state.label().c_str());
    speed_command_buffer_.writeFromNonRT(0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

void SpeedEffortController::speed_command_callback(const std_msgs::msg::Float32::SharedPtr msg){
    reset_watchdog();
    speed_command_buffer_.writeFromNonRT(msg->data);
}

controller_interface::return_type SpeedEffortController::update_reference_from_subscribers(){
    // nothing to do, as the callback already updated the buffer
    return controller_interface::return_type::OK;
}


controller_interface::return_type SpeedEffortController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){
    (void)time;
    (void)period;

    double command=0.0;
    if(!chainable_){
        command=*speed_command_buffer_.readFromRT();
    }
    else{
        command=*speed_command_chain_;
    }
    if(is_watchdog_triggered()){
        command=0.0;
    }
    set_command_used();

    double state_speed = state_interfaces_[SPEED_STATE_INDEX].get_value();
    double state_effort = state_interfaces_[EFFORT_STATE_INDEX].get_value();
    /**
     * @todo PID控制器
     * 
     */

}

void SpeedEffortController::reset_watchdog(){
    last_command_time_=get_node()->now();
    command_used_.store(false);
}

bool SpeedEffortController::is_watchdog_triggered(){
    return ((get_node()->now() - last_command_time_) > watchdog_timeout_)&&(!command_used_.load());
}


}// namespace RM_hardware_interface

