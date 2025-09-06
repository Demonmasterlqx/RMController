#include "speed_effor_controller/speed_effor_controller.hpp"

namespace RM_hardware_interface{

SpeedEffortController::SpeedEffortController():controller_interface::ChainableControllerInterface(){}

controller_interface::CallbackReturn SpeedEffortController::on_init(){
    try{
        param_listener_=std::make_shared<speed_effor_controller::ParamListener>(get_node());
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not initialize parameters: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    speed_command_buffer_.writeFromNonRT(0.0);

    reset_watchdog();

    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> SpeedEffortController::on_export_reference_interfaces(){

    reference_interfaces_.resize(1, 0.0);

    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name() , params_.joint + "/" + "velocity" , &reference_interfaces_[0]));
    RCLCPP_INFO(get_node()->get_logger(),"Exporting reference interface: %s", interfaces.back().get_name().c_str());
    return interfaces;
}

bool SpeedEffortController::on_set_chained_mode(bool){
    return true;
}

controller_interface::CallbackReturn SpeedEffortController::on_configure(const rclcpp_lifecycle::State &){
    params_ = param_listener_->get_params();

    // 设置看门狗
    watchdog_timeout_=rclcpp::Duration::from_seconds(params_.reference_timeout);

    // 设置参数
    K_P_.store(params_.K_P);
    K_I_.store(params_.K_I);
    K_D_.store(params_.K_D);
    front_feed_.store(params_.front_feed);
    forgetting_factor_.store(params_.forgetting_factor);


    // 是否启用链式控制
    chainable_ = params_.chainable;
    if(!chainable_){
        RCLCPP_INFO_STREAM(get_node()->get_logger(),"not in chained mode, subscribing to topic " << params_.command_topic);
        speed_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32>(
            params_.command_topic,1,
            std::bind(&SpeedEffortController::speed_command_callback,this,std::placeholders::_1)
        );
        set_chained_mode(false);
    }
    else{
        set_chained_mode(true);
        RCLCPP_INFO(get_node()->get_logger(),"in chained mode, not subscribing to any topic");
    }

    try{
        // 设置publisher

        velocity_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(params_.joint + "/state/" + params_.speed_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (params_.joint + "/state/" + params_.speed_state_name).c_str());
        effort_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(params_.joint + "/state/" + params_.effort_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (params_.joint + "/state/" + params_.effort_state_name).c_str());
        effort_reference_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(params_.joint + "/reference/" + params_.effort_command_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (params_.joint + "/reference/" + params_.effort_command_name).c_str());
        velocity_reference_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(params_.joint + "/reference/" + params_.speed_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (params_.joint + "/reference/" + params_.speed_state_name).c_str());

    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not create publishers: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedEffortController::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"cleaning up from state %s",previous_state.label().c_str());
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
    RCLCPP_INFO(get_node()->get_logger(),"activating from state %s",previous_state.label().c_str());
    speed_command_buffer_.writeFromNonRT(0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpeedEffortController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"deactivating from state %s",previous_state.label().c_str());
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

void SpeedEffortController::set_command_used(){
    command_used_.store(true);
}

controller_interface::return_type SpeedEffortController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){
    (void)time;
    (void)period;

    static double integral_error=0.0;
    static double last_error=0.0;

    double reference_speed=0.0;
    double state_speed = 0.0;
    double state_effort = 0.0;
    try{
        if(!chainable_){
            reference_speed=*speed_command_buffer_.readFromRT();
        }
        else{
            reference_speed=reference_interfaces_[0];
        }
        if(is_watchdog_triggered()){
            reference_speed=0.0;
        }
        set_command_used();

        state_speed = state_interfaces_[SPEED_STATE_INDEX].get_value();
        state_effort = state_interfaces_[EFFORT_STATE_INDEX].get_value();
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during reading interfaces: %s",e.what());
        return controller_interface::return_type::ERROR;
    }

    // 更新 error 统计
    double error = reference_speed - state_speed;
    double differential_error = (error - last_error);
    integral_error += error * forgetting_factor_.load();
    last_error = error;

    // 输出当前的状态
    try{
        {
            auto msg = std_msgs::msg::Float32();
            msg.data = state_speed;
            velocity_state_publisher_->publish(msg);
        }
        {
            auto msg = std_msgs::msg::Float32();
            msg.data = state_effort;
            effort_state_publisher_->publish(msg);
        }
        {
            auto msg = std_msgs::msg::Float32();
            msg.data = reference_speed;
            velocity_reference_publisher_->publish(msg);
        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing state: %s",e.what());
    }

    try{
        // 计算输出
        double effort_command = K_P_ * error + K_I_ * integral_error + K_D_ * differential_error + front_feed_.load() * reference_speed;

        // publish reference
        {
            auto msg = std_msgs::msg::Float32();
            msg.data = effort_command;
            effort_reference_publisher_->publish(msg);
        }

        if(!rclcpp::ok()){
            effort_command = 0.0;
        }

        command_interfaces_[EFFORT_COMMAND_INDEX].set_value(effort_command);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during calculate and writing command: %s",e.what());
        return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
}

void SpeedEffortController::reset_watchdog(){
    last_command_time_=get_node()->now();
    command_used_.store(false);
}

bool SpeedEffortController::is_watchdog_triggered(){
    return ((get_node()->now() - last_command_time_) > watchdog_timeout_)&&(command_used_.load());
}


}// namespace RM_hardware_interface

