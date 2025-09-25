#include "speed_effor_controller/speed_effor_controller.hpp"

namespace RM_hardware_interface{

SpeedEffortController::SpeedEffortController():controller_interface::ChainableControllerInterface(){}

controller_interface::CallbackReturn SpeedEffortController::on_init(){

    #ifdef DEBUG
    debug_time_interval_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>( std::string(get_node()->get_name()) + "/time_interval", 1);
    RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) + "/time_interval").c_str());
    last_time_ = get_node()->now();
    #endif

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
    front_feed_.store(params_.front_feed);
    friction_compensation_ = params_.friction_compensation;

    pid_config_ = PID_Init_Config_s{
        .Kp = static_cast<float>(params_.Kp),
        .Ki = static_cast<float>(params_.Ki),
        .Kd = static_cast<float>(params_.Kd),
        .MaxOut = static_cast<float>(params_.MaxOut),
        .DeadBand = static_cast<float>(params_.DeadBand),

        .Improve = static_cast<PID_Improvement_e>(
            PID_Integral_Limit |
            PID_Derivative_On_Measurement |
            PID_Trapezoid_Intergral |
            // PID_OutputFilter |
            PID_ChangingIntegrationRate |
            // PID_DerivativeFilter |
            PID_ErrorHandle
        ),
        .IntegralLimit = static_cast<float>(params_.IntegralLimit),
        .CoefA = static_cast<float>(params_.CoefA),
        .CoefB = static_cast<float>(params_.CoefB),
        .Output_LPF_RC = static_cast<float>(params_.Output_LPF_RC),
        .Derivative_LPF_RC = static_cast<float>(params_.Derivative_LPF_RC),
    };

    pid_controller_ = std::make_shared<PIDController>(pid_config_);


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

        velocity_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.speed_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.speed_state_name).c_str());
        effort_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.effort_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.effort_state_name).c_str());
        effort_reference_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + params_.effort_command_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + params_.effort_command_name).c_str());
        velocity_reference_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + params_.speed_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + params_.speed_state_name).c_str());

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

    // 输出和上一次的时间间隔
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
    

    (void)time;
    (void)period;

    double reference_speed=0.0;
    double state_speed = 0.0;
    double state_effort = 0.0;
    try{
        if(!chainable_){
            reference_speed=*speed_command_buffer_.readFromRT();
        }
        else{
            reset_watchdog();
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

    (void)state_effort;

    // 计算输出
    double effort_command = pid_controller_->calculate(state_speed, reference_speed);
    effort_command += front_feed_.load() * reference_speed;

    if(std::abs(reference_speed) <= 1e-6){
        effort_command = effort_command;
    }
    else if(reference_speed < 0){
        effort_command -= friction_compensation_;
    }
    else if(reference_speed > 0){
        effort_command += friction_compensation_;
    }

    // 限制输出
    if(effort_command > params_.MaxOut){
        effort_command = params_.MaxOut;
    }
    else if(effort_command < -params_.MaxOut){
        effort_command = -params_.MaxOut;
    }

    try{
        command_interfaces_[EFFORT_COMMAND_INDEX].set_value(effort_command);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during writing command interface: %s",e.what());
        return controller_interface::return_type::ERROR;
    }

    // pub 消息

    try{
        auto vel_state = std_msgs::msg::Float32();
        auto eff_state = std_msgs::msg::Float32();
        auto vel_ref = std_msgs::msg::Float32();
        auto eff_ref = std_msgs::msg::Float32();
        vel_state.data = state_speed;
        eff_state.data = state_effort;
        vel_ref.data = reference_speed;
        eff_ref.data = effort_command;
        velocity_state_publisher_->publish(vel_state);
        effort_state_publisher_->publish(eff_state);
        velocity_reference_publisher_->publish(vel_ref);
        effort_reference_publisher_->publish(eff_ref);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing velocity state: %s",e.what());
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

