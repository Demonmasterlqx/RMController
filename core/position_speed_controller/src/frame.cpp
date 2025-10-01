#include "position_speed_controller/position_speed_controller.hpp"

namespace RM_hardware_interface{

PositionSpeedController::PositionSpeedController():controller_interface::ChainableControllerInterface(){}

controller_interface::CallbackReturn PositionSpeedController::on_init(){

    #ifdef DEBUG
    call_period_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/time_interval", 1);
    RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/time_interval").c_str());
    last_time_ = get_node()->now();
    #endif

    try{
        param_listener_=std::make_shared<speed_effor_controller::ParamListener>(get_node());
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not initialize parameters: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    command_buffer_.writeFromNonRT(rm_controller_interface::msg::PositionSpeedCommand());

    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> PositionSpeedController::on_export_reference_interfaces(){

    reference_interfaces_.resize(2, 0.0);

    std::vector<hardware_interface::CommandInterface> interfaces;
    interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name() , params_.joint + "/" + "velocity" , &reference_interfaces_[CHAINED_SPEED_COMMAND_INDEX]));
    interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name() , params_.joint + "/" + "position" , &reference_interfaces_[CHAINED_POSITION_COMMAND_INDEX]));
    RCLCPP_INFO(get_node()->get_logger(),"Exporting reference interface: %s", interfaces.back().get_name().c_str());
    return interfaces;
}

bool PositionSpeedController::on_set_chained_mode(bool){
    return true;
}

controller_interface::CallbackReturn PositionSpeedController::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"cleaning up from state %s",previous_state.label().c_str());
    command_buffer_.writeFromNonRT(rm_controller_interface::msg::PositionSpeedCommand());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PositionSpeedController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(params_.joint + "/" + params_.effort_command_name);

    return config;
}


controller_interface::InterfaceConfiguration PositionSpeedController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(params_.joint + "/" + params_.speed_state_name);
    config.names.push_back(params_.joint + "/" + params_.position_state_name);
    config.names.push_back(params_.joint + "/" + params_.effort_state_name);

    return config;
}

controller_interface::CallbackReturn PositionSpeedController::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"activating from state %s",previous_state.label().c_str());
    command_buffer_.writeFromNonRT(rm_controller_interface::msg::PositionSpeedCommand());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PositionSpeedController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(),"deactivating from state %s",previous_state.label().c_str());
    command_buffer_.writeFromNonRT(rm_controller_interface::msg::PositionSpeedCommand());
    return controller_interface::CallbackReturn::SUCCESS;
}

void PositionSpeedController::position_speed_command_callback(const rm_controller_interface::msg::PositionSpeedCommand::SharedPtr msg){
    watchdog_->reset();
    RCLCPP_INFO(get_node()->get_logger(),"Received command: position: %f, speed: %f",msg->position,msg->speed);
    command_buffer_.writeFromNonRT(*msg);
}

controller_interface::return_type PositionSpeedController::update_reference_from_subscribers(){
    // nothing to do, as the callback already updated the buffer
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn PositionSpeedController::on_configure(const rclcpp_lifecycle::State &){

    try{
        params_ = param_listener_->get_params();
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not get parameters: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 设置看门狗
    try{
        watchdog_ = std::make_shared<Watchdog>(params_.reference_timeout);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not create watchdog: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 设置位置PID
    try{

        auto pid_config = PID_Init_Config_s{
            .Kp = static_cast<float>(params_.pos_Kp),
            .Ki = static_cast<float>(params_.pos_Ki),
            .Kd = 0,
            .MaxOut = static_cast<float>(params_.pos_MaxOut),
            .DeadBand = 0.001,
            .Improve = static_cast<PID_Improvement_e> (
                PID_Integral_Limit | 
                PID_Trapezoid_Intergral | 
                PID_Derivative_On_Measurement | 
                PID_ErrorHandle
            ),
            .IntegralLimit = static_cast<float>(params_.pos_Intergral_Limit),
            .CoefA = 0,
            .CoefB = 0,
            .Output_LPF_RC = 0,
            .Derivative_LPF_RC = 0,
        };

        position_pid_ = std::make_shared<PIDController>(pid_config);

    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not create position PID controllers: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 设置速度PID
    try{
        auto pid_config = PID_Init_Config_s{
            .Kp = static_cast<float>(params_.vel_Kp),
            .Ki = static_cast<float>(params_.vel_Ki),
            .Kd = 0,
            .MaxOut = static_cast<float>(params_.vel_MaxOut),
            .DeadBand = 0.001,
            .Improve = static_cast<PID_Improvement_e> (
                PID_Integral_Limit | 
                PID_Trapezoid_Intergral | 
                PID_Derivative_On_Measurement | 
                PID_ErrorHandle
            ),
            .IntegralLimit = static_cast<float>(params_.vel_Intergral_Limit),
            .CoefA = 0,
            .CoefB = 0,
            .Output_LPF_RC = 0,
            .Derivative_LPF_RC = 0,
        };

        speed_pid_ = std::make_shared<PIDController>(pid_config);

    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not create speed PID controllers: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    chainable_ = params_.chainable;

    if(!chainable_){
        RCLCPP_INFO_STREAM(get_node()->get_logger(),"not in chained mode, subscribing to topic " << params_.command_topic);
        command_subscriber_ = get_node()->create_subscription<rm_controller_interface::msg::PositionSpeedCommand>(
            std::string(get_node()->get_name()) + "/" + params_.command_topic,1,
            std::bind(&PositionSpeedController::position_speed_command_callback,this,std::placeholders::_1)
        );
        set_chained_mode(false);
    }
    else{
        set_chained_mode(true);
        RCLCPP_INFO(get_node()->get_logger(),"in chained mode, not subscribing to any topic");
    }

    // 设置publisher
    try{
        position_reference_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + "position", 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + "position").c_str());
        speed_reference_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + "velocity", 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + "velocity").c_str());
        position_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.position_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.position_state_name).c_str());
        velocity_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.speed_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.speed_state_name).c_str());
        effort_state_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.effort_state_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/state/" + params_.effort_state_name).c_str());
        effort_reference_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + params_.effort_command_name, 1);
        RCLCPP_INFO(get_node()->get_logger(),"Created publisher for topic: %s", (std::string(get_node()->get_name()) +"/"+ params_.joint + "/reference/" + params_.effort_command_name).c_str());
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Could not create publishers: %s",e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}



} // namespace RM_hardware_interface