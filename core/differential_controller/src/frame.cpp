#include "differential_controller/differential_controller.hpp"
namespace RM_hardware_interface{

DifferentialController::DifferentialController():controller_interface::ChainableControllerInterface(){}

controller_interface::CallbackReturn DifferentialController::on_init(){

    // 创建参数监听器
    try{
        param_listener_ = std::make_shared<differential_controller::ParamListener>(get_node());
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create ParamListener: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 初始化 last_command_msg_
    last_command_msg_.initRT(rm_controller_interface::msg::EndDifferentialCommand());

    RCLCPP_INFO(get_node()->get_logger(), "DifferentialController initialized");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DifferentialController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // 电机的 command
    config.names.push_back(motor_left_command_position_name_);
    config.names.push_back(motor_left_command_velocity_name_);
    config.names.push_back(motor_right_command_position_name_);
    config.names.push_back(motor_right_command_velocity_name_);
    return config;
}

controller_interface::InterfaceConfiguration DifferentialController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // motor state: effort, position, velocity
    config.names.push_back(motor_left_state_effort_name_);
    config.names.push_back(motor_right_state_effort_name_);
    config.names.push_back(motor_left_state_position_name_);
    config.names.push_back(motor_right_state_position_name_);
    config.names.push_back(motor_left_state_velocity_name_);
    config.names.push_back(motor_right_state_velocity_name_);
    return config;
}

controller_interface::CallbackReturn DifferentialController::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(), "Cleaning up from %s", previous_state.label().c_str());
    // reset internal state
    zero_state_.store(ZERO_STATE::ZERO_FAIL);
    left_motor_position_counter_.reset();
    right_motor_position_counter_.reset();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialController::on_configure(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(), "Configuring DifferentialController from %s", previous_state.label().c_str());

    try{
        params_ = param_listener_->get_params();
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Could not get parameters: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    if(params_.chainable){
        chainable_ = true;
        RCLCPP_INFO(get_node()->get_logger(), "Chainable mode is enabled.");
    }
    else{
        chainable_ = false;
        // 订阅 command 话题
        command_sub_ = get_node()->create_subscription<rm_controller_interface::msg::EndDifferentialCommand>(std::string(this->get_node()->get_name()) + "/command", 1,
            std::bind(&DifferentialController::command_sub_callback_, this, std::placeholders::_1));
        RCLCPP_INFO(get_node()->get_logger(), "Chainable mode is disabled.");
    }
    set_chained_mode(chainable_);

    // 初始化 watchdog
    watchdog_ = std::make_shared<RM_hardware_interface::Watchdog>(0.5); // 0.5 秒没有收到命令就报警

    // initialize counters
    left_motor_position_counter_ = std::make_shared<CircleCounter>(params_.maximum_rotational_range);
    right_motor_position_counter_ = std::make_shared<CircleCounter>(params_.maximum_rotational_range);

    // zero topics
    if(params_.zero_topic.empty()){
        zero_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(std::string(this->get_node()->get_name()) + "/zero", 1,
            std::bind(&DifferentialController::zero_sub_callback_, this, std::placeholders::_1));
    }
    else{
        zero_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(params_.zero_topic, 1,
            std::bind(&DifferentialController::zero_sub_callback_, this, std::placeholders::_1));
    }

    if(!params_.force_zero_topic.empty()){
        force_zero_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(params_.force_zero_topic, 1,
            std::bind(&DifferentialController::force_zero_sub_callback_, this, std::placeholders::_1));
    }
    else{
        force_zero_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(std::string(this->get_node()->get_name()) + "/force_zero", 1,
            std::bind(&DifferentialController::force_zero_sub_callback_, this, std::placeholders::_1));
    }


    // publishers
    if(params_.is_zero_topic.empty()){
        is_zero_pub_ = get_node()->create_publisher<std_msgs::msg::Int8>(std::string(this->get_node()->get_name()) + "/is_zero", 1);
    }
    else{
        is_zero_pub_ = get_node()->create_publisher<std_msgs::msg::Int8>(params_.is_zero_topic, 1);
    }
    if(params_.joint_state_topic.empty()){
        joint_state_pub_ = get_node()->create_publisher<std_msgs::msg::Int8>(std::string(this->get_node()->get_name()) + "/joint_state", 1);
    }
    else{
        joint_state_pub_ = get_node()->create_publisher<std_msgs::msg::Int8>(params_.joint_state_topic, 1);
    }

    // store zero parameters
    zero_delta_position_ = params_.zero_delta_position;
    zero_velocity_ = params_.zero_velocity;
    zero_pitch_position_ = params_.zero_pitch_position;
    zero_roll_position_ = params_.zero_roll_position;
    zero_timeout_ = params_.zero_timeout;
    effort_enable_threshold_ = params_.effort_enable_threshold;
    maximum_rotational_range_ = params_.maximum_rotational_range;
    gear_ratio_ = params_.gear_ratio;
    stable_velocity_command_ = params_.stable_velocity_command;

    // 检查参数是否合理
    if(gear_ratio_ <= 0.0){
        RCLCPP_ERROR(get_node()->get_logger(), "Gear ratio must be positive.");
        return controller_interface::CallbackReturn::ERROR;
    }
    if(effort_enable_threshold_ < 0.0){
        RCLCPP_ERROR(get_node()->get_logger(), "Effort enable threshold must be non-negative.");
        return controller_interface::CallbackReturn::ERROR;
    }
    if(zero_timeout_ <= 0.0){
        RCLCPP_ERROR(get_node()->get_logger(), "Zero timeout must be positive.");
        return controller_interface::CallbackReturn::ERROR;
    }
    if(zero_velocity_ <= 0.0){
        RCLCPP_ERROR(get_node()->get_logger(), "Zero velocity must be positive.");
        return controller_interface::CallbackReturn::ERROR;
    }
    if(maximum_rotational_range_ <= 0.0){
        RCLCPP_ERROR(get_node()->get_logger(), "Maximum rotational range must be positive.");
        return controller_interface::CallbackReturn::ERROR;
    }

    if(params_.auto_zero_on_start){
        {
            std::lock_guard<std::mutex> lock(zero_start_time_mutex_);
            zero_state_.store(ZERO_STATE::ZERO_INPROGRESS);
            zero_start_time_ = this->get_node()->get_clock()->now();
        }
        RCLCPP_INFO(get_node()->get_logger(), "Auto zero on start is enabled, starting zeroing procedure");
    }
    else{
        zero_state_.store(ZERO_STATE::ZERO_FAIL);
        RCLCPP_INFO(get_node()->get_logger(), "Auto zero on start is disabled, controller will not work until zeroed");
    }
    

    // 输出以上的参数
    RCLCPP_INFO(get_node()->get_logger(), "Zero delta position: %f rad", zero_delta_position_);
    RCLCPP_INFO(get_node()->get_logger(), "Zero velocity: %f rad/s", zero_velocity_);
    RCLCPP_INFO(get_node()->get_logger(), "Zero pitch position: %f rad", zero_pitch_position_);
    RCLCPP_INFO(get_node()->get_logger(), "Zero roll position: %f rad", zero_roll_position_);
    RCLCPP_INFO(get_node()->get_logger(), "Zero timeout: %f s", zero_timeout_);
    RCLCPP_INFO(get_node()->get_logger(), "Effort enable threshold: %f", effort_enable_threshold_);
    RCLCPP_INFO(get_node()->get_logger(), "Maximum rotational range: %f rad", maximum_rotational_range_);
    RCLCPP_INFO(get_node()->get_logger(), "Gear ratio: %f", gear_ratio_);
    RCLCPP_INFO(get_node()->get_logger(), "Stable velocity command: %f rad/s", stable_velocity_command_);
    // 输出topic
    RCLCPP_INFO(get_node()->get_logger(), "Zero topic: %s", zero_sub_->get_topic_name());
    RCLCPP_INFO(get_node()->get_logger(), "Force zero topic: %s", force_zero_sub_->get_topic_name());
    RCLCPP_INFO(get_node()->get_logger(), "Is zero topic: %s", is_zero_pub_->get_topic_name());
    RCLCPP_INFO(get_node()->get_logger(), "Joint state topic: %s", joint_state_pub_->get_topic_name());


    // Resolve motor interface names: prefer explicit overrides, otherwise infer from motor name
    auto infer = [&](const std::string & motor, const std::string & suffix){
        return motor + "/" + suffix;
    };

    motor_left_state_position_name_ = params_.motor_left_state_position.empty() ? infer(params_.motor_left, "position") : params_.motor_left_state_position;
    motor_left_state_velocity_name_ = params_.motor_left_state_velocity.empty() ? infer(params_.motor_left, "velocity") : params_.motor_left_state_velocity;
    motor_left_state_effort_name_ = params_.motor_left_state_effort.empty() ? infer(params_.motor_left, "effort") : params_.motor_left_state_effort;
    motor_left_command_position_name_ = params_.motor_left_command_position.empty() ? infer(params_.motor_left, "position") : params_.motor_left_command_position;
    motor_left_command_velocity_name_ = params_.motor_left_command_velocity.empty() ? infer(params_.motor_left, "velocity") : params_.motor_left_command_velocity;

    motor_right_state_position_name_ = params_.motor_right_state_position.empty() ? infer(params_.motor_right, "position") : params_.motor_right_state_position;
    motor_right_state_velocity_name_ = params_.motor_right_state_velocity.empty() ? infer(params_.motor_right, "velocity") : params_.motor_right_state_velocity;
    motor_right_state_effort_name_ = params_.motor_right_state_effort.empty() ? infer(params_.motor_right, "effort") : params_.motor_right_state_effort;
    motor_right_command_position_name_ = params_.motor_right_command_position.empty() ? infer(params_.motor_right, "position") : params_.motor_right_command_position;
    motor_right_command_velocity_name_ = params_.motor_right_command_velocity.empty() ? infer(params_.motor_right, "velocity") : params_.motor_right_command_velocity;

    // 输出所有的这些name
    RCLCPP_INFO(get_node()->get_logger(), "Motor left state position interface: %s", motor_left_state_position_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor left state velocity interface: %s", motor_left_state_velocity_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor left state effort interface: %s", motor_left_state_effort_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor left command position interface: %s", motor_left_command_position_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor left command velocity interface: %s", motor_left_command_velocity_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor right state position interface: %s", motor_right_state_position_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor right state velocity interface: %s", motor_right_state_velocity_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor right state effort interface: %s", motor_right_state_effort_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor right command position interface: %s", motor_right_command_position_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Motor right command velocity interface: %s", motor_right_command_velocity_name_.c_str());
    // publishers

    try{
        const std::string controller_name = std::string(this->get_node()->get_name());
        pitch_ref_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.pitch_joint + "/command/position", 1);
        pitch_ref_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.pitch_joint + "/command/velocity", 1);
        roll_ref_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.roll_joint + "/command/position", 1); 
        roll_ref_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.roll_joint + "/command/velocity", 1);
        pitch_state_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.pitch_joint + "/state/position", 1);
        roll_state_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.roll_joint + "/state/position", 1); 
        pitch_state_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.pitch_joint + "/state/velocity", 1);
        roll_state_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.roll_joint + "/state/velocity", 1); 
        left_wheel_travel_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.motor_left + "/travel", 1);
        right_wheel_travel_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(controller_name + "/" + params_.motor_right + "/travel", 1);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create publishers: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // DEBUG pub
    #ifdef DEBUG
    try{
        call_period_publisher_ = get_node()->create_publisher<std_msgs::msg::Float32>("differential_controller/call_period", 1);
        last_time_ = get_node()->now();

        motor_left_ref_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_left + "/command/position", 1);
        motor_left_ref_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_left + "/command/velocity", 1);
        motor_left_state_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_left + "/state/position", 1);
        motor_left_state_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_left + "/state/velocity", 1);
        motor_right_ref_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_right + "/command/position", 1);
        motor_right_ref_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_right + "/command/velocity", 1);
        motor_right_state_position_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_right + "/state/position", 1);
        motor_right_state_velocity_pub_ = get_node()->create_publisher<std_msgs::msg::Float32>(std::string(this->get_node()->get_name()) + "/" + params_.motor_right + "/state/velocity", 1);

    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create debug publisher: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    
    #endif

    RCLCPP_INFO(get_node()->get_logger(), "Configured DifferentialController");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialController::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(), "Activating DifferentialController, previous state: %s", previous_state.label().c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DifferentialController::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating DifferentialController, from state: %s", previous_state.label().c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> DifferentialController::on_export_reference_interfaces(){
    std::vector<hardware_interface::CommandInterface> ifaces;
    reference_interfaces_.resize(4, 0.0);
    // roll velocity, roll position, pitch velocity, pitch position
    ifaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), params_.roll_joint + "/" + "velocity", &reference_interfaces_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX]));
    ifaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), params_.roll_joint + "/" + "position", &reference_interfaces_[CHAIN_ROLL_POSITION_COMMAND_INDEX]));
    ifaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), params_.pitch_joint + "/" + "velocity", &reference_interfaces_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX]));
    ifaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), params_.pitch_joint + "/" + "position", &reference_interfaces_[CHAIN_PITCH_POSITION_COMMAND_INDEX]));
    return ifaces;
}

// std::vector<hardware_interface::StateInterface> DifferentialController::on_export_state_interfaces(){
//     std::vector<hardware_interface::StateInterface> ifaces;
//     state_interfaces_.resize(4);
//     // roll velocity, roll position, pitch velocity, pitch position
//     ifaces.push_back(hardware_interface::StateInterface(get_node()->get_name(), params_.roll_joint + "/" + "velocity", &state_interfaces_[CHAIN_ROLL_VELOCITY_STATE_INDEX]));
//     ifaces.push_back(hardware_interface::StateInterface(get_node()->get_name(), params_.roll_joint + "/" + "position", &state_interfaces_[CHAIN_ROLL_POSITION_STATE_INDEX]));
//     ifaces.push_back(hardware_interface::StateInterface(get_node()->get_name(), params_.pitch_joint + "/" + "velocity", &state_interfaces_[CHAIN_PITCH_VELOCITY_STATE_INDEX]));
//     ifaces.push_back(hardware_interface::StateInterface(get_node()->get_name(), params_.pitch_joint + "/" + "position", &state_interfaces_[CHAIN_PITCH_POSITION_STATE_INDEX]));
//     return ifaces;
// }

controller_interface::return_type DifferentialController::update_reference_from_subscribers(){
    // no topic-based refs yet
    return controller_interface::return_type::OK;
}

bool DifferentialController::on_set_chained_mode(bool chained_mode){
    chainable_ = chained_mode;
    return true;
}

} // namespace RM_hardware_interface