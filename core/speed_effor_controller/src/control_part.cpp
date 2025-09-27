#include "speed_effor_controller/speed_effor_controller.hpp"

namespace RM_hardware_interface{

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

    // 斜率限制
    double delta = (reference_speed - last_reference_speed);
    if(std::abs(delta) > max_delta_){
        reference_speed = last_reference_speed + std::copysign(max_delta_, delta);
    }


    #ifdef DEBUG

    try{
        auto msg = std_msgs::msg::Float32();
        msg.data = delta;
        debug_delta_publisher_->publish(msg);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing debug delta: %s",e.what());
    }

    try{
        auto msg = std_msgs::msg::Float32();
        msg.data = reference_speed - last_reference_speed;
        debug_after_delta_publisher_->publish(msg);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing debug after delta: %s",e.what());
    }

    #endif

    last_reference_speed = reference_speed;


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


} // namespace RM_hardware_interface