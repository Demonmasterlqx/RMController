#include "differential_controller/differential_controller.hpp"

namespace RM_hardware_interface{

controller_interface::return_type DifferentialController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){

    (void)time;
    (void)period;

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

    double epli_theta_l = left_motor_position_counter_->update(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value());
    double epli_theta_r = right_motor_position_counter_->update(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value());

    double w_l = state_interfaces_[LEFT_VELOCITY_STATE_INDEX].get_value();
    double w_r = state_interfaces_[RIGHT_VELOCITY_STATE_INDEX].get_value();

    state_[CHAIN_ROLL_POSITION_STATE_INDEX] = zero_roll_position_ - gear_ratio_ * (epli_theta_l + epli_theta_r) * 0.5;
    state_[CHAIN_ROLL_VELOCITY_STATE_INDEX] = - gear_ratio_ * 0.5 * (w_l + w_r);
    state_[CHAIN_PITCH_POSITION_STATE_INDEX] = zero_pitch_position_ + (epli_theta_r - epli_theta_l) * 0.5;
    state_[CHAIN_PITCH_VELOCITY_STATE_INDEX] = 0.5 * (w_r - w_l);

    // 输出 state_
    // RCLCPP_INFO(this->get_node()->get_logger(), "state_: roll_pos: %.4f, roll_vel: %.4f, pitch_pos: %.4f, pitch_vel: %.4f",
    //             state_[CHAIN_ROLL_POSITION_STATE_INDEX], state_[CHAIN_ROLL_VELOCITY_STATE_INDEX],
    //             state_[CHAIN_PITCH_POSITION_STATE_INDEX], state_[CHAIN_PITCH_VELOCITY_STATE_INDEX]);


    // 刷新参考值
    if(chainable_){
        reference_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX] = reference_interfaces_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX];
        reference_[CHAIN_ROLL_POSITION_COMMAND_INDEX] = reference_interfaces_[CHAIN_ROLL_POSITION_COMMAND_INDEX];
        reference_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX] = reference_interfaces_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX];
        reference_[CHAIN_PITCH_POSITION_COMMAND_INDEX] = reference_interfaces_[CHAIN_PITCH_POSITION_COMMAND_INDEX];
        watchdog_->reset();
    }
    else{
        auto cmd = last_command_msg_.readFromRT();
        if(cmd != nullptr){
            reference_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX] = cmd->roll_velocity;
            reference_[CHAIN_ROLL_POSITION_COMMAND_INDEX] = cmd->roll_position;
            reference_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX] = cmd->pitch_velocity;
            reference_[CHAIN_PITCH_POSITION_COMMAND_INDEX] = cmd->pitch_position;
            watchdog_->set_used();
        }
        else{
            // 没有收到过命令
            reference_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX] = 0.0;
            reference_[CHAIN_ROLL_POSITION_COMMAND_INDEX] = 0.0;
            reference_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX] = 0.0;
            reference_[CHAIN_PITCH_POSITION_COMMAND_INDEX] = 0.0;
        }

        if(!watchdog_->is_triggered()){
            reference_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX] = 0.0;
            reference_[CHAIN_ROLL_POSITION_COMMAND_INDEX] = state_[CHAIN_ROLL_POSITION_STATE_INDEX];
            reference_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX] = 0.0;
            reference_[CHAIN_PITCH_POSITION_COMMAND_INDEX] = state_[CHAIN_PITCH_POSITION_STATE_INDEX];
        }
    }

    // 按照 zero 的状态
    switch (zero_state_.load()) {
        case ZERO_STATE::ZERO_FAIL:

            process_zero_failure_();
            break;
        case ZERO_STATE::ZERO_INPROGRESS:
            try{
                process_zero_();
                zero_call_attempts_=0;
            }
            catch(const std::exception & e){
                RCLCPP_ERROR(this->get_node()->get_logger(), "DifferentialController: zero process error: %s", e.what());
                zero_state_.store(ZERO_STATE::ZERO_FAIL);
            }
            break;
        case ZERO_STATE::ZERO_FORCE:
            try{
                process_force_zero_();
                zero_call_attempts_=0;
            }
            catch(const std::exception & e){
                RCLCPP_ERROR(this->get_node()->get_logger(), "DifferentialController: force zero process error: %s", e.what());
                zero_state_.store(ZERO_STATE::ZERO_FAIL);
            }
            break;
        case ZERO_STATE::ZERO_OK:
            process_command_();
            zero_call_attempts_=0;
            break;
        default:
            RCLCPP_ERROR(this->get_node()->get_logger(), "DifferentialController: unknown zero state!");
            zero_state_.store(ZERO_STATE::ZERO_FAIL);
            break;
    }

    // publish state
    try{
        std_msgs::msg::Float32 msg;
        msg.data = state_[CHAIN_PITCH_POSITION_STATE_INDEX];
        pitch_state_position_pub_->publish(msg);
        msg.data = state_[CHAIN_PITCH_VELOCITY_STATE_INDEX];
        pitch_state_velocity_pub_->publish(msg);
        msg.data = state_[CHAIN_ROLL_POSITION_STATE_INDEX];
        roll_state_position_pub_->publish(msg);
        msg.data = state_[CHAIN_ROLL_VELOCITY_STATE_INDEX];
        roll_state_velocity_pub_->publish(msg);

        msg.data = reference_[CHAIN_PITCH_POSITION_COMMAND_INDEX];
        pitch_ref_position_pub_->publish(msg);
        msg.data = reference_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX];
        pitch_ref_velocity_pub_->publish(msg);
        msg.data = reference_[CHAIN_ROLL_POSITION_COMMAND_INDEX];
        roll_ref_position_pub_->publish(msg);
        msg.data = reference_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX];
        roll_ref_velocity_pub_->publish(msg);

        // 发布展开的路程
        msg.data = epli_theta_l;
        left_wheel_travel_pub_->publish(msg);
        msg.data = epli_theta_r;
        right_wheel_travel_pub_->publish(msg);

        // publish is_zero
        std_msgs::msg::Int8 zero_msg;
        zero_msg.data = static_cast<int8_t>(zero_state_.load());
        is_zero_pub_->publish(zero_msg);



    }
    catch(const std::exception & e){
        RCLCPP_ERROR(this->get_node()->get_logger(), "DifferentialController: publish state error: %s", e.what());
    }


    #ifdef DEBUG
    // 发布电机相关调试信息
    try{
        std_msgs::msg::Float32 msg;
        msg.data = command_interfaces_[LEFT_POSITION_COMMAND_INDEX].get_value();
        motor_left_ref_position_pub_->publish(msg);
        msg.data = command_interfaces_[LEFT_VELOCITY_COMMAND_INDEX].get_value();
        motor_left_ref_velocity_pub_->publish(msg);
        msg.data = state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value();
        motor_left_state_position_pub_->publish(msg);
        msg.data = state_interfaces_[LEFT_VELOCITY_STATE_INDEX].get_value();
        motor_left_state_velocity_pub_->publish(msg);
        msg.data = command_interfaces_[RIGHT_POSITION_COMMAND_INDEX].get_value();
        motor_right_ref_position_pub_->publish(msg);
        msg.data = command_interfaces_[RIGHT_VELOCITY_COMMAND_INDEX].get_value();
        motor_right_ref_velocity_pub_->publish(msg);
        msg.data = state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value();
        motor_right_state_position_pub_->publish(msg);
        msg.data = state_interfaces_[RIGHT_VELOCITY_STATE_INDEX].get_value();
        motor_right_state_velocity_pub_->publish(msg);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(this->get_node()->get_logger(), "DifferentialController: publish motor debug error: %s", e.what());
    }
    #endif

    last_process_zero_left_position_ = state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value();
    last_process_zero_right_position_ = state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value();

    return controller_interface::return_type::OK;
}

void DifferentialController::process_zero_(){

    static const double eps = 1e-4;

    std::lock_guard<std::mutex> lock(zero_start_time_mutex_);
    auto now = this->get_node()->get_clock()->now();
    // 判断是否超时
    if((now - zero_start_time_).seconds() > params_.zero_timeout){
        RCLCPP_ERROR(this->get_node()->get_logger(), "DifferentialController: zero timeout!");
        zero_state_.store(ZERO_STATE::ZERO_FAIL);
        return;
    }
    bool is_enabled = true;
    // 判断是否使能
    if(std::abs(state_interfaces_[LEFT_EFFORT_STATE_INDEX].get_value()) < effort_enable_threshold_ ||
       std::abs(state_interfaces_[RIGHT_EFFORT_STATE_INDEX].get_value()) < effort_enable_threshold_){
        // 力矩不够，无法置零
        zero_effort_not_enable_attempts_ ++;
        if(zero_effort_not_enable_attempts_ >= 1000){
            zero_effort_not_enable_attempts_ = 0;
            RCLCPP_WARN(this->get_node()->get_logger(), "DifferentialController: zero effort not enable, please check the hardware!");
        }
        is_enabled = false;
    }

    // 如果发现一个还在转，另一个没转了，直接false
    if((std::abs(state_interfaces_[LEFT_VELOCITY_STATE_INDEX].get_value()) > eps &&
        std::abs(state_interfaces_[RIGHT_VELOCITY_STATE_INDEX].get_value()) < eps) ||
       (std::abs(state_interfaces_[RIGHT_VELOCITY_STATE_INDEX].get_value()) > eps &&
        std::abs(state_interfaces_[LEFT_VELOCITY_STATE_INDEX].get_value()) < eps)){
        zero_one_motor_stall_attempts_++;
        if (zero_one_motor_stall_attempts_ >= 1000){
            RCLCPP_WARN(this->get_node()->get_logger(), "DifferentialController: zero state fail due to one motor stop but the other is still running, please set zero or check the hardware!");
            zero_state_.store(ZERO_STATE::ZERO_FAIL);
            // 将命令置零
            command_interfaces_[LEFT_VELOCITY_COMMAND_INDEX].set_value(0.0);
            command_interfaces_[RIGHT_VELOCITY_COMMAND_INDEX].set_value(0.0);
            command_interfaces_[LEFT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value());
            command_interfaces_[RIGHT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value());
            return;
        }
    }
    else{
        zero_one_motor_stall_attempts_ = 0;
    }

    zero_effort_not_enable_attempts_ = 0;

    command_interfaces_[LEFT_VELOCITY_COMMAND_INDEX].set_value(zero_velocity_);
    command_interfaces_[RIGHT_VELOCITY_COMMAND_INDEX].set_value(zero_velocity_);
    command_interfaces_[LEFT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value() - zero_delta_position_);
    command_interfaces_[RIGHT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value() + zero_delta_position_);

    if (std::abs(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value() - last_process_zero_left_position_) < 1e-6 &&
         std::abs(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value() - last_process_zero_right_position_) < 1e-6 && is_enabled) {
        // 位置没有变化，到达限位
        zero_reach_limit_attempts_++;
    }
    else{
        zero_reach_limit_attempts_ = 0;
    }

    if (zero_reach_limit_attempts_ >= 200){
        zero_state_.store(ZERO_STATE::ZERO_OK);
        left_motor_position_counter_->reset(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value());
        right_motor_position_counter_->reset(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value());
        // 将命令全部清零
        command_interfaces_[LEFT_VELOCITY_COMMAND_INDEX].set_value(0.0);
        command_interfaces_[RIGHT_VELOCITY_COMMAND_INDEX].set_value(0.0);
        command_interfaces_[LEFT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value());
        command_interfaces_[RIGHT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value());
        RCLCPP_INFO(this->get_node()->get_logger(), "DifferentialController: zero success!");
        return;
    }

}

void DifferentialController::process_command_(){

    double left_velocity = -reference_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX] - reference_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX] / gear_ratio_;
    double right_velocity = reference_[CHAIN_PITCH_VELOCITY_COMMAND_INDEX] - reference_[CHAIN_ROLL_VELOCITY_COMMAND_INDEX] / gear_ratio_;

    double delta_P = reference_[CHAIN_PITCH_POSITION_COMMAND_INDEX] - state_[CHAIN_PITCH_POSITION_STATE_INDEX];
    double delta_R = reference_[CHAIN_ROLL_POSITION_COMMAND_INDEX] - state_[CHAIN_ROLL_POSITION_STATE_INDEX];

    double delta_left_position = -delta_P - delta_R / gear_ratio_;
    double delta_right_position = delta_P - delta_R / gear_ratio_;

    command_interfaces_[LEFT_VELOCITY_COMMAND_INDEX].set_value(left_velocity);
    command_interfaces_[RIGHT_VELOCITY_COMMAND_INDEX].set_value(right_velocity);
    command_interfaces_[LEFT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value() + delta_left_position);
    command_interfaces_[RIGHT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value() + delta_right_position);

    return;

}

void DifferentialController::process_zero_failure_(){
    zero_call_attempts_ += 1;
    if(zero_call_attempts_ >= 1000){
        zero_call_attempts_ = 0;
        RCLCPP_WARN(this->get_node()->get_logger(), "DifferentialController: zero state fail, please set zero or check the hardware!");
    }

    // 将命令接口全部清零
    command_interfaces_[LEFT_VELOCITY_COMMAND_INDEX].set_value(0.0);
    command_interfaces_[RIGHT_VELOCITY_COMMAND_INDEX].set_value(0.0);
    command_interfaces_[LEFT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value());
    command_interfaces_[RIGHT_POSITION_COMMAND_INDEX].set_value(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value());

}

void DifferentialController::process_force_zero_(){

    // 直接置零
    try{
        left_motor_position_counter_->reset(state_interfaces_[LEFT_POSITION_STATE_INDEX].get_value());
        right_motor_position_counter_->reset(state_interfaces_[RIGHT_POSITION_STATE_INDEX].get_value());
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(this->get_node()->get_logger(), "DifferentialController: force zero reset error: %s", e.what());
        zero_state_.store(ZERO_STATE::ZERO_FAIL);
        return;
    }
    
    RCLCPP_INFO(this->get_node()->get_logger(), "DifferentialController: force zero success!");
    zero_state_.store(ZERO_STATE::ZERO_OK);

}

} // namespace RM_hardware_interface