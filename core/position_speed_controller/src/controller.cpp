#include "position_speed_controller/position_speed_controller.hpp"

namespace RM_hardware_interface{

controller_interface::return_type PositionSpeedController::update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period){

    (void)period;

    #ifdef DEBUG

    try{
        auto msg = std_msgs::msg::Float32();
        msg.data = (time - last_time_).seconds();
        last_time_ = time;
        call_period_publisher_->publish(msg);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing debug time interval: %s",e.what());
    }

    #endif

    // 获取数据
    double current_position = 0;
    double current_speed = 0;
    double current_effort = 0;
    double position_command=0.0;
    double speed_command=0.0;

    try{
        current_position = state_interfaces_[POSITION_STATE_INDEX].get_value();
        current_speed = state_interfaces_[SPEED_STATE_INDEX].get_value();
        current_effort = state_interfaces_[EFFORT_STATE_INDEX].get_value();
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during getting state interface value: %s",e.what());
        return controller_interface::return_type::ERROR;
    }

    // 获取命令
    if(chainable_){
        speed_command = reference_interfaces_[CHAINED_SPEED_COMMAND_INDEX];
        position_command = reference_interfaces_[CHAINED_POSITION_COMMAND_INDEX];
        watchdog_->reset();
    }
    else{
        auto command = command_buffer_.readFromRT();
        if(command!=nullptr){
            position_command = command->position;
            speed_command = command->speed;
        }
        watchdog_->set_used();
    }

    if (watchdog_->is_triggered()){
        position_command = current_position;
        speed_command = 0.0;
    }

    // 对 position_command 限幅
    // position_command = std::min(std::max(position_command, pos_min_), pos_max_);
    position_command = position_command - (pos_max_ - pos_min_)*int(position_command / (pos_max_ - pos_min_));

    double effort_command=0.0;

    double position_error = 0;
    // 计算最短路径后的速度和力矩方向
    double go_direction = 0;

    // 自动选择最小路径

    if (std::abs(current_position - position_command) > (pos_max_ - pos_min_) * 0.5) {
        if (current_position > position_command) {
            position_error = (pos_max_ - current_position) + (position_command - pos_min_);
            go_direction = 1.0;
        } else {
            position_error = -((pos_max_ - position_command) + (current_position - pos_min_));
            go_direction = -1.0;
            RCLCPP_DEBUG(get_node()->get_logger(),"go_direction set to -1");
        }
    }
    else {
        position_error = position_command - current_position;
        go_direction = (position_error >= 0) ? 1.0 : -1.0;
    }
    // 输出这里的几个值
    // RCLCPP_INFO(get_node()->get_logger(),"current_position: %f, position_command: %f",current_position,position_command);
    // RCLCPP_INFO(get_node()->get_logger(),"pos_min_: %f, pos_max_: %f",pos_min_,pos_max_);
    // RCLCPP_INFO(get_node()->get_logger(),"position_error: %f, go_direction: %f",position_error,go_direction);

    position_pid_->calculate(0, position_error);
    double position_output = position_pid_->get_output();
    if (std::abs(position_output) > std::abs(speed_command)){
        position_output = std::abs(speed_command) * go_direction;
    }
    speed_pid_->calculate(current_speed, position_output);
    effort_command = speed_pid_->get_output();

    // 写入命令
    try{
        command_interfaces_[EFFORT_COMMAND_INDEX].set_value(effort_command);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during setting command interface value: %s",e.what());
        return controller_interface::return_type::ERROR;
    }

    // pub
    try{
        auto msg = std_msgs::msg::Float32();
        msg.data = position_command;
        position_reference_publisher_->publish(msg);
        msg.data = speed_command;
        speed_reference_publisher_->publish(msg);
        msg.data = current_position;
        position_state_publisher_->publish(msg);
        msg.data = current_speed;
        velocity_state_publisher_->publish(msg);
        msg.data = current_effort;
        effort_state_publisher_->publish(msg);
        msg.data = effort_command;
        effort_reference_publisher_->publish(msg);
        msg.data = position_output;
        position_pid_output_publisher_->publish(msg);

        #ifdef DEBUG
        auto dir_msg = std_msgs::msg::Float32();
        dir_msg.data = (go_direction);
        direction_publisher_->publish(dir_msg);
        auto err_msg = std_msgs::msg::Float32();
        err_msg.data = position_error;
        position_error_publisher_->publish(err_msg);
        #endif

    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing state or reference: %s",e.what());
    }

    return controller_interface::return_type::OK;

}

} // namespace RM_hardware_interface