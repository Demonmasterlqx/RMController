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
    }

    if (watchdog_->is_triggered()){
        RCLCPP_WARN(get_node()->get_logger(),"Watchdog triggered! Stopping the controller.");
        speed_command = 0.0;
        position_command = current_position;
    }

    // 控制逻辑
    double effort_command=0.0;
    double trajecotry_position=0.0;
    double trajecotry_speed=0.0;

    // 轨迹生成
    speed_command = trajectory_generator_->set_target(position_command, 
        speed_command,
        current_position,
        current_speed);
    
    bool active = trajectory_generator_->update(trajecotry_position, trajecotry_speed);

    if(!active){
        RCLCPP_DEBUG(get_node()->get_logger(),"Trajectory generation completed.");
        trajecotry_position = current_position;
        trajecotry_speed = 0.0;
    }

    // 发现误差过大，重新规划轨迹
    if(std::abs(trajecotry_position - current_position) > replan_threshold_){
        RCLCPP_WARN(get_node()->get_logger(),"Trajectory error too large, replanning trajectory.");
        speed_command = trajectory_generator_->set_target(position_command, 
            speed_command,
            current_position,
            current_speed,
            true);
        trajectory_generator_->update(trajecotry_position, trajecotry_speed);
    }

    // 位置 PID
    position_pid_->calculate(current_position, trajecotry_position);
    double position_output = position_pid_->get_output();
    // 速度 PID
    speed_pid_->calculate(current_speed, trajecotry_speed + position_output);
    effort_command = speed_pid_->get_output();

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
        msg.data = trajecotry_position;
        trajectory_position_publisher_->publish(msg);
        msg.data = trajecotry_speed;
        trajectory_speed_publisher_->publish(msg);
        msg.data = (active ? 1.0 : 0.0);
        trajectory_state_publisher_->publish(msg);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_node()->get_logger(),"Exception during publishing state or reference: %s",e.what());
    }

    return controller_interface::return_type::OK;

}

} // namespace RM_hardware_interface