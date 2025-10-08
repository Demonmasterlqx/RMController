#include "differential_controller_hardware_interface/differential_controller_hardware_interface.hpp"

namespace RM_hardware_interface{

DifferentialControllerHardwareInterface::DifferentialControllerHardwareInterface(){

}


std::vector<StateInterface> DifferentialControllerHardwareInterface::export_state_interfaces(){

    return state_interfaces_;

}

std::vector<CommandInterface> DifferentialControllerHardwareInterface::export_command_interfaces(){
    return {};
}

return_type DifferentialControllerHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){

    (void)time;
    (void)period;

    for(auto & state_ptr : state_ptrs_){

        if (!state_ptr->new_data_available){
            state_ptr->no_read_count++;
        }

        auto data = state_ptr->buffer.readFromRT();
        if(data != nullptr){
            *(state_ptr->data_ptr) = *data;
            state_ptr->new_data_available = false;
        }

        if(state_ptr->no_read_count > 1000){
            RCLCPP_WARN(node_->get_logger(), "No data received for state interface: %s", state_ptr->name.c_str());
            state_ptr->no_read_count = 0;
        }
    }

    return return_type::OK;
}

return_type DifferentialControllerHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){

    (void)time;
    (void)period;

    return return_type::OK;

}

CallbackReturn DifferentialControllerHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(node_->get_logger(), "Configuring DifferentialControllerHardwareInterface, previous state: %s", previous_state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn DifferentialControllerHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(node_->get_logger(), "Cleaning up DifferentialControllerHardwareInterface, previous state: %s", previous_state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn DifferentialControllerHardwareInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(node_->get_logger(), "Shutting down DifferentialControllerHardwareInterface, previous state: %s", previous_state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn DifferentialControllerHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(node_->get_logger(), "Activating DifferentialControllerHardwareInterface, previous state: %s", previous_state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn DifferentialControllerHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(node_->get_logger(), "Deactivating DifferentialControllerHardwareInterface, from state: %s", previous_state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn DifferentialControllerHardwareInterface::on_error(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(node_->get_logger(), "Error in DifferentialControllerHardwareInterface, from state: %s", previous_state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn DifferentialControllerHardwareInterface::on_init(const HardwareInfo & hardware_info){
    node_ = rclcpp::Node::make_shared(hardware_info.name);

    // 解析 hardware_info, 创建 state 接口
    for(const auto & component : hardware_info.joints){
        const std::string & joint_name = component.name;
        if(component.parameters.find("controller_name") == component.parameters.end()){
            RCLCPP_ERROR(node_->get_logger(), "Missing parameter 'controller_name' in joint '%s'", joint_name.c_str());
            return CallbackReturn::ERROR;
        }
        std::string controller_name = component.parameters.at("controller_name");

        // 创建 state velocity 接口
        state_ptrs_.push_back(std::make_shared<StateInfo>(
            controller_name + "/" + joint_name + "/state/velocity", node_
        ));
        state_interfaces_.emplace_back(
            StateInterface(
                controller_name,
                joint_name + "/state/velocity",
                state_ptrs_.back()->data_ptr.get()
            )
        );

        // 创建 state position 接口
        state_ptrs_.push_back(std::make_shared<StateInfo>(
            controller_name + "/" + joint_name + "/state/position", node_
        ));
        state_interfaces_.emplace_back(
            StateInterface(
                controller_name,
                joint_name + "/state/position",
                state_ptrs_.back()->data_ptr.get()
            )
        );

        RCLCPP_INFO(node_->get_logger(), "Created state interfaces for joint '%s' under controller '%s'", joint_name.c_str(), controller_name.c_str());

    }

    // 启动 node spin 线程

    node_thread_ = std::make_shared<std::thread>([this]() {
        while(rclcpp::ok()) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    RCLCPP_INFO(node_->get_logger(), "DifferentialControllerHardwareInterface initialized with %zu state interfaces", state_interfaces_.size());
    return CallbackReturn::SUCCESS;

}

} // namespace RM_hardware_interface