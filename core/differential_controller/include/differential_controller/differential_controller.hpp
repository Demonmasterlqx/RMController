#ifndef DIFFERENTIAL_CONTROLLER_HPP_
#define DIFFERENTIAL_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "differential_controller/differential_controller_parameters.hpp"
#include "differential_controller/circle_counter.hpp"

#include "watchdog/watchdog.hpp"


namespace RM_hardware_interface{

// 标零状态
enum class ZERO_STATE{
    ZERO_FAIL = -1,
    ZERO_INPROGRESS = 0,
    ZERO_OK = 1
};

class DifferentialController : public controller_interface::ChainableControllerInterface{
public:

    DifferentialController();
    ~DifferentialController()=default;


    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    controller_interface::return_type update_reference_from_subscribers() override;

    bool on_set_chained_mode(bool chained_mode) override;

private:

    // hardware 层的 state / command 接口
    // command
    const int LEFT_POSITION_COMMAND_INDEX = 0;
    const int LEFT_VELOCITY_COMMAND_INDEX = 1;
    const int RIGHT_POSITION_COMMAND_INDEX = 2;
    const int RIGHT_VELOCITY_COMMAND_INDEX = 3;
    // state
    const int LEFT_EFFORT_STATE_INDEX = 0;
    const int RIGHT_EFFORT_STATE_INDEX = 1;
    const int LEFT_POSITION_STATE_INDEX = 2;
    const int RIGHT_POSITION_STATE_INDEX = 3;
    const int LEFT_VELOCITY_STATE_INDEX = 4;
    const int RIGHT_VELOCITY_STATE_INDEX = 5;

    // chainable 的 command
    const int CHAIN_ROLL_VELOCITY_COMMAND_INDEX = 0;
    const int CHAIN_ROLL_POSITION_COMMAND_INDEX = 1;
    const int CHAIN_PITCH_VELOCITY_COMMAND_INDEX = 2;
    const int CHAIN_PITCH_POSITION_COMMAND_INDEX = 3;

    // chainable 的 state
    const int CHAIN_ROLL_VELOCITY_STATE_INDEX = 0;
    const int CHAIN_ROLL_POSITION_STATE_INDEX = 1;
    const int CHAIN_PITCH_VELOCITY_STATE_INDEX = 2;
    const int CHAIN_PITCH_POSITION_STATE_INDEX = 3;

    // 参数获取
    std::shared_ptr<differential_controller::ParamListener> param_listener_=nullptr;

    // 参数
    differential_controller::Params params_;

    // 是否启动链式控制
    bool chainable_=false;

    // 看门狗
    std::shared_ptr<Watchdog> watchdog_=nullptr;

    // pub
    // Differential joint publishers (pitch)
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pitch_ref_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pitch_ref_velocity_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pitch_state_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pitch_state_velocity_pub_ = nullptr;

    // Differential joint publishers (roll)
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> roll_ref_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> roll_ref_velocity_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> roll_state_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> roll_state_velocity_pub_ = nullptr;

    // is_zero status publisher (int8: -1 fail, 0 in_progress, 1 success)
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int8>> is_zero_pub_ = nullptr;


    #ifdef DEBUG
    // 调试信息发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> call_period_publisher_=nullptr;
    rclcpp::Time last_time_;
    // // Motor publishers (left)
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_ref_position_pub_ = nullptr;
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_ref_velocity_pub_ = nullptr;
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_state_position_pub_ = nullptr;
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_state_velocity_pub_ = nullptr;

    // // Motor publishers (right)
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_ref_position_pub_ = nullptr;
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_ref_velocity_pub_ = nullptr;
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_state_position_pub_ = nullptr;
    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_state_velocity_pub_ = nullptr;

    #endif

    // sub
    // 置零sub
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> zero_sub_ = nullptr;
    // 强制置零 sub
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> force_zero_sub_ = nullptr;

    // 标零参数
    // 标零时的 增加的position 的值
    double zero_delta_position_ = 0.0;
    // 标零时的速度
    double zero_velocity_ = 0.0;

    // 标零后的 pitch 的position
    double zero_pitch_position_ = 0.0;
    // 标零后的 roll 的position
    double zero_roll_position_ = 0.0;
    // 标零的最大时间
    double zero_timeout_ = 0.0;
    // 用来判断单机是否使能
    double effort_enable_threshold_ = 0.0;

    // 最大最小的旋转范围的差值
    double maximum_rotational_range_ = 0.0;

    // 置零状态
    std::atomic<ZERO_STATE> zero_state_{ZERO_STATE::ZERO_FAIL};
    // 置零计时器
    rclcpp::Time zero_start_time_ = rclcpp::Clock().now();

    // 两个电机的位置计算器

    std::shared_ptr<CircleCounter> left_motor_position_counter_;
    std::shared_ptr<CircleCounter> right_motor_position_counter_;

    // Resolved motor interface names (after applying optional overrides)
    std::string motor_left_state_position_name_;
    std::string motor_left_state_velocity_name_;
    std::string motor_left_state_effort_name_;
    std::string motor_left_command_position_name_;
    std::string motor_left_command_velocity_name_;

    std::string motor_right_state_position_name_;
    std::string motor_right_state_velocity_name_;
    std::string motor_right_state_effort_name_;
    std::string motor_right_command_position_name_;
    std::string motor_right_command_velocity_name_;

    // chain reference storage (velocity,pos for roll and pitch)
    // call_back
    void zero_sub_callback_(const std_msgs::msg::Bool::SharedPtr msg);
    void force_zero_sub_callback_(const std_msgs::msg::Bool::SharedPtr msg);

}; // class DifferentialController



} // namespace RM_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::DifferentialController, controller_interface::ChainableControllerInterface);


#endif // DIFFERENTIAL_CONTROLLER_HPP_