#ifndef DIFFERENTIAL_CONTROLLER_HPP_
#define DIFFERENTIAL_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <mutex>

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

#include "rm_controller_interface/msg/end_differential_command.hpp"

#include "differential_controller/differential_controller_parameters.hpp"
#include "differential_controller/circle_counter.hpp"

#include "watchdog/watchdog.hpp"


namespace RM_hardware_interface{

// 标零状态
enum class ZERO_STATE{
    ZERO_FAIL = -1,
    ZERO_INPROGRESS = 0,
    ZERO_FORCE = 2,
    ZERO_OK = 1
};

struct EndDifferentialState{
    double roll_position=0.0;
    double roll_velocity=0.0;
    double pitch_position=0.0;
    double pitch_velocity=0.0;
};

// 只从ref角度，指示电机当前的状态
enum class JointState{
    STABLE = 0,
    MOVING = 1,
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

    // 展开的路程 publishers
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> left_wheel_travel_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> right_wheel_travel_pub_ = nullptr;

    // is_zero status publisher (int8: -1 fail, 0 in_progress, 1 success)
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int8>> is_zero_pub_ = nullptr;
    // joint state publisher (int8: 0 stable, 1 moving)
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int8>> joint_state_pub_ = nullptr;

    #ifdef DEBUG
    // 调试信息发布器
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> call_period_publisher_=nullptr;
    rclcpp::Time last_time_;
    // Motor publishers (left)
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_ref_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_ref_velocity_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_state_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_left_state_velocity_pub_ = nullptr;

    // Motor publishers (right)
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_ref_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_ref_velocity_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_state_position_pub_ = nullptr;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> motor_right_state_velocity_pub_ = nullptr;

    #endif

    // sub
    // 置零sub
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> zero_sub_ = nullptr;
    // 强制置零 sub
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> force_zero_sub_ = nullptr;
    // command sub
    std::shared_ptr<rclcpp::Subscription<rm_controller_interface::msg::EndDifferentialCommand>> command_sub_ = nullptr;

    // 记录最新的 command 消息
    realtime_tools::RealtimeBuffer<rm_controller_interface::msg::EndDifferentialCommand> last_command_msg_;

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
    // 齿轮比
    double gear_ratio_ = 0.0;

    // 最大最小的旋转范围的差值
    double maximum_rotational_range_ = 0.0;

    // 记录在没有置零的时候call update 的次数
    int zero_call_attempts_ = 0;

    // 记录启动了置零但是没有使能的次数
    int zero_effort_not_enable_attempts_ = 0;

    // 记录电机一个停转了另外一个宕机了的次数
    int zero_one_motor_stall_attempts_ = 0;

    // 记录电机到达限位的次数
    int zero_reach_limit_attempts_ = 0;

    // 前一次被调用时的 left position
    double last_process_zero_left_position_ = -1e9;
    // 前一次被调用时的 right position
    double last_process_zero_right_position_ = -1e9;

    // 进入静止状态时发出的velocity命令
    double stable_velocity_command_ = 0.0;

    // 置零处理函数，只会在 update_and_write_commands 中调用
    void process_zero_();

    // 命令处理函数，只会在 update_and_write_commands 中调用
    void process_command_();

    // 置零失败处理函数 只会在 update_and_write_commands 中调用，在没有置零时，调用update会调用这个函数
    void process_zero_failure_();

    // 强制置零处理函数 只会在 update_and_write_commands 中调用
    void process_force_zero_();

    // 最新的参考值：roll_pos, roll_vel, pitch_pos, pitch_vel
    std::vector<double> reference_{0.0, 0.0, 0.0, 0.0};

    // 上一次的参考值：roll_pos, roll_vel, pitch_pos, pitch_vel
    std::vector<double> last_reference_{0.0, 0.0, 0.0, 0.0};

    // 最新的状态值：roll_pos, roll_vel, pitch_pos, pitch_vel
    std::vector<double> state_{0.0, 0.0, 0.0, 0.0};

    // stable 状态的期望位置/速度
    // 在置零成功的时候，会给这个值赋值，赋值为置零时的位置
    // 在MOVEING 的时候，这个值等于 state_
    std::vector<double> stable_position_{0.0, 0.0, 0.0, 0.0};

    /**
     * @brief 用于检查当前的状态和期望的稳定值的差距是不是太大，防止出现因为手动移动导致的超大误差
     * 
     * @return true 
     * @return false 
     */
    bool calculate_stable_state_();

    std::atomic<JointState> joint_state_{JointState::MOVING};

    // 置零状态
    std::atomic<ZERO_STATE> zero_state_{ZERO_STATE::ZERO_FAIL};
    // 置零计时器 互斥锁
    std::mutex zero_start_time_mutex_;
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
    void command_sub_callback_(const rm_controller_interface::msg::EndDifferentialCommand::SharedPtr msg);

    // /**
    //  * @brief 用于计算差速器位置速度的部分，会计算并更新state_变量，并且更新 left_motor_position_counter_ right_motor_position_counter_
    //  * 
    //  * @return EndDifferentialState 
    //  */
    // EndDifferentialState calculate_differential_state_();

}; // class DifferentialController



} // namespace RM_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RM_hardware_interface::DifferentialController, controller_interface::ChainableControllerInterface);


#endif // DIFFERENTIAL_CONTROLLER_HPP_