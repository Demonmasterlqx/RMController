#ifndef POSITION_SPEED_CONTROLLER__TRAJECTORY_GENERATOR_HPP_
#define POSITION_SPEED_CONTROLLER__TRAJECTORY_GENERATOR_HPP_

#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"

namespace RM_hardware_interface{

/**
 * @brief 轨迹生成器，给定最大速度和期望位置用于生成梯形速度曲线
 * 
 */
class TrajectoryGenerator{

public:
    /**
     * @brief 构造 函数
     * 
     * @param max_velocity_ 全局最大速度，默认为0，输入负数会被取绝对值
     * @param max_acceleration_ 最大加速度，默认为0，输入负数会被取绝对值
     */
    TrajectoryGenerator(double max_velocity_, double max_acceleration_):
        max_velocity_(std::abs(max_velocity_)),
        max_acceleration_(std::abs(max_acceleration_)),
        target_position_(0),
        target_velocity_(0),
        start_position_(0),
        start_velocity_(0),
        active_(false),
        start_time_(rclcpp::Clock().now())
    {}

    /**
     * @brief 设置目标
     * 
     * @param position 目标位置
     * @param velocity 期望速度，会按照pos的方向取正负
     * @param current_position 当前电机位置
     * @param current_velocity 当前电机速度
     * 
     * @return double desired_velocity的实际值，包括符号
     */
    double set_target(double desired_position, double desired_velocity, double current_position, double current_velocity, bool force = false){

        // 检测发现期望没有变化，直接返回
        if(active_ && std::abs(desired_position - target_position_) < 1e-6 && std::abs(std::abs(desired_velocity) - std::abs(target_velocity_)) < 1e-6 && !force){
            return target_velocity_;
        }

        // 如果期望速度为0，则不做处理
        if(std::abs(desired_velocity) <= 1e-9){
            active_ = false;
            return 0.0;
        }

        start_time_ = rclcpp::Clock().now();

        active_ = true;
        target_position_ = desired_position;
        target_velocity_ = std::min(std::abs(desired_velocity), max_velocity_);
        start_position_ = current_position;
        start_velocity_ = current_velocity;

        // 计算时间阶段
        double delta_position = (target_position_ - start_position_);
        // 加速方向
        sign = (target_position_ > start_position_) ? 1 : -1;
        // 减速方向
        res_sign = (target_position_ > start_position_) ? -1 : 1;
        target_velocity_ = std::abs(target_velocity_) * sign;

        // 判断是使用三角曲线还是梯形曲线
        double need_to_accelerate_distance = (target_velocity_ * target_velocity_*0.5 - start_velocity_ * start_velocity_ * 0.5) / (max_acceleration_*sign) - target_velocity_ * target_velocity_ * 0.5 / (max_acceleration_ * res_sign);
        if (std::abs(need_to_accelerate_distance) > std::abs(delta_position)) {
            // 三角曲线

            double v_peak = std::sqrt(std::max(0.0,sign * max_acceleration_ * delta_position + start_velocity_ * start_velocity_ * 0.5)) * sign;

            t1_ = (v_peak - start_velocity_) / (max_acceleration_ * sign);

            t2_ = t1_;

            t3_ = t1_ + (- v_peak) / (max_acceleration_ * res_sign);

        }
        else {

            // 梯形曲线
            t1_ = (target_velocity_ - start_velocity_) / (max_acceleration_ * sign);
            t2_ = t1_ + (delta_position - need_to_accelerate_distance) / (target_velocity_);
            t3_ = t2_ - target_velocity_ / (max_acceleration_ * res_sign);

        }

        // 输出所有的计算结果
        std::cout << "TrajectoryGenerator set_target:" << std::endl;
        std::cout << "  start_position_: " << start_position_ << std::endl;
        std::cout << "  start_velocity_: " << start_velocity_ << std::endl;
        std::cout << "  target_position_: " << target_position_ << std::endl;
        std::cout << "  target_velocity_: " << target_velocity_ << std::endl;
        std::cout << "  max_acceleration_: " << max_acceleration_ << std::endl;
        std::cout << "  t1_: " << t1_ << std::endl;
        std::cout << "  t2_: " << t2_ << std::endl;
        std::cout << "  t3_: " << t3_ << std::endl;
        std::cout << "  delta_position: " << delta_position << std::endl;
        std::cout << "  need_to_accelerate_distance: " << need_to_accelerate_distance << std::endl;
        std::cout << "  sign: " << sign << std::endl;
        std::cout << "  active_: " << active_ << std::endl;
        std::cout << "  ----------------------------------------" << std::endl;

        return target_velocity_;

    }

    /**
     * @brief 更新当前位置和速度
     * @param position 函数给出的期望电机位置
     * @param velocity 函数给出的期望电机速度
     * 
     * @return true 处于活动状态
     * @return false 非活动状态/轨迹生成完成
     */
    bool update(double & position, double & velocity){
        if (!active_){
            position = 0;
            velocity = 0;
            return false;
        }

        rclcpp::Time now = rclcpp::Clock().now();
        double dt = (now - start_time_).seconds();

        if (dt >= t3_){
            // 轨迹完成
            active_ = false;
            position = target_position_;
            velocity = 0;
            // std::cout << "TrajectoryGenerator update: Trajectory finished." << std::endl;
            // std::cout << "  position: " << position << std::endl;
            // std::cout << "  velocity: " << velocity << std::endl;
            // std::cout << "  active_: " << active_ << std::endl;
            // std::cout << "  ----------------------------------------" << std::endl;
            return false;
        }
        else if (dt < t1_){
            // 加速阶段
            velocity = start_velocity_ + max_acceleration_ * sign * dt;
            position = start_position_ + start_velocity_ * dt + 0.5 * max_acceleration_ * sign * dt * dt;
        }
        else if (dt < t2_){
            // 匀速阶段
            velocity = start_velocity_ + max_acceleration_ * sign * t1_;
            position = start_position_ + start_velocity_ * t1_ + 0.5 * max_acceleration_ * sign * t1_ * t1_
                        + velocity * (dt - t1_);
        }
        else{
            // 减速阶段
            double t_dec = dt - t2_;
            double v_peak = start_velocity_ + max_acceleration_ * sign * t1_;
            velocity = v_peak + max_acceleration_ * res_sign * t_dec;
            position = start_position_ + start_velocity_ * t1_ + 0.5 * max_acceleration_ * sign * t1_ * t1_
                        + v_peak * (t2_ - t1_)
                        + (velocity + v_peak) * 0.5 * t_dec;
        }
        // std::cout << "TrajectoryGenerator update:" << std::endl;
        // std::cout << "  dt: " << dt << std::endl;
        // std::cout << "  position: " << position << std::endl;
        // std::cout << "  velocity: " << velocity << std::endl;
        // std::cout << "  active_: " << active_ << std::endl;
        // std::cout << "  ----------------------------------------" << std::endl;
        return true;
    }

    bool is_active() const {
        return active_;
    }

    void cancel(){
        active_ = false;
    }

private:
    // 所有速度的最大值
    double max_velocity_;
    // 最大加速度
    double max_acceleration_;

    // 目标位置
    double target_position_ = 0;
    // 期望速度
    double target_velocity_ = 0;
    // 当前速度
    double start_position_ = 0;
    // 当前位置
    double start_velocity_ = 0;

    // 是否处于活动状态
    bool active_ = false;

    // 开始时间
    rclcpp::Time start_time_;

    // 三个时间点

    // 加速结束时间长度
    double t1_;
    // 匀速结束时间长度
    double t2_;
    // 减速结束时间长度
    double t3_;

    // 加速方向
    double sign = 1;
    // 减速方向
    double res_sign = -1;


}; // class TrajectoryGenerator


} // namespace RM_hardware_interface

#endif // POSITION_SPEED_CONTROLLER__TRAJECTORY_GENERATOR_HPP_