#ifndef CIRCLE_COUNTER___CIRCLE_COUNTER_HPP_
#define CIRCLE_COUNTER___CIRCLE_COUNTER_HPP_

#include <cmath>
#include <limits>
#include <exception>
#include <stdexcept>
#include <iostream>

namespace RM_hardware_interface {

/**
 * @brief CircleCounter
 *
 * 用于对周期性角度信号（例如编码器输出的 [-pi,pi] 或 [0,2pi) 角度）
 * 进行多圈展开（unwrap），并提供累计角度与已完成整圈数的查询。
 *
 * 用法:
 *  - 创建后调用 update(angle) 每次传入最新角度（单位：rad）。
 *  - get_unwrapped() 返回连续角度（从初始值开始累加的角度，单位 rad）。
 *  - get_revolutions() 返回以圈数为单位的连续值（double）。
 *  - get_full_rotations() 返回完整整圈数（向零截断的 int）。
 */

class CircleCounter{
public:
	/**
	 * @param maximum_rotational_range 最大允许的旋转范围（rad），默认 2*pi
	 */
	CircleCounter(double maximum_rotational_range = 2.0 * M_PI)
		: initialized_(false), prev_angle_(0.0), continuous_angle_(0.0),
			base_angle_(0.0), maximum_rotational_range_(maximum_rotational_range), half_range_(maximum_rotational_range / 2)
    {

        if(maximum_rotational_range_ < 0.0){
            throw std::invalid_argument("Maximum rotational range must be non-negative");
        }

    }

	/**
	 * @brief 重置计数器
	 * @param initial_angle 初始角度（rad），计数器被设置为从此角度开始
	 */
		void reset(double initial_angle = 0.0)
		{
			prev_angle_ = initial_angle;
			continuous_angle_ = 0.0;
			base_angle_ = initial_angle;
			initialized_ = true;
		}

	/**
	 * @brief 更新当前角度并返回展开后的连续角度
	 * @param angle 最新读取的角度（rad），可以是任意范围
	 * @return 展开后的连续角度（rad）
	 */
	double update(double angle){

		if (!initialized_) {
			reset(angle);
            std::cout << "call update before init, init automatically!. CircleCounter initialized with angle: " << angle << std::endl;
			return 0.0;
		}

		double delta = angle - prev_angle_;

        // 我们认为他套圈了
		if (std::abs(delta) > half_range_) {
            delta = std::copysign(maximum_rotational_range_ - std::abs(delta), -delta);
        }

		continuous_angle_ += delta;
		prev_angle_ = angle;
		return continuous_angle_;
	}

	/**
	 * @brief 获取展开后的连续角度（rad）
	 */
	double get_unwrapped() const { return continuous_angle_; }

    /**
     * @brief 获取基准角
     */
    double get_base_angle() const { return base_angle_; }

	private:

	/**
	 * @brief 获取连续圈数（double，可包含小数）
	 */
	double get_revolutions() const { return continuous_angle_ / (2.0 * M_PI); }

	/**
	 * @brief 获取整圈数（int，向零截断）
	 */
	int get_full_rotations() const { return static_cast<int>(continuous_angle_ / (2.0 * M_PI)); }

	/**
	 * @brief 是否已经初始化（至少调用过一次 update 或 reset）
	 */
	bool initialized() const { return initialized_; }

private:
	bool initialized_;
	double prev_angle_;        // 上一次原始读取角度（rad），保持原始范围
	double continuous_angle_;  // 展开后的连续角度（rad）
	double base_angle_;        // 标零/基准角（rad）
	double maximum_rotational_range_; // 最大允许旋转范围（rad）
	double half_range_;        // 缓存的 half range
};

} // namespace differential_controller

#endif // CIRCLE_COUNTER___CIRCLE_COUNTER_HPP_
