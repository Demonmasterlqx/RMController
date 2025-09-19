// pid_controller.hpp

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <cstdint> // For uint64_t, uint32_t
#include <cmath>   // For std::fabs
#include <chrono>  // For std::chrono time measurement
#include <atomic>  // For atomic parameters

/**
 ******************************************************************************
 * @file	 pid_controller.hpp
 * @author  Wang Hongxi (original C code author), neozng (C code modifier), Gemini (C++ converter)
 * @version V1.0.0
 * @date    2023/10/26 (C++ version date)
 * @brief   C++ class for PID controller with various optimization features.
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
enum PID_Improvement_e : uint8_t{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
    PID_Integral_Limit = 0b00000001,              // 0000 0001
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000 (Not implemented in original C)
    PID_OutputFilter = 0b00010000,                // 0001 0000
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
    PID_DerivativeFilter = 0b01000000,            // 0100 0000
    PID_ErrorHandle = 0b10000000,                 // 1000 0000
};

/* PID 报错类型枚举*/
enum ErrorType_e : uint8_t{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
};

/* PID错误标志结构体 */
struct PIDErrorHandler{
    uint64_t error_count;
    ErrorType_e error_type;

    PIDErrorHandler() : error_count(0), error_type(PID_ERROR_NONE) {}
};

// PID 初始化配置结构体 (用于构造函数或初始化方法)
struct PID_Init_Config_s{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    /**
     * @brief 输出滤波器时间常数RC,单位秒
     * 
     */
    float Output_LPF_RC;
    /**
     * @brief 微分滤波器时间常数RC,单位秒
     * 
     */
    float Derivative_LPF_RC;
};


class PIDController{
public:
    // Constructor
    PIDController(const PID_Init_Config_s& config);

    // Reset all internal states (but keep configuration parameters)
    void reset();

    /**
     * @brief 计算PID输出
     *
     * @param measure 反馈值
     * @param ref     设定值
     * @return float  PID计算输出
     */
    float calculate(float measure, float ref);

    // Getters for current PID state (optional, but good for debugging)
    float get_p_out() const { return p_out_; }
    float get_i_out() const { return i_out_; }
    float get_d_out() const { return d_out_; }
    float get_output() const { return output_; }
    float get_error() const { return err_; }
    float get_dt() const { return dt_; }
    PIDErrorHandler get_error_handler() const { return error_handler_; }

    // Setters for PID gains (can be atomic if real-time tuning is needed)
    void set_kp(float kp) { Kp_.store(kp); }
    void set_ki(float ki) { Ki_.store(ki); }
    void set_kd(float kd) { Kd_.store(kd); }
    void set_max_out(float max_out) { max_out_ = max_out; }
    void set_dead_band(float dead_band) { dead_band_ = dead_band; }
    void set_integral_limit(float integral_limit) { integral_limit_ = integral_limit; }
    void set_coef_a(float coef_a) { coef_a_ = coef_a; }
    void set_coef_b(float coef_b) { coef_b_ = coef_b; }
    void set_output_lpf_rc(float rc) { output_lpf_rc_ = rc; }
    void set_derivative_lpf_rc(float rc) { derivative_lpf_rc_ = rc; }
    void set_improvement_flags(PID_Improvement_e improve_flags) { improve_ = improve_flags; }

private:
    //---------------------------------- init config block
    // config parameter
    std::atomic<float> Kp_;
    std::atomic<float> Ki_;
    std::atomic<float> Kd_;
    float max_out_;
    float dead_band_;

    // improve parameter
    PID_Improvement_e improve_;
    float integral_limit_;     // 积分限幅
    float coef_a_;             // 变速积分 For Changing Integral
    float coef_b_;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float output_lpf_rc_;     // 输出滤波器 RC = 1/omegac
    float derivative_lpf_rc_; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float measure_;
    float last_measure_;
    float err_;
    float last_err_;
    float last_i_term_; // Used in original for trapezoid integral, but the calculation `ITerm = Ki * ((Err + Last_Err) / 2) * dt;` already accounts for `Last_Err` so `Last_ITerm` might be redundant unless for specific purposes. Keeping for now to match original struct.

    float p_out_;
    float i_out_;
    float d_out_;
    float i_term_; // Current integral contribution before accumulation into i_out_

    float output_;
    float last_output_;
    float last_d_out_;

    float ref_;

    std::chrono::high_resolution_clock::time_point last_update_time_;
    float dt_; // Delta time in seconds

    PIDErrorHandler error_handler_;

    // --- Private helper methods for PID optimization stages ---
    // (These correspond to the static functions in the C implementation)

    // 梯形积分
    void f_trapezoid_integral();

    // 变速积分(误差小时积分作用更强)
    void f_changing_integration_rate();

    // 积分限幅
    void f_integral_limit();

    // 微分先行(仅使用反馈值而不计参考输入的微分)
    void f_derivative_on_measurement();

    // 微分滤波(采集微分时,滤除高频噪声)
    void f_derivative_filter();

    // 输出滤波
    void f_output_filter();

    // 输出限幅
    void f_output_limit();

    // 电机堵转检测
    void f_pid_error_handle();
};

#endif // PID_CONTROLLER_HPP