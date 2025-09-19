#include "pid_controller/pid_controller.hpp"
#include <cstring>
#include <algorithm>


PIDController::PIDController(const PID_Init_Config_s& config){

    // load parameters from config
    Kp_.store(config.Kp);
    Ki_.store(config.Ki);
    Kd_.store(config.Kd);
    max_out_ = config.MaxOut;
    dead_band_ = config.DeadBand;
    improve_ = config.Improve;
    integral_limit_ = config.IntegralLimit;
    coef_a_ = config.CoefA;
    coef_b_ = config.CoefB;
    output_lpf_rc_ = config.Output_LPF_RC;
    derivative_lpf_rc_ = config.Derivative_LPF_RC;

    reset();
}

/* ----------------------------下面是pid优化环节的实现---------------------------- */

// 梯形积分
void PIDController::f_trapezoid_integral(){
    // 计算梯形的面积,(上底+下底)*高/2
    i_term_ = Ki_.load() * ((err_ + last_err_) / 2.0f) * dt_;
}

// 变速积分(误差小时积分作用更强)
void PIDController::f_changing_integration_rate(){
    // 积分呈累积趋势
    if (err_ * i_out_ > 0) {
        if (std::fabs(err_) <= coef_b_)
            return; // Full integral
        if (std::fabs(err_) <= (coef_a_ + coef_b_))
            i_term_ *= (coef_a_ - std::fabs(err_) + coef_b_) / coef_a_;
        else // 最大阈值,不使用积分
            i_term_ = 0;
    }
}

void PIDController::f_integral_limit(){

    float temp_output = p_out_ + i_out_ + d_out_;
    float temp_i_out = i_out_ + i_term_;

    // Check if adding current ITerm would cause output to exceed MaxOut, and if integral is accumulating in the wrong direction
    if (std::fabs(temp_output) > max_out_)
    {
        if (err_ * i_out_ > 0) // 积分却还在累积 (err and i_out have same sign)
        {
            i_term_ = 0; // 当前积分项置零
        }
    }

    // Direct integral output clamping
    if (temp_i_out > integral_limit_){
        i_term_ = 0; // Prevent further integration
        i_out_ = integral_limit_; // Clamp the accumulated integral
    }
    if (temp_i_out < -integral_limit_){
        i_term_ = 0; // Prevent further integration
        i_out_ = -integral_limit_; // Clamp the accumulated integral
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
void PIDController::f_derivative_on_measurement(){
    d_out_ = Kd_.load() * (last_measure_ - measure_) / dt_;
}

// 微分滤波(采集微分时,滤除高频噪声)
void PIDController::f_derivative_filter(){
    if (derivative_lpf_rc_ > 0 && dt_ > 0) {
        d_out_ = d_out_ * dt_ / (derivative_lpf_rc_ + dt_) +
                 last_d_out_ * derivative_lpf_rc_ / (derivative_lpf_rc_ + dt_);
    }
}

// 输出滤波
void PIDController::f_output_filter(){
    if (output_lpf_rc_ > 0 && dt_ > 0) {
        output_ = output_ * dt_ / (output_lpf_rc_ + dt_) +
                  last_output_ * output_lpf_rc_ / (output_lpf_rc_ + dt_);
    }
}

// 输出限幅
void PIDController::f_output_limit(){
    if (output_ > max_out_){
        output_ = max_out_;
    }
    else if (output_ < -max_out_){
        output_ = -max_out_;
    }
}

// 电机堵转检测
void PIDController::f_pid_error_handle(){
    /*Motor Blocked Handle*/
    if (std::fabs(output_) < max_out_ * 0.001f || std::fabs(ref_) < 0.0001f){
        error_handler_.error_count = 0; // Reset if output is very small or ref is zero
        return;
    }

    if ((std::fabs(ref_ - measure_) / std::fabs(ref_)) > 0.95f){
        // Motor blocked counting if error is very large relative to reference
        error_handler_.error_count++;
    }
    else{
        error_handler_.error_count = 0;
    }

    if (error_handler_.error_count > 500){
        error_handler_.error_type = PID_MOTOR_BLOCKED_ERROR;
    }
    else{
        error_handler_.error_type = PID_ERROR_NONE; // Reset if not blocked
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief          PID计算
 * @param[in]      measure 测量值
 * @param[in]      ref     期望值
 * @retval         返回PID计算输出
 */
float PIDController::calculate(float measure, float ref){
    // Calculate dt
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration = current_time - last_update_time_;
    dt_ = duration.count(); // dt in seconds
    last_update_time_ = current_time;

    if (dt_ < 1e-6f) {
        return output_;
    }


    // 堵转检测
    if (improve_ & PID_ErrorHandle)
        f_pid_error_handle();

    // 保存上次的测量值和误差,计算当前error
    measure_ = measure;
    ref_ = ref;
    err_ = ref_ - measure_;

    // 如果在死区外,则计算PID
    if (std::fabs(err_) > dead_band_) {
        // 基本的pid计算,使用位置式
        p_out_ = Kp_.load() * err_;
        i_term_ = Ki_.load() * err_ * dt_; // Temporary integral component
        d_out_ = Kd_.load() * (err_ - last_err_) / dt_;

        // 梯形积分
        if (improve_ & PID_Trapezoid_Intergral)
            f_trapezoid_integral();
        // 变速积分
        if (improve_ & PID_ChangingIntegrationRate)
            f_changing_integration_rate();
        // 微分先行
        if (improve_ & PID_Derivative_On_Measurement)
            f_derivative_on_measurement();
        // 微分滤波器
        if (improve_ & PID_DerivativeFilter)
            f_derivative_filter();

        // 累加积分 (must be after trapezoid and changing rate, before integral limit for current cycle)
        i_out_ += i_term_;

        // 积分限幅 (after I_out_ has been updated, but before total output)
        if (improve_ & PID_Integral_Limit)
            f_integral_limit(); // This function modifies i_out_ directly or sets i_term_=0 for next cycle

        output_ = p_out_ + i_out_ + d_out_; // 计算输出

        // 输出滤波
        if (improve_ & PID_OutputFilter)
            f_output_filter();

        // 输出限幅
        f_output_limit();
    }
    else{
        output_ = 0;
        i_out_ = 0; // Clear accumulated integral as well
        i_term_ = 0;
        p_out_ = 0;
        d_out_ = 0;
    }

    // 保存当前数据,用于下次计算
    last_measure_ = measure_;
    last_output_ = output_;
    last_d_out_ = d_out_;
    last_err_ = err_;
    last_i_term_ = i_term_; // Not strictly used by default, but kept for consistency with original struct

    return output_;
}

void PIDController::reset(){
    // Reset all internal states
    measure_ = 0.0f;
    last_measure_ = 0.0f;
    err_ = 0.0f;
    last_err_ = 0.0f;
    last_i_term_ = 0.0f;

    p_out_ = 0.0f;
    i_out_ = 0.0f;
    d_out_ = 0.0f;
    i_term_ = 0.0f;

    output_ = 0.0f;
    last_output_ = 0.0f;
    last_d_out_ = 0.0f;

    ref_ = 0.0f;

    last_update_time_ = std::chrono::high_resolution_clock::now();
    dt_ = 0.0f;

    error_handler_ = PIDErrorHandler();
}