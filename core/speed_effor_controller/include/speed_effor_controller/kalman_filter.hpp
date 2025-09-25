#include <iostream> // 用于标准输入输出
#include <vector>   // 仍可能用于存储历史数据，但核心不再使用
#include <cmath>    // 用于 std::sqrt, std::pow
#include <random>   // 用于 std::normal_distribution, std::mt19937
#include <iomanip>  // 用于 std::setprecision, std::fixed
#include <string>   // 用于 std::string

// 包含 Eigen 库的头文件
// Eigen 是一个纯头文件库，通常只需要 Dense 模块
#include <eigen3/Eigen/Dense>
namespace RM_hardware_interface{

// --- 卡尔曼滤波器类 ---
class KalmanFilter {
public:
    // 状态向量 x: [位置, 速度]
    Eigen::VectorXd x; 
    // 状态协方差矩阵 P: 表示状态估计的不确定性
    Eigen::MatrixXd P; 
    // 状态转移矩阵 F: 描述状态如何随时间步长 dt 演变
    Eigen::MatrixXd F; 
    // 测量矩阵 H: 将状态向量映射到测量值
    Eigen::MatrixXd H; 
    // 过程噪声协方差矩阵 Q: 反映模型的不确定性
    Eigen::MatrixXd Q; 
    // 测量噪声协方差矩阵 R: 反映传感器测量值的不确定性
    Eigen::MatrixXd R; 
    // 单位矩阵 I (用于更新 P)
    Eigen::MatrixXd I; 

    double dt; // 时间步长

    KalmanFilter(double dt_val, double initial_velocity, double process_variance, double measurement_variance) : dt(dt_val) {
        // 1. 初始状态向量 x: [位置, 速度]
        // 使用 Eigen::VectorXd，大小为 2
        x = Eigen::VectorXd(2);
        x << 0.0,            // 初始位置设为0
             initial_velocity; // 初始速度为估计值

        // 2. 初始状态协方差矩阵 P: 大的不确定性表示初始猜测不准确
        // 使用 Eigen::MatrixXd，大小为 2x2
        P = Eigen::MatrixXd(2, 2);
        P << 100.0, 0.0,
             0.0, 100.0;

        // 3. 状态转移矩阵 F: 匀速模型
        // 大小为 2x2
        F = Eigen::MatrixXd(2, 2);
        F << 1.0, dt,
             0.0, 1.0;

        // 4. 测量矩阵 H: 我们直接测量速度
        // 大小为 1x2
        H = Eigen::MatrixXd(1, 2);
        H << 0.0, 1.0;

        // 5. 过程噪声协方差矩阵 Q: 反映模型不确定性，乘以过程噪声方差进行缩放
        // 这里的 Q 是基于常加速度模型的离散化，再乘以一个调节因子
        Q = Eigen::MatrixXd(2, 2);
        Q << 0.25 * std::pow(dt, 4), 0.5 * std::pow(dt, 3),
             0.5 * std::pow(dt, 3),  std::pow(dt, 2);
        Q *= process_variance; // 使用 *= 进行标量乘法

        // 6. 测量噪声协方差矩阵 R: 反映传感器噪声
        // 大小为 1x1
        R = Eigen::MatrixXd(1, 1);
        R << measurement_variance;

        // 7. 单位矩阵 I (与状态向量维度相同)
        I = Eigen::MatrixXd::Identity(x.size(), x.size()); // Eigen::MatrixXd::Identity 方便创建单位矩阵
    }

    // 预测步骤
    double predict() {
        // 预测状态: x_pred = F * x
        x = F * x;
        // 预测协方差: P_pred = F * P * F.transpose() + Q
        P = F * P * F.transpose() + Q;
        return x(1); // 返回预测的速度 (状态向量的第二个元素，Eigen 向量从 0 开始索引)
    }

    // 更新步骤
    double update(double measurement) {
        // 将标量测量值转换为 Eigen::VectorXd (1x1 矩阵)
        Eigen::VectorXd z = Eigen::VectorXd(1);
        z << measurement;

        // 测量残差 (或创新) y: y = z - H * x_pred
        Eigen::VectorXd y = z - (H * x);

        // 创新协方差 S: S = H * P_pred * H.transpose() + R
        Eigen::MatrixXd S = H * P * H.transpose() + R;

        // 卡尔曼增益 K: K = P_pred * H.transpose() * S.inverse()
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        // 更新状态估计: x = x_pred + K * y
        x = x + (K * y);

        // 更新误差协方差: P = (I - K * H) * P_pred
        P = (I - K * H) * P;
        
        return x(1); // 返回更新后的速度估计
    }
};

} // namespace RM_hardware_interface

// // --- 示例使用 ---
// int main() {
//     double dt = 0.1; // 时间步长，例如 0.1 秒
//     double true_velocity = 10.0; // 假设的真实速度，例如 10 m/s

//     // 初始化卡尔曼滤波器的参数
//     double initial_velocity_guess = 5.0; // 对初始速度的猜测
//     double process_noise_variance = 0.1; // 过程噪声方差，反映模型的不确定性 (根据经验或系统特性调整)
//     double measurement_noise_variance = 1.0; // 测量噪声方差，反映传感器数据的不确定性 (可从传感器规格或实验获得)

//     // 创建卡尔曼滤波器实例
//     KalmanFilter kf(dt, initial_velocity_guess, process_noise_variance, measurement_noise_variance);

//     // 用于模拟传感器噪声的随机数生成器
//     std::random_device rd; // 硬件熵源
//     std::mt19937 gen(rd()); // Mersenne Twister 伪随机数生成器
//     // 创建正态分布，均值为 0，标准差为测量噪声方差的平方根
//     std::normal_distribution<> d(0, std::sqrt(measurement_noise_variance)); 

//     int num_steps = 100; // 模拟的步数

//     // 设置输出精度
//     std::cout << std::fixed << std::setprecision(3);

//     // 打印表头
//     std::cout << "时间 (s) | 真实速度 | 测量速度 (含噪声) | 预测速度 | 估计速度" << std::endl;
//     std::cout << "---------|----------|-----------------|----------|----------" << std::endl;

//     for (int i = 0; i < num_steps; ++i) {
//         // 模拟一个带噪声的速度测量值
//         double noisy_measurement = true_velocity + d(gen);

//         // 卡尔曼滤波的预测和更新步骤
//         double predicted_velocity = kf.predict();
//         double estimated_velocity = kf.update(noisy_measurement);

//         // 打印当前步的结果
//         std::cout << std::setw(8) << i * dt << " | "
//                   << std::setw(8) << true_velocity << " | "
//                   << std::setw(17) << noisy_measurement << " | "
//                   << std::setw(10) << predicted_velocity << " | "
//                   << std::setw(8) << estimated_velocity << std::endl;
//     }

//     return 0;
// }