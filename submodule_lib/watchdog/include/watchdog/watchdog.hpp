
#ifndef WATCHDOG_HPP_
#define WATCHDOG_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace RM_hardware_interface{


class Watchdog{

public:

    Watchdog(double timeout) : watchdog_timeout_(timeout) {}

    // 是否触发
    bool is_triggered() {
        return ((rclcpp::Clock().now() - last_command_time_) > rclcpp::Duration::from_seconds(watchdog_timeout_)) && command_used_.load();
    }

    // 重置
    void reset() {
        last_command_time_ = rclcpp::Clock().now();
        command_used_.store(false);
    }

    void set_used() {
        command_used_.store(true);
    }


private:

    // 看门狗超时时间，单位秒
    double watchdog_timeout_;

    // 上次收到命令的时间
    rclcpp::Time last_command_time_;

    std::atomic<bool> command_used_{false};

};

};

#endif  // WATCHDOG_HPP_