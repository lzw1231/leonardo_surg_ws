#pragma once

#include <atomic>
#include <string>
#include <thread>

// ROS 2 依赖
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/int32.hpp>

extern "C" {
#include <libevdev/libevdev.h>
}

namespace leonardo_human_interfaces {

namespace {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
}

class FootPedalNode : public rclcpp_lifecycle::LifecycleNode {
  public:
    explicit FootPedalNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~FootPedalNode() override = default;

    // 状态迁移回调
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

  private:
    enum PedalState : int {
        KEY1 = 1,
        KEY2 = 2,
        KEY3 = 3,
        KEY4 = 4,
        KEY5 = 5,
        KEY6 = 6,
    };

    static constexpr int KEY_CODE_1 = 2; // KEY_1
    static constexpr int KEY_CODE_2 = 3; // KEY_2
    static constexpr int KEY_CODE_3 = 4; // KEY_3
    static constexpr int KEY_CODE_4 = 5; // KEY_4
    static constexpr int KEY_CODE_5 = 6; // KEY_5
    static constexpr int KEY_CODE_6 = 7; // KEY_6

    void evdevLoop();

    // 发布者：脚踏板按键事件
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>::SharedPtr pedal_key_event_pub_;

    // 参数
    std::string pedal_key_event_topic_;
    std::string pedal_device_path_;

    // 设备句柄
    int fd_{-1};

    // 【关键修改】：使用 ::libevdev 明确指向全局命名空间中的 libevdev 结构体
    // 这样就不会和当前命名空间可能存在的同名符号冲突
    ::libevdev *dev_{nullptr};

    // 线程控制
    std::atomic<bool> running_{false};
    std::thread evdev_thread_;
};

} // namespace leonardo_human_interfaces
