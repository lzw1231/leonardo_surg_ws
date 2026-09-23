#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace leonardo_teleop {
/**
 * 踏板状态发布者
 *
 * 职责：
 *   - 独立线程读 PCsensor FS20Pro 踏板的键盘事件（通过 libevdev）
 *   - 把踏板状态码发布到话题（Int32：1 ～ 6）
 */
class PedalPublisher : public rclcpp::Node {
  public:
    PedalPublisher();
    ~PedalPublisher() override;

  private:
    // 踏板状态码（仅编号，无业务语义）
    enum PedalState : int {
        STATE_1 = 1, // 踏板 1 按下
        STATE_2 = 2, // 踏板 2 按下
        STATE_3 = 3, // 踏板 3 按下
        STATE_4 = 4, // 踏板 4 按下
        STATE_5 = 5, // 踏板 5 按下
        STATE_6 = 6, // 踏板 6 按下
    };

    // 键码（evdev 常量值）
    static constexpr int KEY_CODE_1 = 2; // KEY_1
    static constexpr int KEY_CODE_2 = 3; // KEY_2
    static constexpr int KEY_CODE_3 = 4; // KEY_3
    static constexpr int KEY_CODE_4 = 5; // KEY_4
    static constexpr int KEY_CODE_5 = 6; // KEY_5
    static constexpr int KEY_CODE_6 = 7; // KEY_6

    // evdev 事件循环（独立线程）
    void evdevLoop();

    // 发布者：踏板状态
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pedal_state_pub_;

    // 参数
    std::string pedal_state_topic_;
    std::string pedal_device_path_;

    // evdev 线程控制
    std::atomic<bool> running_{true};
    std::thread evdev_thread_;
};
} // namespace leonardo_teleop