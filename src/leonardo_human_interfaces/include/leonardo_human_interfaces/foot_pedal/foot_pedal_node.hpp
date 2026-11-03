#pragma once

#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <thread>

extern "C" {
#include <libevdev/libevdev.h>
}

namespace leonardo_human_interfaces {

class FootPedalNode : public rclcpp::Node {
  public:
    explicit FootPedalNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~FootPedalNode() override;

  private:
    enum PedalState : int {
        KEY1 = 1,
        KEY2 = 2,
        KEY3 = 3,
        KEY4 = 4,
        KEY5 = 5,
        KEY6 = 6,
    };

    static constexpr int KEY_CODE_1 = 2;
    static constexpr int KEY_CODE_2 = 3;
    static constexpr int KEY_CODE_3 = 4;
    static constexpr int KEY_CODE_4 = 5;
    static constexpr int KEY_CODE_5 = 6;
    static constexpr int KEY_CODE_6 = 7;

    void evdevLoop();

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pedal_key_event_pub_;

    std::string pedal_key_event_topic_;
    std::string pedal_device_path_;

    int fd_{-1};
    ::libevdev *dev_{nullptr};

    std::atomic<bool> running_{false};
    std::thread evdev_thread_;
};

} // namespace leonardo_human_interfaces