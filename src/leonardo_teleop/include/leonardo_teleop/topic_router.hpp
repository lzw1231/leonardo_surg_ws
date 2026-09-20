#pragma once

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace leonardo_teleop{
    /**
     * 话题路由器
     *
     * 职责：
     *   - 独立线程读 PCsensor FS20Pro 踏板的键盘事件（通过 libevdev）
     *   - 订阅左右主手的 RPY 话题
     *   - 根据踏板状态，把 RPY 转发到两个下游话题（pedal_a / pedal_b）
     *
     * 踏板语义：
     *   踏板 1（KEY_1, code=2）→ 正常：左→A，右→B
     *   踏板 2（KEY_2, code=3）→ 空闲：不转发
     *   踏板 3（KEY_3, code=4）→ 交换：左→B，右→A
     *   初始状态：空闲
     *   只处理按下事件，忽略松开事件
     */
    class TopicRouter : public rclcpp::Node {
    public:
        TopicRouter();
        ~TopicRouter() override;

    private:
        // 踏板状态
        enum PedalState : int {
            STATE_1 = 1, // 正常：左→A，右→B
            STATE_2 = 2, // 空闲：不转发
            STATE_3 = 3, // 交换：左→B，右→A
        };

        // 键码（evdev 常量值）
        static constexpr int KEY_CODE_1 = 2; // KEY_1
        static constexpr int KEY_CODE_2 = 3; // KEY_2
        static constexpr int KEY_CODE_3 = 4; // KEY_3

        // ROS 订阅回调
        void onLeftRpy(const example_interfaces::msg::Float64MultiArray::SharedPtr msg);
        void onRightRpy(const example_interfaces::msg::Float64MultiArray::SharedPtr msg);

        // evdev 事件循环（独立线程）
        void evdevLoop();

        // 订阅者
        rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr left_sub_;
        rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr right_sub_;

        // 发布者
        rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr pedal_a_pub_;
        rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr pedal_b_pub_;

        // 参数
        std::string left_topic_;
        std::string right_topic_;
        std::string pedal_a_topic_;
        std::string pedal_b_topic_;
        std::string device_path_;

        // 踏板状态，多线程读写，用 atomic
        std::atomic<int> pedal_state_{STATE_2};

        // evdev 线程控制
        std::atomic<bool> running_{true};
        std::thread evdev_thread_;
    };
} // namespace leonardo_teleop
