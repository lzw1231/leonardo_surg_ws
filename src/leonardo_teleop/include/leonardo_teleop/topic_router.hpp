#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <atomic>
#include <string>
#include <thread>

namespace leonardo_teleop{
    /**
     * 踏板状态发布者
     *
     * 职责：
     *   - 独立线程读 PCsensor FS20Pro 踏板的键盘事件（通过 libevdev）
     *   - 把踏板状态发布到话题（Int32：1/2/3）
     *
     * 踏板语义：
     *   踏板 1（KEY_1, code=2）→ STATE_1：正常（左→A，右→B）
     *   踏板 2（KEY_2, code=3）→ STATE_2：空闲（不转发）
     *   踏板 3（KEY_3, code=4）→ STATE_3：交换（左→B，右→A）
     *   初始状态：空闲
     *   只处理按下事件，忽略松开事件
     *
     * 注意：本节点不再转发主手 RPY 数据。
     *       主手 RPY 数据已改由控制器链（引用接口）直接传递，
     *       本节点只负责产生“选路开关”信号。
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

        // evdev 事件循环（独立线程）
        void evdevLoop();

        // 发布者：踏板状态
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pedal_state_pub_;

        // 参数
        std::string pedal_state_topic_;
        std::string device_path_;

        // 踏板状态，多线程读写，用 atomic
        std::atomic<int> pedal_state_{STATE_2};

        // evdev 线程控制
        std::atomic<bool> running_{true};
        std::thread evdev_thread_;
    };
} // namespace leonardo_teleop
