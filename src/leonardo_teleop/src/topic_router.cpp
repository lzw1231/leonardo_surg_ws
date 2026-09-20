#include "leonardo_teleop/topic_router.hpp"

#include <libevdev/libevdev.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace leonardo_teleop{
    TopicRouter::TopicRouter() : Node("topic_router") {
        // -------------------------------------------------------------------------
        // 读取参数（YAML 里配，launch 里加载 teleop_params.yaml）
        // -------------------------------------------------------------------------
        pedal_state_topic_ = declare_parameter<std::string>("pedal_state_topic", "/leonardo/pedal_state");
        device_path_ = declare_parameter<std::string>("device_path", "/dev/input/by-id/usb-PCsensor_FS20Pro-event-kbd");

        // -------------------------------------------------------------------------
        // QoS：和下游 ft3215_controller 保持一致
        // -------------------------------------------------------------------------
        const auto qos = rclcpp::SystemDefaultsQoS();

        // -------------------------------------------------------------------------
        // 发布者：踏板状态
        // -------------------------------------------------------------------------
        pedal_state_pub_ = create_publisher<std_msgs::msg::Int32>(pedal_state_topic_, qos);

        // -------------------------------------------------------------------------
        // 启动 evdev 线程
        // -------------------------------------------------------------------------
        evdev_thread_ = std::thread(&TopicRouter::evdevLoop, this);

        RCLCPP_INFO(get_logger(), "PedalStatePublisher 已启动");
        RCLCPP_INFO(get_logger(), "  踏板状态话题: %s", pedal_state_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  踏板设备    : %s", device_path_.c_str());
        RCLCPP_INFO(get_logger(), "  初始状态    : 空闲（STATE_2）");
    }

    TopicRouter::~TopicRouter() {
        running_ = false;
        if (evdev_thread_.joinable()) {
            evdev_thread_.join();
        }
    }

    // -----------------------------------------------------------------------------
    // evdev 事件循环（独立线程）
    // -----------------------------------------------------------------------------
    void TopicRouter::evdevLoop() {
        // 打开设备（非阻塞，配合 poll）
        int fd = ::open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            RCLCPP_ERROR(get_logger(), "无法打开踏板设备 [%s]: %s", device_path_.c_str(), std::strerror(errno));
            return;
        }

        // 初始化 libevdev
        struct libevdev* dev = nullptr;
        int rc = libevdev_new_from_fd(fd, &dev);
        if (rc < 0) {
            RCLCPP_ERROR(get_logger(), "libevdev 初始化失败: %s", std::strerror(-rc));
            ::close(fd);
            return;
        }

        RCLCPP_INFO(get_logger(), "踏板已打开: %s", libevdev_get_name(dev));

        // 事件循环
        while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
            // 用 poll 等事件，100ms 超时，便于退出
            struct pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLIN;

            int poll_rc = ::poll(&pfd, 1, 100);
            if (poll_rc < 0) {
                if (errno == EINTR) continue; // 被信号打断，继续
                RCLCPP_ERROR(get_logger(), "poll 错误: %s", std::strerror(errno));
                break;
            }
            if (poll_rc == 0) {
                // 超时，没事件，回到循环顶部检查 running_
                continue;
            }

            // 读事件
            struct input_event ev{};
            rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);

            if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
                // 只关心按键按下：EV_KEY + value==1
                if (ev.type == EV_KEY && ev.value == 1) {
                    int new_state = -1;
                    const char* state_name = nullptr;

                    switch (ev.code) {
                    case KEY_CODE_1: // 踏板1
                        new_state = STATE_1;
                        state_name = "正常（左→A，右→B）";
                        break;
                    case KEY_CODE_2: // 踏板2
                        new_state = STATE_2;
                        state_name = "空闲（不转发）";
                        break;
                    case KEY_CODE_3: // 踏板3
                        new_state = STATE_3;
                        state_name = "交换（左→B，右→A）";
                        break;
                    default:
                        break; // 其他键忽略
                    }

                    if (new_state != -1) {
                        pedal_state_.store(new_state, std::memory_order_relaxed);

                        // 发布踏板状态
                        std_msgs::msg::Int32 msg;
                        msg.data = new_state;
                        pedal_state_pub_->publish(msg);

                        RCLCPP_INFO(get_logger(), "踏板状态 -> %s (published %d)",
                                    state_name, new_state);
                    }
                }
            } else if (rc == -EAGAIN) {
                // 非阻塞模式下暂时没数据，正常，继续
                continue;
            } else if (rc == LIBEVDEV_READ_STATUS_SYNC) {
                // 同步事件（设备断开重连），继续
                continue;
            } else {
                RCLCPP_ERROR(get_logger(), "libevdev_next_event 错误: %s", std::strerror(-rc));
                break;
            }
        }

        libevdev_free(dev);
        ::close(fd);
        RCLCPP_INFO(get_logger(), "踏板事件循环已退出");
    }
} // namespace leonardo_teleop

// -----------------------------------------------------------------------------
// main
// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<leonardo_teleop::TopicRouter>());
    rclcpp::shutdown();
    return 0;
}
