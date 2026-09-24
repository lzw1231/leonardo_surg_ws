#include "leonardo_teleop/pedal_publisher.hpp"

#include <libevdev/libevdev.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace leonardo_teleop {
PedalPublisher::PedalPublisher() : Node("pedal_publisher") {
    // -------------------------------------------------------------------------
    // 读取参数
    // -------------------------------------------------------------------------
    pedal_state_topic_ = declare_parameter<std::string>("pedal_state_topic", "/leonardo/pedal_state");
    pedal_device_path_ =
        declare_parameter<std::string>("device_path", "/dev/input/by-id/usb-PCsensor_FS20Pro-event-kbd");

    // -------------------------------------------------------------------------
    // 发布者
    // -------------------------------------------------------------------------
    pedal_state_pub_ = create_publisher<std_msgs::msg::Int32>(pedal_state_topic_, rclcpp::SystemDefaultsQoS());

    // -------------------------------------------------------------------------
    // 启动 evdev 线程
    // -------------------------------------------------------------------------
    evdev_thread_ = std::thread(&PedalPublisher::evdevLoop, this);

    RCLCPP_INFO(get_logger(), "PedalPublisher 已启动");
    RCLCPP_INFO(get_logger(), "  踏板状态话题: %s", pedal_state_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  踏板设备    : %s", pedal_device_path_.c_str());
}

PedalPublisher::~PedalPublisher() {
    running_ = false;
    if (evdev_thread_.joinable()) {
        evdev_thread_.join();
    }
}

// -----------------------------------------------------------------------------
// evdev 事件循环（独立线程）
// -----------------------------------------------------------------------------
void PedalPublisher::evdevLoop() {
    // 打开设备（非阻塞，配合 poll）
    int fd = ::open(pedal_device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        RCLCPP_ERROR(get_logger(), "无法打开踏板设备 [%s]: %s", pedal_device_path_.c_str(), std::strerror(errno));
        return;
    }

    // 初始化 libevdev
    struct libevdev *dev = nullptr;
    int rc = libevdev_new_from_fd(fd, &dev);
    if (rc < 0) {
        RCLCPP_ERROR(get_logger(), "libevdev 初始化失败: %s", std::strerror(-rc));
        ::close(fd);
        return;
    }

    RCLCPP_INFO(get_logger(), "踏板已打开: %s", libevdev_get_name(dev));

    // 事件循环
    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        int poll_rc = ::poll(&pfd, 1, 100);
        if (poll_rc < 0) {
            if (errno == EINTR)
                continue;
            RCLCPP_ERROR(get_logger(), "poll 错误: %s", std::strerror(errno));
            break;
        }
        if (poll_rc == 0) {
            // 超时，回到循环顶部检查 running_
            continue;
        }

        struct input_event ev{};
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);

        if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            // 只处理按下事件
            if (ev.type == EV_KEY && ev.value == 1) {
                int new_state = -1;

                switch (ev.code) {
                case KEY_CODE_1:
                    new_state = STATE_1;
                    break;
                case KEY_CODE_2:
                    new_state = STATE_2;
                    break;
                case KEY_CODE_3:
                    new_state = STATE_3;
                    break;
                case KEY_CODE_4:
                    new_state = STATE_4;
                    break;
                case KEY_CODE_5:
                    new_state = STATE_5;
                    break;
                case KEY_CODE_6:
                    new_state = STATE_6;
                    break;

                default:
                    break; // 其他键忽略
                }

                if (new_state != -1) {
                    std_msgs::msg::Int32 msg;
                    msg.data = new_state;
                    pedal_state_pub_->publish(msg);

                    RCLCPP_INFO(get_logger(), "踏板状态 -> %d (published)", new_state);
                }
            }
        } else if (rc == -EAGAIN) {
            // 非阻塞模式下暂时没数据
            continue;
        } else if (rc == LIBEVDEV_READ_STATUS_SYNC) {
            // 同步事件（设备断开重连）
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
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<leonardo_teleop::PedalPublisher>());
    rclcpp::shutdown();
    return 0;
}