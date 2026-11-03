#include "leonardo_human_interfaces/foot_pedal/foot_pedal_node.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace leonardo_human_interfaces {

namespace {
static const rclcpp::Logger LOGGER = rclcpp::get_logger("foot_pedal_node");
}

FootPedalNode::FootPedalNode(const rclcpp::NodeOptions &options) : rclcpp::Node("foot_pedal_node", "/leonardo", options) {
    RCLCPP_INFO(LOGGER, "脚踏板节点已构造");

    declare_parameter<std::string>("pedal_key_event_topic", "/leonardo/foot_pedal/pedal_key_event");
    declare_parameter<std::string>("pedal_device_path", "/dev/input/by-id/usb-PCsensor_FS20Pro-event-kbd");

    pedal_key_event_topic_ = get_parameter("pedal_key_event_topic").as_string();
    pedal_device_path_ = get_parameter("pedal_device_path").as_string();

    pedal_key_event_pub_ = create_publisher<std_msgs::msg::Int32>(pedal_key_event_topic_, rclcpp::SystemDefaultsQoS());

    fd_ = ::open(pedal_device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        RCLCPP_ERROR(LOGGER, "脚踏板设备连接失败 [%s]: %s", pedal_device_path_.c_str(), std::strerror(errno));
        rclcpp::shutdown();
        std::exit(1);
        return;
    }

    if (::libevdev_new_from_fd(fd_, &dev_) < 0) {
        RCLCPP_ERROR(LOGGER, "libevdev 初始化失败");
        ::close(fd_);
        fd_ = -1;
        rclcpp::shutdown();
        std::exit(1);
        return;
    }

    RCLCPP_INFO(LOGGER, "脚踏板设备连接成功: %s", ::libevdev_get_name(dev_));

    running_ = true;
    evdev_thread_ = std::thread(&FootPedalNode::evdevLoop, this);
}

FootPedalNode::~FootPedalNode() {
    running_ = false;
    if (evdev_thread_.joinable()) {
        evdev_thread_.join();
    }
    if (dev_) {
        ::libevdev_free(dev_);
        dev_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    RCLCPP_INFO(LOGGER, "脚踏板节点已析构");
}

void FootPedalNode::evdevLoop() {
    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        int poll_rc = ::poll(&pfd, 1, 100);

        if (poll_rc < 0) {
            if (errno == EINTR)
                continue;
            RCLCPP_ERROR(LOGGER, "poll 错误: %s", std::strerror(errno));
            break;
        }

        if (poll_rc == 0) {
            continue;
        }

        struct input_event ev{};

        int rc = ::libevdev_next_event(dev_, LIBEVDEV_READ_FLAG_NORMAL, &ev);

        if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            if (ev.type == EV_KEY && ev.value == 1) {
                int new_key = -1;

                switch (ev.code) {
                case KEY_CODE_1:
                    new_key = KEY1;
                    break;
                case KEY_CODE_2:
                    new_key = KEY2;
                    break;
                case KEY_CODE_3:
                    new_key = KEY3;
                    break;
                case KEY_CODE_4:
                    new_key = KEY4;
                    break;
                case KEY_CODE_5:
                    new_key = KEY5;
                    break;
                case KEY_CODE_6:
                    new_key = KEY6;
                    break;
                default:
                    break;
                }

                if (new_key != -1) {
                    std_msgs::msg::Int32 msg;
                    msg.data = new_key;
                    pedal_key_event_pub_->publish(msg);
                    RCLCPP_INFO(LOGGER, "发布踏板状态: %d", new_key);
                }
            }
        } else if (rc == -EAGAIN) {
            continue;
        } else if (rc == LIBEVDEV_READ_STATUS_SYNC) {
            continue;
        } else {
            RCLCPP_ERROR(LOGGER, "libevdev_next_event 错误: %s", std::strerror(-rc));
            break;
        }
    }
}

} // namespace leonardo_human_interfaces

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<leonardo_human_interfaces::FootPedalNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}