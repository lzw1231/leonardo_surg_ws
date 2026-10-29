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

FootPedalNode::FootPedalNode(const rclcpp::NodeOptions &options)
    : rclcpp_lifecycle::LifecycleNode("foot_pedal_node", "/leonardo", options) {
    RCLCPP_INFO(LOGGER, "FootPedalNode constructed.");
}

// ---------------------------------------------------------------------------
// on_configure：读参数 + 创建 lifecycle publisher
// ---------------------------------------------------------------------------
CallbackReturn FootPedalNode::on_configure(const rclcpp_lifecycle::State & /*state*/) {
    try {
        if (!has_parameter("pedal_key_event_topic")) {
            declare_parameter<std::string>("pedal_key_event_topic", "/leonardo/foot_pedal/pedal_key_event");
        }
        if (!has_parameter("pedal_device_path")) {
            declare_parameter<std::string>("pedal_device_path", "/dev/input/by-id/usb-PCsensor_FS20Pro-event-kbd");
        }
        pedal_key_event_topic_ = get_parameter("pedal_key_event_topic").as_string();
        pedal_device_path_ = get_parameter("pedal_device_path").as_string();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Parameter init failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }

    // 创建 Lifecycle Publisher
    pedal_key_event_pub_ = create_publisher<std_msgs::msg::Int32>(pedal_key_event_topic_, rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(LOGGER, "FootPedalNode configured. Topic=%s, Device=%s", pedal_key_event_topic_.c_str(), pedal_device_path_.c_str());
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate：启动 evdev 监听线程
// ---------------------------------------------------------------------------
CallbackReturn FootPedalNode::on_activate(const rclcpp_lifecycle::State &state) {
    // 1. 激活基类 Publisher
    auto ret = rclcpp_lifecycle::LifecycleNode::on_activate(state);
    if (ret != CallbackReturn::SUCCESS)
        return ret;

    // 2. 打开硬件设备
    fd_ = ::open(pedal_device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        RCLCPP_ERROR(LOGGER, "无法打开脚踏板设备 [%s]: %s", pedal_device_path_.c_str(), std::strerror(errno));
        return CallbackReturn::FAILURE;
    }

    // 3. 初始化 libevdev
    if (::libevdev_new_from_fd(fd_, &dev_) < 0) {
        RCLCPP_ERROR(LOGGER, "libevdev 初始化失败");
        ::close(fd_);
        fd_ = -1;
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(LOGGER, "设备已连接: %s", ::libevdev_get_name(dev_));

    // 4. 启动监听线程
    running_ = true;
    evdev_thread_ = std::thread(&FootPedalNode::evdevLoop, this);

    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate：停止线程并释放硬件资源
// ---------------------------------------------------------------------------
CallbackReturn FootPedalNode::on_deactivate(const rclcpp_lifecycle::State &state) {
    RCLCPP_INFO(LOGGER, "正在停用节点...");

    // 1. 停止线程并等待结束 (防止野指针)
    running_ = false;
    if (evdev_thread_.joinable()) {
        evdev_thread_.join();
    }

    // 2. 释放硬件资源
    if (dev_) {
        ::libevdev_free(dev_);
        dev_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }

    // 3. 停用基类 Publisher
    return rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
}

CallbackReturn FootPedalNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(LOGGER, "on_cleanup(): 清理配置资源");

    // 1. 重置 Publisher
    pedal_key_event_pub_.reset();

    // 2. 清除参数缓存
    pedal_key_event_topic_.clear();
    pedal_device_path_.clear();

    return CallbackReturn::SUCCESS;
}

CallbackReturn FootPedalNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_INFO(LOGGER, "on_shutdown(): 节点正在彻底关闭...");

    // 1. 确保线程已停止
    running_ = false;
    if (evdev_thread_.joinable()) {
        evdev_thread_.join();
    }

    // 2. 确保硬件资源已释放 【修正】：双重检查保护
    if (dev_) {
        ::libevdev_free(dev_);
        dev_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }

    // 3. 清理 ROS 资源
    pedal_key_event_pub_.reset();

    RCLCPP_INFO(LOGGER, "节点资源已全部释放，安全退出。");
    return CallbackReturn::SUCCESS;
}

CallbackReturn FootPedalNode::on_error(const rclcpp_lifecycle::State & /*state*/) {
    RCLCPP_ERROR(LOGGER, "on_error()");
    // 在错误状态下，通常也尝试清理资源以防泄漏
    running_ = false;
    if (evdev_thread_.joinable())
        evdev_thread_.join();
    if (dev_) {
        ::libevdev_free(dev_);
        dev_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    _exit(EXIT_FAILURE);
    return CallbackReturn::ERROR;
}

// ---------------------------------------------------------------------------
// evdev 事件循环
// ---------------------------------------------------------------------------
void FootPedalNode::evdevLoop() {
    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLIN;

        // poll 超时设为 100ms，以便定期检查 running_ 标志
        // 【修正】：使用 ::poll
        int poll_rc = ::poll(&pfd, 1, 100);

        if (poll_rc < 0) {
            if (errno == EINTR)
                continue; // 被信号中断，重试
            RCLCPP_ERROR(LOGGER, "poll 错误: %s", std::strerror(errno));
            break;
        }

        if (poll_rc == 0) {
            continue; // 超时，继续循环
        }

        // 有数据可读
        struct input_event ev{};
        // 注意：这里使用 dev_ 成员变量
        // 【修正】：使用 ::libevdev_next_event
        int rc = ::libevdev_next_event(dev_, LIBEVDEV_READ_FLAG_NORMAL, &ev);

        if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            // 只处理按键按下事件 (value=1)
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
                    break; // 忽略其他键
                }

                if (new_key != -1) {
                    std_msgs::msg::Int32 msg;
                    msg.data = new_key;
                    // 此时 Publisher 已通过基类 on_activate 激活
                    pedal_key_event_pub_->publish(msg);
                    RCLCPP_INFO(LOGGER, "发布踏板状态: %d", new_key); // 改为 DEBUG 减少刷屏
                }
            }
        } else if (rc == -EAGAIN) {
            // 非阻塞模式下暂时没数据，正常现象
            continue;
        } else if (rc == LIBEVDEV_READ_STATUS_SYNC) {
            // 同步事件，通常发生在设备断开重连后，忽略或重新初始化
            continue;
        } else {
            RCLCPP_ERROR(LOGGER, "libevdev_next_event 错误: %s", std::strerror(-rc));
            break;
        }
    }
}

} // namespace leonardo_human_interfaces

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<leonardo_human_interfaces::FootPedalNode>();

    rclcpp::spin(node->get_node_base_interface());

    rclcpp::shutdown();
    return 0;
}
