#include "leonardo_teleop/pedal_publisher.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <libevdev/libevdev.h>
#include <poll.h>
#include <stdexcept>
#include <unistd.h>

namespace leonardo_teleop {

namespace {
rclcpp::Logger LOGGER = rclcpp::get_logger("PedalPublisher");
}

PedalPublisher::PedalPublisher() : Node("pedal_publisher") {
    // -------------------------------------------------------------------------
    // 1. 读取参数
    // -------------------------------------------------------------------------
    pedal_state_topic_ = declare_parameter<std::string>("pedal_state_topic", "/leonardo/pedal_state");
    pedal_device_path_ = declare_parameter<std::string>("device_path", "/dev/input/by-id/usb-PCsensor_FS20Pro-event-kbd");

    // -------------------------------------------------------------------------
    // 2. 【关键】启动前硬件预检查
    // 尝试以非阻塞方式打开设备，验证路径和权限是否正确
    // -------------------------------------------------------------------------
    int test_fd = ::open(pedal_device_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (test_fd < 0) {
        std::string err_msg = "无法打开踏板设备 [" + pedal_device_path_ + "]: " + std::strerror(errno);
        RCLCPP_FATAL(LOGGER, "%s", err_msg.c_str());
        RCLCPP_FATAL(LOGGER, "请检查: 1. 设备是否连接 2. udev 规则是否正确 3. 当前用户是否有权限");

        // 抛出异常，中断节点初始化
        throw std::runtime_error(err_msg);
    }

    // 预检查成功，立即关闭，真正的长连接将在 evdevLoop 线程中建立
    ::close(test_fd);

    // -------------------------------------------------------------------------
    // 3. 创建发布者
    // -------------------------------------------------------------------------
    pedal_state_pub_ = create_publisher<std_msgs::msg::Int32>(pedal_state_topic_, rclcpp::SystemDefaultsQoS());

    // -------------------------------------------------------------------------
    // 4. 启动事件监听线程
    // -------------------------------------------------------------------------
    running_ = true;
    evdev_thread_ = std::thread(&PedalPublisher::evdevLoop, this);

    RCLCPP_INFO(LOGGER, "PedalPublisher 已启动");
    RCLCPP_INFO(LOGGER, "  踏板状态话题: %s", pedal_state_topic_.c_str());
    RCLCPP_INFO(LOGGER, "  踏板设备    : %s", pedal_device_path_.c_str());
}

PedalPublisher::~PedalPublisher() {
    // 通知线程退出
    running_ = false;

    // 等待线程结束，避免资源泄露
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
        RCLCPP_ERROR(LOGGER, "线程中无法打开踏板设备 [%s]: %s", pedal_device_path_.c_str(), std::strerror(errno));
        return;
    }

    // 初始化 libevdev
    struct libevdev *dev = nullptr;
    int rc = libevdev_new_from_fd(fd, &dev);
    if (rc < 0) {
        RCLCPP_ERROR(LOGGER, "libevdev 初始化失败: %s", std::strerror(-rc));
        ::close(fd);
        return;
    }

    RCLCPP_INFO(LOGGER, "踏板设备已连接: %s", libevdev_get_name(dev));

    // 事件循环
    while (running_.load(std::memory_order_relaxed) && rclcpp::ok()) {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        // poll 超时设为 100ms，以便定期检查 running_ 标志
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
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);

        if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
            // 只处理按键按下事件 (value=1)
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
                    break; // 忽略其他键
                }

                if (new_state != -1) {
                    std_msgs::msg::Int32 msg;
                    msg.data = new_state;
                    pedal_state_pub_->publish(msg);
                    RCLCPP_INFO(LOGGER, "发布踏板状态: %d", new_state);
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

    // 清理资源
    libevdev_free(dev);
    ::close(fd);
    RCLCPP_INFO(LOGGER, "踏板事件监听线程已退出");
}

} // namespace leonardo_teleop

// -----------------------------------------------------------------------------
// Main 入口
// -----------------------------------------------------------------------------
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        // 如果构造函数中设备打开失败，这里会抛出 std::runtime_error
        auto node = std::make_shared<leonardo_teleop::PedalPublisher>();
        rclcpp::spin(node);
    } catch (const std::runtime_error &e) {
        // 捕获致命错误，打印日志并退出
        RCLCPP_FATAL(rclcpp::get_logger("pedal_publisher_main"), "节点启动失败: %s", e.what());
        rclcpp::shutdown();
        return 1; // 返回错误码
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("pedal_publisher_main"), "未知异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}