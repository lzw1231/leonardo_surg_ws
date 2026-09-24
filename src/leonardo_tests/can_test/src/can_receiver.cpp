#include <atomic>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

class CanReceiver : public rclcpp::Node {
  public:
    CanReceiver() : Node("can_receiver"), running_(true) {
        // 参数：要监听的 CAN 接口，默认 can0 和 can1
        this->declare_parameter<std::vector<std::string>>("interfaces", {"can0", "can1"});
        interfaces_ = this->get_parameter("interfaces").as_string_array();

        if (interfaces_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未指定任何 CAN 接口");
            rclcpp::shutdown();
            return;
        }

        // 打开每个接口
        for (const auto &iface : interfaces_) {
            int sock = open_can(iface);
            if (sock < 0) {
                RCLCPP_ERROR(this->get_logger(), "打开 %s 失败", iface.c_str());
                rclcpp::shutdown();
                return;
            }
            sockets_.push_back(sock);
            RCLCPP_INFO(this->get_logger(), "已打开 %s", iface.c_str());
        }

        // 启动接收线程
        recv_thread_ = std::thread(&CanReceiver::receive_loop, this);
    }

    ~CanReceiver() {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        for (int sock : sockets_) {
            if (sock >= 0)
                close(sock);
        }
    }

  private:
    int open_can(const std::string &ifname) {
        int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "socket() 失败: %s", strerror(errno));
            return -1;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "ioctl SIOCGIFINDEX 失败: %s", strerror(errno));
            close(sock);
            return -1;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "bind 失败: %s", strerror(errno));
            close(sock);
            return -1;
        }
        return sock;
    }

    void receive_loop() {
        std::vector<struct pollfd> fds(sockets_.size());
        for (size_t i = 0; i < sockets_.size(); ++i) {
            fds[i].fd = sockets_[i];
            fds[i].events = POLLIN;
        }

        while (running_ && rclcpp::ok()) {
            int ret = poll(fds.data(), fds.size(), 100); // 100ms 超时
            if (ret < 0) {
                if (errno == EINTR)
                    continue;
                RCLCPP_ERROR(this->get_logger(), "poll 错误: %s", strerror(errno));
                break;
            }
            if (ret == 0)
                continue; // 超时

            for (size_t i = 0; i < fds.size(); ++i) {
                if (fds[i].revents & POLLIN) {
                    struct can_frame frame;
                    int nbytes = read(fds[i].fd, &frame, sizeof(frame));
                    if (nbytes == sizeof(frame)) {
                        print_frame(interfaces_[i], frame);
                    } else if (nbytes < 0) {
                        RCLCPP_ERROR(this->get_logger(), "读取 %s 失败: %s", interfaces_[i].c_str(), strerror(errno));
                    }
                }
            }
        }
    }

    void print_frame(const std::string &iface, const struct can_frame &frame) {
        // 格式化数据部分
        char data_str[64] = {0};
        int offset = 0;
        for (int i = 0; i < frame.can_dlc && i < 8; ++i) {
            offset += snprintf(data_str + offset, sizeof(data_str) - offset, "%02X ", frame.data[i]);
        }

        // 判断标准帧/扩展帧
        if (frame.can_id & CAN_EFF_FLAG) {
            uint32_t id = frame.can_id & CAN_EFF_MASK;
            RCLCPP_INFO(this->get_logger(), "[%s] ID=0x%08X DLC=%d DATA=%s", iface.c_str(), id, frame.can_dlc,
                        data_str);
        } else {
            uint32_t id = frame.can_id & CAN_SFF_MASK;
            RCLCPP_INFO(this->get_logger(), "[%s] ID=0x%03X DLC=%d DATA=%s", iface.c_str(), id, frame.can_dlc,
                        data_str);
        }
    }

    std::vector<std::string> interfaces_;
    std::vector<int> sockets_;
    std::thread recv_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}