#include "psm_hardware/common/can_socket.hpp"
#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <system_error>
#include <unistd.h>
#include <utility>

namespace psm_hardware_interface {

CanSocket::~CanSocket() { close(); }

CanSocket::CanSocket(CanSocket &&other) noexcept : handle_(std::exchange(other.handle_, -1)), ifname_(std::move(other.ifname_)) {}

CanSocket &CanSocket::operator=(CanSocket &&other) noexcept {
    if (this != &other) {
        close();
        handle_ = std::exchange(other.handle_, -1);
        ifname_ = std::move(other.ifname_);
    }
    return *this;
}

void CanSocket::open(const std::string &ifname) {
    close();

    int handle = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (handle < 0) {
        throw std::system_error(errno, std::generic_category(), "创建 CAN 套接字失败");
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (::ioctl(handle, SIOCGIFINDEX, &ifr) < 0) {
        int err = errno;
        ::close(handle);
        throw std::system_error(err, std::generic_category(), "获取 CAN 接口索引失败 [" + ifname + "]");
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(handle, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        int err = errno;
        ::close(handle);
        throw std::system_error(err, std::generic_category(), "绑定 CAN 接口失败 [" + ifname + "]");
    }

    handle_ = handle;
    ifname_ = ifname;
}

void CanSocket::send(const can_frame &frame) {
    ssize_t n = ::write(handle_, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
        throw std::system_error(errno, std::generic_category(), "CAN 发送失败");
    }
}

void CanSocket::close() {
    if (handle_ >= 0) {
        ::close(handle_);
        handle_ = -1;
    }
    ifname_.clear();
}

} // namespace psm_hardware_interface