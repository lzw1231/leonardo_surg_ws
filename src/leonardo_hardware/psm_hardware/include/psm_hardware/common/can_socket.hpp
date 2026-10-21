#pragma once

#include <linux/can.h>
#include <string>

namespace psm_hardware_interface {

class CanSocket {
  public:
    CanSocket() = default;
    ~CanSocket();

    CanSocket(const CanSocket &) = delete;
    CanSocket &operator=(const CanSocket &) = delete;

    CanSocket(CanSocket &&other) noexcept;
    CanSocket &operator=(CanSocket &&other) noexcept;

    /// 打开并绑定接口。失败抛 std::system_error（携带 errno）。
    void open(const std::string &ifname);
    void send(const can_frame &frame);

    void close();
    bool is_open() const { return handle_ >= 0; }
    int handle() const { return handle_; }
    const std::string &ifname() const { return ifname_; }

  private:
    int handle_ = -1;
    std::string ifname_;
};
} // namespace psm_hardware_interface