#pragma once

#include "psm_hardware/common/visibility_control.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace psm_hardware_interface {

/**
 * @brief PSM can0 硬件接口
 *
 * PSM（患者侧机械臂）系自主研制的从端机械臂。
 * 本类绑定 Linux SocketCAN 接口 can0，控制 1、2 号从臂。
 */
class IntelligentCan0HardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IntelligentCan0HardwareInterface);

    PSM_HARDWARE_PUBLIC
    ~IntelligentCan0HardwareInterface() override = default;

    // 生命周期回调
    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // 接口导出
    PSM_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

    PSM_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

    // 数据读写
    PSM_HARDWARE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // 每个关节的状态与命令成对存放
    struct Joint {
        double state{0.0};
        double command{0.0};
    };

    // ---------------- 1 号从臂（s1，CAN ID 0x441）----------------
    Joint s1_j03_oc;
    Joint s1_j04_oc;
    Joint s1_j07_oc;
    Joint s1_j08_oc;
    Joint s1_j11_oc;
    Joint s1_j12_oc;
    Joint s1_j13_oc;
    Joint s1_j14_oc;
    Joint s1_j17_18;
    Joint s1_j20_21;

    // ---------------- 2 号从臂（s2，CAN ID 0x442）----------------
    Joint s2_j03_oc;
    Joint s2_j04_oc;
    Joint s2_j07_oc;
    Joint s2_j08_oc;
    Joint s2_j11_oc;
    Joint s2_j12_oc;
    Joint s2_j13_oc;
    Joint s2_j14_oc;
    Joint s2_j17_18;
    Joint s2_j20_21;

    // ---------------- CAN 运行参数 ----------------
    std::string can_interface_;
    uint32_t slave_1_id_{0};
    uint32_t slave_2_id_{0};
    int max_current_{0};
    int speed_level_{0};
};

} // namespace psm_hardware_interface