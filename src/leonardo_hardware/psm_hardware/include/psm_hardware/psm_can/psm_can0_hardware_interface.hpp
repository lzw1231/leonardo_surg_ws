#pragma once

#include "psm_hardware/common/visibility_control.hpp"

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <vector>

namespace psm_can_hardware {

/**
 * @brief PSM can0 硬件接口
 *
 * PSM（患者侧机械臂）系自主研制的从端机械臂。
 * 本类绑定 Linux SocketCAN 接口 can0，实现使能/去使能
 * 生命周期管理与 V6 协议数据收发。
 */
class PSMCan0HardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PSMCan0HardwareInterface);

    PSM_HARDWARE_PUBLIC
    ~PSMCan0HardwareInterface() override = default;

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
};

} // namespace psm_can_hardware