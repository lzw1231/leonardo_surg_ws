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
 * @brief PSM can1 硬件接口
 *
 * PSM（患者侧机械臂）系自主研制的从端机械臂。
 * 本类绑定 Linux SocketCAN 接口 can1，控制 3 号从臂。
 */
class IntelligentCan1HardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(IntelligentCan1HardwareInterface);

    PSM_HARDWARE_PUBLIC
    ~IntelligentCan1HardwareInterface() override = default;

    // 生命周期回调
    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

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
    struct Joint {
        double state{0.0};
        double command{0.0};
    };

    // ---------------- 3 号从臂（s3，CAN ID 0x443）----------------
    Joint s3_j01_base; // 末端的直线电机
    Joint s3_j03_oc;
    Joint s3_j04_oc;
    Joint s3_j07_oc;
    Joint s3_j08_oc;
    Joint s3_j11_oc;
    Joint s3_j12_oc;
    Joint s3_j13_oc;
    Joint s3_j14_oc;
    Joint s3_j17_18;
    Joint s3_j20_21;

    // 虚拟电机
    Joint s3_joint_motor_01;
    Joint s3_joint_motor_02;
    Joint s3_joint_motor_03;
    Joint s3_joint_motor_04;
    Joint s3_joint_motor_05;
    Joint s3_joint_motor_06;

    // ---------------- CAN 运行参数 ----------------
    std::string can_interface_;
    uint32_t slave_3_id_{0};
    int max_current_{0};
    int speed_level_{0};
};

} // namespace psm_hardware_interface