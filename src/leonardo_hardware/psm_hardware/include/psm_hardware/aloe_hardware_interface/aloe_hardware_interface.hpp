#pragma once

#include "psm_hardware/common/can_protocol.hpp"
#include "psm_hardware/common/can_socket.hpp"
#include "psm_hardware/common/visibility_control.hpp"
#include <cstdint>
#include <hardware_interface/system_interface.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace psm_hardware_interface {

/**
 * @brief 主端——智能边缘计算机
 *
 * PSM（患者侧机械臂）系自主研制的从端机械臂。  aloe_芦荟版
 * 本类绑定 Linux SocketCAN 接口， can0控制 1、2号从臂；can1控制 3号从臂 、ecm 内窥镜
 */
class AloeHardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AloeHardwareInterface);

    PSM_HARDWARE_PUBLIC
    ~AloeHardwareInterface() override = default;

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
    CanSocket can0_, can1_;

    // 每个关节的状态与命令成对存放；默认 NaN 表示"未读到 / 未下发"
    struct Joint {
        double state{std::numeric_limits<double>::quiet_NaN()};
        double command{std::numeric_limits<double>::quiet_NaN()};
    };

    using JointMap = std::unordered_map<std::string, Joint *>;

    // 建立关节名 → Joint* 映射，并校验 URDF 中每个关节名都能查到
    bool build_joint_map();

    // ---------------- 1 号从臂（s1，CAN ID 0x441）----------------
    // 末端直线电机 + 公转关节 + 6虚拟电机
    Joint s1_j01_base, s1_j03_oc, s1_j04_oc, s1_j07_oc, s1_j08_oc, s1_j11_oc, s1_j12_oc, s1_j13_oc, s1_j14_oc, s1_j17_18, s1_j20_21,
        s1_joint_motor_01, s1_joint_motor_02, s1_joint_motor_03, s1_joint_motor_04, s1_joint_motor_05, s1_joint_motor_06;

    // ---------------- 2 号从臂（s2，CAN ID 0x442）----------------
    // 末端直线电机 + 公转关节 + 6虚拟电机
    Joint s2_j01_base, s2_j03_oc, s2_j04_oc, s2_j07_oc, s2_j08_oc, s2_j11_oc, s2_j12_oc, s2_j13_oc, s2_j14_oc, s2_j17_18, s2_j20_21,
        s2_joint_motor_01, s2_joint_motor_02, s2_joint_motor_03, s2_joint_motor_04, s2_joint_motor_05, s2_joint_motor_06;

    // ---------------- 3 号从臂（s3，CAN ID 0x443）----------------
    // 末端直线电机 + 公转关节 + 6虚拟电机
    Joint s3_j01_base, s3_j03_oc, s3_j04_oc, s3_j07_oc, s3_j08_oc, s3_j11_oc, s3_j12_oc, s3_j13_oc, s3_j14_oc, s3_j17_18, s3_j20_21,
        s3_joint_motor_01, s3_joint_motor_02, s3_joint_motor_03, s3_joint_motor_04, s3_joint_motor_05, s3_joint_motor_06;

    // 关节名 → Joint* 映射表，on_init 建立，export 使用
    JointMap joint_map_;

    // ---------------- CAN 运行参数 ----------------
    std::string can0_interface_, can1_interface_;
    uint32_t snake_1_id_{0}, snake_2_id_{0}, snake_3_id_{0};
    int max_current_{0}, speed_level_{0};
};

} // namespace psm_hardware_interface