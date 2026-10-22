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
 * @brief 主端——智能边缘计算机 (Aloe版)
 *
 * 优化点：
 * 1. 使用 ArmDefinition 结构体管理配置，避免硬编码字符串。
 * 2. 动态生成 Joint 对象和映射表，减少重复代码。
 * 3. 接口导出基于 Map 自动匹配，增强鲁棒性。
 */
class AloeHardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AloeHardwareInterface);

    PSM_HARDWARE_PUBLIC
    ~AloeHardwareInterface() override = default;

    // --- ROS2 Control 生命周期回调 ---
    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // --- 接口导出 ---
    PSM_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

    PSM_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

    // --- 数据读写 ---
    PSM_HARDWARE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    PSM_HARDWARE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // --- 内部数据结构 ---

    // 单个关节的数据容器
    struct JointData {
        double state{std::numeric_limits<double>::quiet_NaN()};
        double command{std::numeric_limits<double>::quiet_NaN()};
    };

    // 机械臂定义结构体：将 CAN ID、前缀和关节后缀绑定在一起
    struct ArmDefinition {
        std::string prefix;                      // 例如 "s1"
        uint32_t can_id;                         // 例如 0x441
        std::vector<std::string> joint_suffixes; // 该臂包含的所有关节后缀

        // 构造函数方便初始化
        ArmDefinition(const std::string &p, uint32_t id, std::vector<std::string> suffixes)
            : prefix(p), can_id(id), joint_suffixes(std::move(suffixes)) {}
    };

    // 存储所有机械臂的定义
    std::vector<ArmDefinition> arm_definitions_;

    // 全局关节映射表: Joint Name -> JointData*
    // 使用 unique_ptr 管理内存，避免手动 new/delete
    std::unordered_map<std::string, std::unique_ptr<JointData>> joint_map_;

    // --- CAN 通信相关 ---
    CanSocket can0_, can1_;
    std::string can0_interface_, can1_interface_;

    // 运行参数
    int max_current_{0};
    int speed_level_{0};

    // --- 辅助方法 ---

    /**
     * @brief 根据 arm_definitions_ 动态构建 joint_map_
     * @return true if success
     */
    bool build_dynamic_joint_map();

    /**
     * @brief 获取指定前缀的 CAN Socket 引用
     */
    CanSocket &get_can_socket_for_arm(const std::string &prefix);
};

} // namespace psm_hardware_interface