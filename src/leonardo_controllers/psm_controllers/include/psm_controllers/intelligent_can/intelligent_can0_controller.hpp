#pragma once
#include "psm_controllers/common/visibility_control.hpp"
#include <array>
#include <controller_interface/controller_interface.hpp>
#include <cstddef>
#include <hardware_interface/loaned_command_interface.hpp>
#include <limits>
#include <string>
#include <vector>

namespace psm_controllers {

/**
 * IntelligentCan0Controller —— CAN0 末端控制器，控制 psm_1、psm_2 两台从手。
 *
 * state_interfaces_ 布局（24 个，读中游，索引见 StateIdx）：
 *   psm_1 的 11 个数据 + psm_2 的 11 个数据 + 2 个 epoch
 *
 * command_interfaces_ 布局（34 个，写硬件）：
 *   [0, 17)  : psm_1 的 17 个指令（11 个 s1_* + 6 个 s1_joint_motor_*）
 *   [17, 34) : psm_2 的 17 个指令（11 个 s2_* + 6 个 s2_joint_motor_*）
 *
 * 增量控制（基准式，非累加）：
 *   正常帧：Q_out = S_ref + (M_now − M_ref)
 *   踩踏板：M_ref := M_now, S_ref := last_cmd, 本帧 34 接口全发 NaN, last_cmd 不动
 *   首帧 / q_now 算不出：发 NaN，不刷任何基准/缓存
 */
class IntelligentCan0Controller : public controller_interface::ControllerInterface {
  public:
    PSM_CONTROLLERS_PUBLIC
    IntelligentCan0Controller() = default;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // ===================== 常量 =====================
    static constexpr size_t kArms = 2;
    static constexpr size_t kUpstreamPerArm = 11;
    static constexpr size_t kPerArmCommand = 17;
    static constexpr size_t kCommandSize = kArms * kPerArmCommand;
    static constexpr size_t kMappedPerArm = 11; // 参与增量映射的接口数（前 11 个）

    /// 每台从手state字段后缀，顺序和StateIdx严格对齐
    static constexpr const char *kSuffixes[kUpstreamPerArm] = {"x", "y", "z", "qw", "qx", "qy", "qz", "roll", "pitch", "yaw", "gripper"};

    /// yaml参数名，顺序：psm1, psm2
    static constexpr const char *kPrefixParamNames[kArms] = {"psm1_prefix", "psm2_prefix"};

    /// 单臂指令接口后缀，拼接 "s1"+后缀 / "s2"+后缀
    static constexpr const char *kArmCommandSuffix[kPerArmCommand] = {
        "_j01_base/position",       "_j03_oc/position",         "_j04_oc/position",         "_j07_oc/position",         "_j08_oc/position",
        "_j11_oc/position",         "_j12_oc/position",         "_j13_oc/position",         "_j14_oc/position",         "_j17_18/position",
        "_j20_21/position",         "_joint_motor_01/position", "_joint_motor_02/position", "_joint_motor_03/position", "_joint_motor_04/position",
        "_joint_motor_05/position", "_joint_motor_06/position",
    };

    // ===================== state接口索引 =====================
    enum StateIdx : size_t {
        // PSM1 11个
        PSM_1_X = 0,
        PSM_1_Y,
        PSM_1_Z,
        PSM_1_QW,
        PSM_1_QX,
        PSM_1_QY,
        PSM_1_QZ,
        PSM_1_ROLL,
        PSM_1_PITCH,
        PSM_1_YAW,
        PSM_1_GRIPPER,

        // PSM2 11个
        PSM_2_X,
        PSM_2_Y,
        PSM_2_Z,
        PSM_2_QW,
        PSM_2_QX,
        PSM_2_QY,
        PSM_2_QZ,
        PSM_2_ROLL,
        PSM_2_PITCH,
        PSM_2_YAW,
        PSM_2_GRIPPER,

        // epoch
        PSM_1_EPOCH,
        PSM_2_EPOCH,

        STATE_SIZE,
    };

    // ===================== 单臂局部结构体 =====================
    struct ArmStateSnapshot {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double qw{0.0};
        double qx{0.0};
        double qy{0.0};
        double qz{0.0};
        double roll{0.0};
        double pitch{0.0};
        double yaw{0.0};
        double gripper{0.0};
        double epoch{0.0};
    };

    // ===================== 参数 =====================
    std::string psm1_prefix_;
    std::string psm2_prefix_;

    // ===================== 预生成接口名称 =====================
    std::vector<std::string> state_interfaces_names_;
    std::vector<std::string> command_interface_names_;

    // ===================== 状态缓存 =====================
    std::array<double, STATE_SIZE> upstream_state_{};

    // ===================== 增量控制基准（接口空间，11 维/臂）=====================
    std::array<std::array<double, kMappedPerArm>, kArms> master_ref_ci_{}; // M_ref
    std::array<std::array<double, kMappedPerArm>, kArms> slave_ref_ci_{};  // S_ref
    std::array<std::array<double, kMappedPerArm>, kArms> last_cmd_ci_{};   // last_cmd
    bool ref_ready_{false};                                                // on_activate 后为 false，首个有效帧建立基准

    // 上次 epoch（NaN=未初始化，两臂应相同，任一有效同步刷两个）
    std::array<double, kArms> last_epoch_{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};

    // ===================== 私有辅助函数 =====================
    /// 从upstream_state_读取指定臂的快照，arm_idx=0:psm1, 1:psm2
    ArmStateSnapshot read_arm_snapshot(size_t arm_idx) const;

    /// 校验command接口名字，on_activate使用
    bool validate_command_interfaces(const std::vector<hardware_interface::LoanedCommandInterface> &cmd_if) const;
};

} // namespace psm_controllers