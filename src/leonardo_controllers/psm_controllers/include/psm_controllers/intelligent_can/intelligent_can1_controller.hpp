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
 * IntelligentCan1Controller —— CAN1 末端控制器，控制 psm_3 一条从手。
 *
 * state_interfaces_ 布局（12 个，读中游，索引见 StateIdx）：
 *   psm_3 的 11 个数据 + 1 个 epoch
 *
 * command_interfaces_ 布局（17 个，写硬件）：
 *   [0, 11) : 11 个 s3_* 映射接口（参与增量控制）
 *   [11, 17): 6 个 s3_joint_motor_*（不参与映射，保持 on_activate 的 0）
 *
 * 增量控制（基准式，非累加）：
 *   正常帧：Q_out = S_ref + (M_now − M_ref)
 *   踩踏板：M_ref := M_now, S_ref := last_cmd, 本帧 17 接口全发 NaN, last_cmd 不动
 *   首帧 / q_now 算不出：发 NaN，不刷任何基准/缓存
 */
class IntelligentCan1Controller : public controller_interface::ControllerInterface {
  public:
    PSM_CONTROLLERS_PUBLIC
    IntelligentCan1Controller() = default;

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
    static constexpr size_t kUpstreamPerArm = 11;
    static constexpr size_t kPerArmCommand = 17;
    static constexpr size_t kCommandSize = kPerArmCommand;
    static constexpr size_t kMappedPerArm = 11;

    static constexpr const char *kSuffixes[kUpstreamPerArm] = {"x", "y", "z", "qw", "qx", "qy", "qz", "roll", "pitch", "yaw", "gripper"};

    static constexpr const char *kArmCommandSuffix[kPerArmCommand] = {
        "_j01_base/position",       "_j03_oc/position",         "_j04_oc/position",         "_j07_oc/position",         "_j08_oc/position",
        "_j11_oc/position",         "_j12_oc/position",         "_j13_oc/position",         "_j14_oc/position",         "_j17_18/position",
        "_j20_21/position",         "_joint_motor_01/position", "_joint_motor_02/position", "_joint_motor_03/position", "_joint_motor_04/position",
        "_joint_motor_05/position", "_joint_motor_06/position",
    };

    // ===================== state接口索引 =====================
    enum StateIdx : size_t {
        PSM_3_X = 0,
        PSM_3_Y,
        PSM_3_Z,
        PSM_3_QW,
        PSM_3_QX,
        PSM_3_QY,
        PSM_3_QZ,
        PSM_3_ROLL,
        PSM_3_PITCH,
        PSM_3_YAW,
        PSM_3_GRIPPER,
        PSM_3_EPOCH,
        STATE_SIZE,
    };

    // ===================== 单臂快照 =====================
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
    std::string psm3_prefix_; // ← YAML: psm3_prefix

    // ===================== 预生成接口名称 =====================
    std::vector<std::string> state_interfaces_names_;
    std::vector<std::string> command_interface_names_;

    // ===================== 状态缓存 =====================
    std::array<double, STATE_SIZE> upstream_state_{};

    // ===================== 增量控制基准（接口空间，11 维）=====================
    std::array<double, kMappedPerArm> master_ref_ci_{}; // M_ref
    std::array<double, kMappedPerArm> slave_ref_ci_{};  // S_ref
    std::array<double, kMappedPerArm> last_cmd_ci_{};   // last_cmd
    bool ref_ready_{false};

    // 上次 epoch（NaN=未初始化）
    double last_epoch_{std::numeric_limits<double>::quiet_NaN()};

    // ===================== 私有辅助函数 =====================
    ArmStateSnapshot read_arm_snapshot() const;
    bool validate_command_interfaces(const std::vector<hardware_interface::LoanedCommandInterface> &cmd_if) const;
};

} // namespace psm_controllers