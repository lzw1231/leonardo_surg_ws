#pragma once

#include "psm_controllers/common/visibility_control.hpp"
#include <array>
#include <controller_interface/controller_interface.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>

namespace psm_controllers {

/**
 * AloeController —— 末端控制器，统一控制 psm1 / psm2 / psm3 三台从手， ecm1 内窥镜
 *
 * state 侧（45 个，读上游）：4 臂 × 11 字段 + 1 共享 epoch
 * command 侧（51 个，写硬件）：3 臂 × 17 关节（ecm1 暂无 command）
 */
class AloeController : public controller_interface::ControllerInterface {
  public:
    PSM_CONTROLLERS_PUBLIC
    AloeController() = default;

    // ---- 接口声明 ----
    PSM_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    PSM_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // ---- 初始化：声明 YAML 参数 ----
    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    // ---- 配置：拼接口名 ----
    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    // ---- 激活：校验接口、初始化基准 ----
    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    // ---- 周期运行 ----
    PSM_CONTROLLERS_PUBLIC
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // ---- 去激活 ----
    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // ---- 清理 ----
    PSM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  private:
    // ---- 规模常量 ----
    static constexpr size_t kArmsState = 4;                                // psm1 / psm2 / psm3 / ecm1
    static constexpr size_t kArmsCommand = 3;                              // psm1 / psm2 / psm3
    static constexpr size_t kUpstreamPerArm = 11;                          // 每臂上游字段数
    static constexpr size_t kPerArmCommand = 17;                           // 每臂指令接口数
    static constexpr size_t kMappedPerArm = 11;                            // 参与增量映射的前 11 个
    static constexpr size_t kCommandSize = kArmsCommand * kPerArmCommand;  // 51
    static constexpr size_t kStateSize = kArmsState * kUpstreamPerArm + 1; // 45

    // ---- 上游字段后缀 ----
    static constexpr std::array<const char *, kUpstreamPerArm> kSuffixes{"x",  "y",    "z",     "qw",  "qx",     "qy",
                                                                         "qz", "roll", "pitch", "yaw", "gripper"};

    // ---- 单臂 command 后缀 ----
    static constexpr std::array<const char *, kPerArmCommand> kArmCommandSuffix{
        "_j01_base/position",       "_j03_oc/position",         "_j04_oc/position",         "_j07_oc/position",
        "_j08_oc/position",         "_j11_oc/position",         "_j12_oc/position",         "_j13_oc/position",
        "_j14_oc/position",         "_j17_18/position",         "_j20_21/position",         "_joint_motor_01/position",
        "_joint_motor_02/position", "_joint_motor_03/position", "_joint_motor_04/position", "_joint_motor_05/position",
        "_joint_motor_06/position",
    };

    // ---- command 硬件前缀 ----
    static constexpr std::array<const char *, kArmsCommand> kCmdPrefixes{"s1", "s2", "s3"};

    // ---- YAML 参数名 ----
    static constexpr std::array<const char *, kArmsState> kPrefixParamNames{"psm1_prefix", "psm2_prefix", "psm3_prefix", "ecm1_prefix"};

    // ---- 单臂运行时基准 ----
    struct ArmRuntime {
        std::array<double, kMappedPerArm> master_ref_ci{}; // M_ref
        std::array<double, kMappedPerArm> slave_ref_ci{};  // S_ref
        std::array<double, kMappedPerArm> last_cmd_ci{};   // last_cmd
    };

    ArmRuntime psm1_, psm2_, psm3_;
    std::array<ArmRuntime *, kArmsCommand> arms_{&psm1_, &psm2_, &psm3_};

    // ---- 运行数据 ----
    std::array<std::string, kArmsState> prefixes_{};
    std::vector<std::string> state_interfaces_names_;
    std::vector<std::string> command_interface_names_;

    std::array<double, kStateSize> upstream_state_{};
    bool ref_ready_{false};
    double last_epoch_{std::numeric_limits<double>::quiet_NaN()};
};

} // namespace psm_controllers