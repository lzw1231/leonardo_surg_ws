#pragma once

#include "psm_controllers/common/visibility_control.hpp"
#include <array>
#include <controller_interface/controller_interface.hpp>
#include <string>
#include <vector>

namespace psm_controllers {

/**
 * IntelligentCan0Controller —— CAN0 末端控制器，控制 psm_1、psm_2 两台从手（每台 6 关节）。
 *
 * 当前阶段：只接中游下发，command 暂未接入。
 *
 * state_interfaces_ 布局（24 个，读中游，索引见 StateIdx）：
 *   psm_1 的 11 个数据 + psm_2 的 11 个数据 + 2 个 epoch
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
    // ========================================================================
    // 常量
    // ========================================================================
    static constexpr size_t kArms = 2;            // psm_1、psm_2
    static constexpr size_t kUpstreamPerArm = 11; // 每台数据字段数

    // 每台从手的 11 个字段后缀（顺序与 StateIdx 严格一致）
    static constexpr const char *kSuffixes[kUpstreamPerArm] = {"x",  "y",    "z",     "qw",  "qx",     "qy",
                                                               "qz", "roll", "pitch", "yaw", "gripper"};

    // 每台从手的上游前缀参数名（与 YAML 参数名逐字对应，顺序与 kArms 一致）
    static constexpr const char *kPrefixParamNames[kArms] = {"psm1_prefix", "psm2_prefix"};

    // ========================================================================
    // state_interfaces_ 索引枚举（24 个）
    // ========================================================================
    enum StateIdx : size_t {
        // psm_1（11 个）
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
        // psm_2（11 个）
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
        // epoch（2 个）
        PSM_1_EPOCH,
        PSM_2_EPOCH,
        // 总数
        STATE_SIZE,
    };

    // ========================================================================
    // 参数（与 YAML 参数名逐字对应）
    // ========================================================================
    std::string psm1_prefix_; // ← YAML: psm1_prefix
    std::string psm2_prefix_; // ← YAML: psm2_prefix

    // ========================================================================
    // configure 时拼好的接口名（顺序与 StateIdx 严格一致）
    // ========================================================================
    std::vector<std::string> state_interfaces_names_; // 24 个

    // ========================================================================
    // 缓存（每帧从 state_interfaces_ 快照）
    // ========================================================================
    std::array<double, STATE_SIZE> upstream_state_{};
};

} // namespace psm_controllers