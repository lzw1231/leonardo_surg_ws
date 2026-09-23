#pragma once

#include "psm_controllers/common/visibility_control.hpp"
#include <controller_interface/controller_interface.hpp>
#include <string>
#include <vector>

namespace psm_controllers {

/**
 * FT3215Controller（从手控制器，链末端）
 *
 * 从上游链式控制器读取 6 个参考接口：x, y, z, roll, pitch, yaw，
 * 按 coefficient_ 与从手当前硬件位置插值后，写入 6 个 FT3215 的 command interface。
 *
 * 本控制器为链末端，不向下游导出引用接口，因此继承 ControllerInterface
 *
 * 参数：
 *   joints:           从手 6 个 FT3215 电机名，长度必须为 6
 *   interface_name:   command/state interface 名，默认 "position"
 *   coefficient:      平滑系数，默认 1.0（1.0 = 直接跟随上游）
 *   upstream_prefix:  上游引用接口前缀，如 "fd_left_ee_controller/ee"
 *                     控制器内部拼成 <upstream_prefix>/x, /y, /z, /roll, /pitch, /yaw
 */
class FT3215Controller : public controller_interface::ControllerInterface {
  public:
    PSM_CONTROLLERS_PUBLIC
    FT3215Controller() = default;

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

    // 上游 6 个参考接口的相对后缀，顺序固定
    static constexpr const char *kUpstreamSuffixes[6] = {"x", "y", "z", "roll", "pitch", "yaw"};

    // ---- 参数 ----
    std::vector<std::string> joint_names_; // 6 个 FT3215
    std::string interface_name_;           // 默认 "position"
    double coefficient_ = 1.0;             // 平滑系数
    std::string upstream_prefix_;          // 如 "fd_left_ee_controller/ee"

    // 上游 6 个接口完整名，on_configure 里拼好后缓存
    std::vector<std::string> upstream_interfaces_;
};

} // namespace psm_controllers