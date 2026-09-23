#pragma once

#include "psm_controllers/common/visibility_control.hpp"
#include <array>
#include <controller_interface/controller_interface.hpp>
#include <string>
#include <vector>

namespace psm_controllers {

/**
 * FT3215Controller（从手控制器，链末端）
 *
 * 从上游读 6 个参考接口（x/y/z/roll/pitch/yaw）+ 1 个 epoch，
 * 用增量方式累加到从手命令，写到 6 个 FT3215 电机。
 *
 * 增量逻辑：
 *   第一帧 / epoch 变化：基准 = 硬件当前位置；last_target = 上游值；不动
 *   其余每帧：delta = (上游值 - last_target) * kScale；slave_cmd += delta
 *   上游无效帧：跳过（last_target 不更新，下帧自动补偿）
 *
 * 缩放：
 *   x/y/z 位置轴用 kScalePos 放大，让从手动作更明显；
 *   roll/pitch/yaw 姿态轴不缩放（保持 1.0）。
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

    static constexpr const char *kUpstreamSuffixes[6] = {"x", "y", "z", "roll", "pitch", "yaw"};

  private:
    static constexpr size_t kN = 6;

    // command_interfaces_ 里的索引
    static constexpr size_t kEpochCmdIdx = kN;       // 第 7 个：epoch
    static constexpr size_t kSlaveCmdStart = kN + 1; // 第 8 个起：从手命令

    // x/y/z 增量放大倍数
    static constexpr double kScalePos = 5.0;

    // 各轴缩放系数：x/y/z 放大，roll/pitch/yaw 保持 1.0
    static constexpr std::array<double, kN> kScale{kScalePos, kScalePos, kScalePos, // x, y, z
                                                   1.0,       1.0,       1.0};      // roll, pitch, yaw

    // ---- 参数 ----
    std::vector<std::string> joint_names_;
    std::string interface_name_;
    std::string upstream_prefix_;

    // 上游 6 个数据接口完整名（不含 epoch）
    std::vector<std::string> upstream_interfaces_;

    // ---- 增量状态 ----
    std::array<double, kN> slave_cmd_;   // 从手命令（自己累加）
    std::array<double, kN> last_target_; // 上一帧的上游值
    bool initialized_ = false;           // 第一帧标志
    double last_epoch_ = -1.0;           // 上一帧的 epoch 值
};

} // namespace psm_controllers