#pragma once

#include "teleop_controllers/common/visibility_control.hpp"
#include <array>
#include <atomic>
#include <controller_interface/chainable_controller_interface.hpp>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>

namespace teleop_controllers {

/**
 * Pedal6KeysMappingController —— 踏板映射控制器（链式，纯转发）
 *
 * 读左右主手 ee/ 位姿数据，按踏板状态选源，
 * 写到 4 组下游接口（psm_1/psm_2/psm_3/ecm_1）。
 * 额外导出 1 个共享 epoch 接口。
 *
 * 转发字段（11 个，与上游 EE 控制器导出完全一致）：
 *   x, y, z, qw, qx, qy, qz, roll, pitch, yaw, gripper
 *
 * epoch 语义：
 *   每次踏板按键事件（收到一次话题消息）→ epoch +1
 *   下游从手检测到 epoch 变化 → 重置基准（重新定位，不产生位移）
 *
 * 踏板语义（当前）：
 *   1     → 左手数据写入 psm_1，右手数据写入 psm_2
 *   3     → 左手数据写入 psm_2，右手数据写入 psm_3
 *   其他  → 不拷贝数据（保持上次值）
 *   任何键 → epoch +1
 */
class Pedal6KeysMappingController : public controller_interface::ChainableControllerInterface {
  public:
    TELEOP_CONTROLLERS_PUBLIC
    Pedal6KeysMappingController() = default;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  protected:
    TELEOP_CONTROLLERS_PUBLIC
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    TELEOP_CONTROLLERS_PUBLIC
    controller_interface::return_type update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // ========================================================================
    // 常量
    // ========================================================================
    static constexpr size_t SRC_SIZE = 11;   // 每组转发字段数，与上游 EE 控制器一致
    static constexpr size_t GROUP_COUNT = 4; // 4 组下游

    // 上游源在 command_interfaces_ 里的起始索引
    static constexpr size_t SRC_LEFT_IDX = 0;
    static constexpr size_t SRC_RIGHT_IDX = SRC_SIZE; // 11

    // 4 组下游在 refs_ 里的起始索引
    static constexpr size_t DST_PSM_1_IDX = 0 * SRC_SIZE; // 0
    static constexpr size_t DST_PSM_2_IDX = 1 * SRC_SIZE; // 11
    static constexpr size_t DST_PSM_3_IDX = 2 * SRC_SIZE; // 22
    static constexpr size_t DST_ECM_1_IDX = 3 * SRC_SIZE; // 33

    // refs_ 布局：[4 组数据(44)] [1 个共享 epoch(1)] = 45
    // 导出 45 个接口：44 数据 + 1 epoch
    static constexpr size_t DATA_SIZE = GROUP_COUNT * SRC_SIZE; // 44
    static constexpr size_t EPOCH_IDX = DATA_SIZE;              // 44
    static constexpr size_t REFS_SIZE = DATA_SIZE + 1;          // 45

    // 后缀（顺序必须与上游 EE 控制器导出的 ee/ 接口顺序严格一致）
    static constexpr std::array<const char *, SRC_SIZE> kSuffixes{"x", "y", "z", "qw", "qx", "qy", "qz", "roll", "pitch", "yaw", "gripper"};
    static constexpr std::array<const char *, GROUP_COUNT> kGroups{"psm1", "psm2", "psm3", "ecm1"};

    // ========================================================================
    // 参数
    // ========================================================================
    std::string pedal_state_topic_;
    std::string left_prefix_;
    std::string right_prefix_;

    // ========================================================================
    // 上游接口名（configure 时拼好）
    // ========================================================================
    std::vector<std::string> left_interfaces_;  // 11 个
    std::vector<std::string> right_interfaces_; // 11 个

    // ========================================================================
    // 导出内存：4 组 × 11 数据 + 1 共享 epoch = 45 个 double
    // ========================================================================
    std::vector<double> refs_;

    // ========================================================================
    // 订阅与状态
    // ========================================================================
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pedal_sub_;

    // 当前踏板 key（回调写，update 读）
    std::atomic<int> pedal_key_{0};

    // 按键事件计数：回调每收到一次话题 +1
    std::atomic<int> pedal_event_counter_{0};

    // update 线程上次处理的事件计数
    int last_event_counter_ = 0;

    // ========================================================================
    // 辅助函数
    // ========================================================================
    void copy_source_to_dest(size_t src_base, size_t dst_base);
};

} // namespace teleop_controllers