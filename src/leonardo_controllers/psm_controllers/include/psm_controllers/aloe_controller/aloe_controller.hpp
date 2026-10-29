#pragma once

#include "controller_interface/controller_interface.hpp"
#include "psm_controllers/common/surgical_kinematics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace psm_controllers {

class AloeController : public controller_interface::ControllerInterface {
  public:
    AloeController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  private:
    // --- 数据结构 ---
    struct ArmContext {
        std::string name;
        bool is_base_locked = false;

        // 基准数据 (在 epoch 变化时锁定)
        std::array<double, 7> base_master_joints; // 锁定时的主手 IK 结果 (7个自由度)
        std::array<double, 17> base_slave_pos;    // 锁定时的从手硬件位置 (17个接口)

        // 当前数据
        std::array<double, 7> current_master_joints;
    };

    ArmContext arms_[3];
    const std::vector<std::string> arm_prefixes_ = {"s1", "s2", "s3"};

    // --- 通信与状态 ---
    std::string arm_mapping_cmd_topic_name_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr topic_sub_;
    std::mutex data_mutex_;
    std::vector<double> latest_topic_data_;
    double last_epoch_ = -1.0;

    // --- 辅助函数 ---
    bool compute_arm_joints(int arm_index, const std::vector<double> &topic_data, std::array<double, 7> &out_joints);
    void resetBaseFrame(ArmContext &arm, const std::array<double, 7> &current_master, size_t state_offset);
    std::array<double, 17> computeTargetCommands(const ArmContext &arm) const;
};

} // namespace psm_controllers
