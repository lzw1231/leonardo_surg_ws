// ============================================================================
// 文件 1: aloe_controller.hpp
// ============================================================================
#pragma once

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

// 假设这些是你项目中已有的依赖
#include "psm_controllers/common/psm_kinematics.hpp"

namespace psm_controllers {

class AloeController : public controller_interface::ControllerInterface {
  public:
    AloeController() = default;

    // --- ROS2 Controller 标准接口 ---
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  private:
    // --- 常量配置 ---
    static constexpr int NUM_ARMS = 3;
    static constexpr int JOINTS_PER_ARM = 7;
    static constexpr int CMD_INTERFACES_PER_ARM = 17;
    static constexpr int TOPIC_DATA_SIZE = 45;
    static constexpr int DATA_PER_ARM_IN_TOPIC = 11;

    // --- 数据结构 ---
    struct ArmContext {
        std::string name;
        std::array<double, JOINTS_PER_ARM> master_ref_joints;
        std::array<double, JOINTS_PER_ARM> slave_ref_joints;
        std::array<double, JOINTS_PER_ARM> current_master_joints;
        bool is_initialized = false;
    };

    ArmContext arms_[NUM_ARMS];
    const std::vector<std::string> arm_prefixes_ = {"s1", "s2", "s3"};

    // --- 通信与状态 ---
    std::string teleop_topic_name_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr teleop_sub_;

    std::mutex data_mutex_;
    std::vector<double> latest_topic_data_;
    double last_epoch_ = std::numeric_limits<double>::quiet_NaN();

    // --- 【新增】用于离合器/异常状态下的位置保持 ---
    // 缓存每个臂上一帧成功计算出的 17 个 command interface 的值
    std::array<std::array<double, CMD_INTERFACES_PER_ARM>, NUM_ARMS> last_valid_commands_;

    // --- 内部辅助方法 ---
    bool compute_arm_joints(int arm_index, const std::vector<double> &topic_data, std::array<double, JOINTS_PER_ARM> &out_joints);
    void write_to_hardware(int arm_index, const std::array<double, JOINTS_PER_ARM> &joints, bool send_nan = false);
    void reset_all_references();
};

} // namespace psm_controllers
