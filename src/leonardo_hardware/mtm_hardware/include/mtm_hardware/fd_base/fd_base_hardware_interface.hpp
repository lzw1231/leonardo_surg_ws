#pragma once

#include "mtm_hardware/common/visibility_control.hpp"
#include <fd_vendor/fd_sdk.hpp>
#include <hardware_interface/system_interface.hpp>

namespace mtm_hardware_interface {
/**
 * @brief Force Dimension 硬件接口基类
 *
 * 封装了 DHD SDK 的通用连接、配置、状态导出及生命周期管理逻辑。
 * 子类需实现具体的 read() 和 write() 方法以适配不同型号设备。
 */
class FDBaseHardwareInterface : public hardware_interface::SystemInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FDBaseHardwareInterface);

    MTM_HARDWARE_PUBLIC
    ~FDBaseHardwareInterface() override = default;

    // 生命周期回调
    MTM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
    MTM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    MTM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    MTM_HARDWARE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    // 接口导出
    MTM_HARDWARE_PUBLIC
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
    MTM_HARDWARE_PUBLIC
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

    // 纯虚函数，由子类实现具体数据读写逻辑
    MTM_HARDWARE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override = 0;
    MTM_HARDWARE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override = 0;

  protected:
    bool connectToDevice();
    bool disconnectFromDevice();

    // 硬件状态缓冲区
    std::vector<double> hw_commands_effort_; // 力/力矩命令
    std::vector<double> hw_states_position_; // 位置/姿态状态
    std::vector<double> hw_states_velocity_; // 速度状态
    std::vector<double> hw_states_effort_;   // 力/力矩反馈
    std::vector<double> hw_states_inertia_;  // 惯性矩阵（上三角扁平化存储）

    // 设备标识与配置参数
    char interface_id_{-1};              // 设备内部ID
    int interface_sn_{-1};               // 设备序列号
    std::string inertia_interface_name_; // 惯性矩阵接口名称前缀
    double effector_mass_{-1.0};         // 末端执行器质量 (kg)
    bool isConnected_{false};            // 连接状态标志

    // 常量定义
    static constexpr size_t INERTIA_MATRIX_FLATTEN_SIZE = 21U; // 6x6 对称矩阵上三角元素个数
    static constexpr double BUF_INIT_NAN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double DEFAULT_MAX_FORCE{12.0}; // 默认最大输出力 (N)

    // 接口存储容器（用于管理 SharedPtr 生命周期）
    std::vector<hardware_interface::StateInterface::SharedPtr> state_if_storage_;
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_storage_;

    std::string logger_name_{"FDHardwareBase"};

  private:
    // 辅助验证与解析函数
    bool validateJointInterfaces(const hardware_interface::ComponentInfo &joint);
    void parseHardwareParameters();
};
} // namespace mtm_hardware_interface
