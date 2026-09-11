#pragma once

#include <hardware_interface/system_interface.hpp>
#include <fd_vendor/fd_sdk.hpp>
#include "mtm_hardware/common/visibility_control.hpp"

namespace fd_hardware_base{
    /**
     * @brief Force Dimension 硬件接口基类
     *
     * 封装了 DHD SDK 的通用连接、配置、状态导出及生命周期管理逻辑。
     * 子类需实现具体的 read() 和 write() 方法以适配不同型号设备。
     */
    class MTM_HARDWARE_PUBLIC FDHardwareBase : public hardware_interface::SystemInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDHardwareBase);

        ~FDHardwareBase() override = default;

        // 生命周期回调
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        // 接口导出
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

        // 纯虚函数，由子类实现具体数据读写逻辑
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override = 0;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override = 0;

    protected:
        // 设备连接与断开
        bool connectToDevice();
        bool disconnectFromDevice();

        // 硬件状态缓冲区
        std::vector<double> hw_commands_effort_; // 力/力矩命令
        std::vector<double> hw_states_position_; // 位置/姿态状态
        std::vector<double> hw_states_velocity_; // 速度状态
        std::vector<double> hw_states_effort_; // 力/力矩反馈
        std::vector<double> hw_states_inertia_; // 惯性矩阵（上三角扁平化存储）
        std::vector<double> hw_button_state_; // 按钮/GPIO 状态

        // 设备标识与配置参数
        char dev_id_{-1}; // DHD 设备内部 ID
        int interface_SN_{-1}; // 设备序列号
        bool emulate_button_{false}; // 是否启用夹爪按键模拟
        std::string inertia_interface_name_; // 惯性矩阵接口名称前缀
        double effector_mass_{-1.0}; // 末端执行器质量 (kg)
        bool ignore_orientation_{false}; // 是否忽略姿态读数
        bool isConnected_{false}; // 连接状态标志

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
        bool validateJointInterfaces(const hardware_interface::ComponentInfo& joint);
        bool validateGpioInterfaces(const hardware_interface::ComponentInfo& button);
        void parseHardwareParameters();
    };
} // namespace fd_hardware_base
