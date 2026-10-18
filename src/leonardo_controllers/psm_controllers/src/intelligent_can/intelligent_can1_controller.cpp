#include "psm_controllers/intelligent_can/intelligent_can1_controller.hpp"

namespace psm_controllers {

controller_interface::InterfaceConfiguration IntelligentCan1Controller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // TODO: 填入你的 command interfaces，例如：
    // config.names.push_back("can0_joint1/position");

    return config;
}

controller_interface::InterfaceConfiguration IntelligentCan1Controller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // TODO: 填入你的 state interfaces，例如：
    // config.names.push_back("can0_joint1/position");

    return config;
}

controller_interface::CallbackReturn IntelligentCan1Controller::on_init() {
    // TODO: 参数声明、初始化等
    // 例如：get_node()->declare_parameter("joints", ...)

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan1Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO: 读取参数、分配内存、配置硬件通信等

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan1Controller::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO: 激活状态，通常在这里给硬件发送使能指令，或者初始化目标位置

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan1Controller::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO: 取消激活，通常在这里给硬件发送停止或失能指令

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan1Controller::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO: 清理资源，关闭通信端口等

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type IntelligentCan1Controller::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // TODO: 实时控制循环（Real-time loop）
    // 1. 从 state_interfaces_ 读取硬件当前状态
    // 2. 进行控制算法计算
    // 3. 将计算结果写入 command_interfaces_

    return controller_interface::return_type::OK;
}

} // namespace psm_controllers