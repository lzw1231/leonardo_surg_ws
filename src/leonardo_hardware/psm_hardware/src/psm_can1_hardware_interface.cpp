#include "psm_hardware/psm_can/psm_can1_hardware_interface.hpp"

namespace psm_can_hardware {

// ---------------------------------------------------------------------------
// 生命周期回调
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn PSMCan1HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PSMCan1HardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PSMCan1HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PSMCan1HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 接口导出
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface::ConstSharedPtr> PSMCan1HardwareInterface::on_export_state_interfaces() {
    // TODO
    return {};
}

std::vector<hardware_interface::CommandInterface::SharedPtr> PSMCan1HardwareInterface::on_export_command_interfaces() {
    // TODO
    return {};
}

// ---------------------------------------------------------------------------
// 数据读写
// ---------------------------------------------------------------------------
hardware_interface::return_type PSMCan1HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // TODO
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type PSMCan1HardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // TODO
    return hardware_interface::return_type::OK;
}

} // namespace psm_can_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_can_hardware::PSMCan1HardwareInterface, hardware_interface::SystemInterface)