#include "psm_hardware/intelligent_can/intelligent_can1_hardware_interface.hpp"

namespace psm_hardware_interface {

// ---------------------------------------------------------------------------
// 生命周期回调
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
IntelligentCan1HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &hp = info_.hardware_parameters;
    can_interface_ = hp.at("can_interface");
    slave_3_id_ = std::stoul(hp.at("slave_3_id"), nullptr, 0);
    max_current_ = std::stoi(hp.at("max_current"));
    speed_level_ = std::stoi(hp.at("speed_level"));

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IntelligentCan1HardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IntelligentCan1HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IntelligentCan1HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 接口导出
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface::ConstSharedPtr> IntelligentCan1HardwareInterface::on_export_state_interfaces() {
    return {
        // 3 号从臂
        std::make_shared<hardware_interface::StateInterface>("s3_j01_base", "position", &s3_j01_base.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j03_oc", "position", &s3_j03_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j04_oc", "position", &s3_j04_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j07_oc", "position", &s3_j07_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j08_oc", "position", &s3_j08_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j11_oc", "position", &s3_j11_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j12_oc", "position", &s3_j12_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j13_oc", "position", &s3_j13_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j14_oc", "position", &s3_j14_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j17_18", "position", &s3_j17_18.state),
        std::make_shared<hardware_interface::StateInterface>("s3_j20_21", "position", &s3_j20_21.state),
        std::make_shared<hardware_interface::StateInterface>("s3_joint_motor_01", "position", &s3_joint_motor_01.state),
        std::make_shared<hardware_interface::StateInterface>("s3_joint_motor_02", "position", &s3_joint_motor_02.state),
        std::make_shared<hardware_interface::StateInterface>("s3_joint_motor_03", "position", &s3_joint_motor_03.state),
        std::make_shared<hardware_interface::StateInterface>("s3_joint_motor_04", "position", &s3_joint_motor_04.state),
    };
}

std::vector<hardware_interface::CommandInterface::SharedPtr> IntelligentCan1HardwareInterface::on_export_command_interfaces() {
    return {
        // 3 号从臂
        std::make_shared<hardware_interface::CommandInterface>("s3_j01_base", "position", &s3_j01_base.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j03_oc", "position", &s3_j03_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j04_oc", "position", &s3_j04_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j07_oc", "position", &s3_j07_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j08_oc", "position", &s3_j08_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j11_oc", "position", &s3_j11_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j12_oc", "position", &s3_j12_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j13_oc", "position", &s3_j13_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j14_oc", "position", &s3_j14_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j17_18", "position", &s3_j17_18.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_j20_21", "position", &s3_j20_21.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_joint_motor_01", "position", &s3_joint_motor_01.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_joint_motor_02", "position", &s3_joint_motor_02.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_joint_motor_03", "position", &s3_joint_motor_03.command),
        std::make_shared<hardware_interface::CommandInterface>("s3_joint_motor_04", "position", &s3_joint_motor_04.command),
    };
}

// ---------------------------------------------------------------------------
// 数据读写
// ---------------------------------------------------------------------------
hardware_interface::return_type IntelligentCan1HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // TODO
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IntelligentCan1HardwareInterface::write(const rclcpp::Time & /*time*/,
                                                                        const rclcpp::Duration & /*period*/) {
    // TODO
    return hardware_interface::return_type::OK;
}

} // namespace psm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_hardware_interface::IntelligentCan1HardwareInterface, hardware_interface::SystemInterface)