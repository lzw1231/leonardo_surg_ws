#include "psm_hardware/intelligent_can/intelligent_can0_hardware_interface.hpp"

namespace psm_hardware_interface {

// ---------------------------------------------------------------------------
// 生命周期回调
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
IntelligentCan0HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &hp = info_.hardware_parameters;
    can_interface_ = hp.at("can_interface");
    slave_1_id_ = std::stoul(hp.at("slave_1_id"), nullptr, 0);
    slave_2_id_ = std::stoul(hp.at("slave_2_id"), nullptr, 0);
    max_current_ = std::stoi(hp.at("max_current"));
    speed_level_ = std::stoi(hp.at("speed_level"));

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IntelligentCan0HardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IntelligentCan0HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IntelligentCan0HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 接口导出
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface::ConstSharedPtr> IntelligentCan0HardwareInterface::on_export_state_interfaces() {
    return {
        // 1 号从臂
        std::make_shared<hardware_interface::StateInterface>("s1_j01_base", "position", &s1_j01_base.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j03_oc", "position", &s1_j03_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j04_oc", "position", &s1_j04_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j07_oc", "position", &s1_j07_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j08_oc", "position", &s1_j08_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j11_oc", "position", &s1_j11_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j12_oc", "position", &s1_j12_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j13_oc", "position", &s1_j13_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j14_oc", "position", &s1_j14_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j17_18", "position", &s1_j17_18.state),
        std::make_shared<hardware_interface::StateInterface>("s1_j20_21", "position", &s1_j20_21.state),
        std::make_shared<hardware_interface::StateInterface>("s1_joint_motor_01", "position", &s1_joint_motor_01.state),
        std::make_shared<hardware_interface::StateInterface>("s1_joint_motor_02", "position", &s1_joint_motor_02.state),
        std::make_shared<hardware_interface::StateInterface>("s1_joint_motor_03", "position", &s1_joint_motor_03.state),
        std::make_shared<hardware_interface::StateInterface>("s1_joint_motor_04", "position", &s1_joint_motor_04.state),

        // 2 号从臂
        std::make_shared<hardware_interface::StateInterface>("s2_j01_base", "position", &s2_j01_base.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j03_oc", "position", &s2_j03_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j04_oc", "position", &s2_j04_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j07_oc", "position", &s2_j07_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j08_oc", "position", &s2_j08_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j11_oc", "position", &s2_j11_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j12_oc", "position", &s2_j12_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j13_oc", "position", &s2_j13_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j14_oc", "position", &s2_j14_oc.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j17_18", "position", &s2_j17_18.state),
        std::make_shared<hardware_interface::StateInterface>("s2_j20_21", "position", &s2_j20_21.state),
        std::make_shared<hardware_interface::StateInterface>("s2_joint_motor_01", "position", &s2_joint_motor_01.state),
        std::make_shared<hardware_interface::StateInterface>("s2_joint_motor_02", "position", &s2_joint_motor_02.state),
        std::make_shared<hardware_interface::StateInterface>("s2_joint_motor_03", "position", &s2_joint_motor_03.state),
        std::make_shared<hardware_interface::StateInterface>("s2_joint_motor_04", "position", &s2_joint_motor_04.state),
    };
}

std::vector<hardware_interface::CommandInterface::SharedPtr> IntelligentCan0HardwareInterface::on_export_command_interfaces() {
    return {
        // 1 号从臂
        std::make_shared<hardware_interface::CommandInterface>("s1_j01_base", "position", &s1_j01_base.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j03_oc", "position", &s1_j03_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j04_oc", "position", &s1_j04_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j07_oc", "position", &s1_j07_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j08_oc", "position", &s1_j08_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j11_oc", "position", &s1_j11_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j12_oc", "position", &s1_j12_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j13_oc", "position", &s1_j13_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j14_oc", "position", &s1_j14_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j17_18", "position", &s1_j17_18.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_j20_21", "position", &s1_j20_21.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_joint_motor_01", "position", &s1_joint_motor_01.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_joint_motor_02", "position", &s1_joint_motor_02.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_joint_motor_03", "position", &s1_joint_motor_03.command),
        std::make_shared<hardware_interface::CommandInterface>("s1_joint_motor_04", "position", &s1_joint_motor_04.command),

        // 2 号从臂
        std::make_shared<hardware_interface::CommandInterface>("s2_j01_base", "position", &s2_j01_base.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j03_oc", "position", &s2_j03_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j04_oc", "position", &s2_j04_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j07_oc", "position", &s2_j07_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j08_oc", "position", &s2_j08_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j11_oc", "position", &s2_j11_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j12_oc", "position", &s2_j12_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j13_oc", "position", &s2_j13_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j14_oc", "position", &s2_j14_oc.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j17_18", "position", &s2_j17_18.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_j20_21", "position", &s2_j20_21.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_joint_motor_01", "position", &s2_joint_motor_01.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_joint_motor_02", "position", &s2_joint_motor_02.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_joint_motor_03", "position", &s2_joint_motor_03.command),
        std::make_shared<hardware_interface::CommandInterface>("s2_joint_motor_04", "position", &s2_joint_motor_04.command),
    };
}

// ---------------------------------------------------------------------------
// 数据读写
// ---------------------------------------------------------------------------
hardware_interface::return_type IntelligentCan0HardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // TODO
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type IntelligentCan0HardwareInterface::write(const rclcpp::Time & /*time*/,
                                                                        const rclcpp::Duration & /*period*/) {
    // TODO
    return hardware_interface::return_type::OK;
}

} // namespace psm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_hardware_interface::IntelligentCan0HardwareInterface, hardware_interface::SystemInterface)