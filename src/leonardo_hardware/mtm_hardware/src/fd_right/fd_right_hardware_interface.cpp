#include "mtm_hardware/fd_right/fd_right_hardware_interface.hpp"
#include "mtm_hardware/common/math_utils.hpp"

namespace mtm_hardware_interface {
FDRightHardwareInterface::~FDRightHardwareInterface() {
    // 确保析构时断开设备连接
    (void)disconnectFromDevice();
}

/**
 * @brief 读取硬件状态
 *
 * 从 DHD 设备获取位置、速度、力、惯性矩阵，并更新至状态缓冲区。
 */
hardware_interface::return_type FDRightHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    auto logger = rclcpp::get_logger(logger_name_);
    int flag = 0; // 用于累计 API 调用错误码

    // 1. 读取末端位置和姿态、夹爪弧度（Omega 7 固定 7 关节：xyz + rpy + gripper）
    flag += dhdGetPosition(&hw_states_position_[0], &hw_states_position_[1], &hw_states_position_[2], interface_id_);
    flag += dhdGetOrientationRad(&hw_states_position_[3], &hw_states_position_[4], &hw_states_position_[5], interface_id_);
    flag += dhdGetGripperAngleRad(&hw_states_position_[6], interface_id_);

    // 2. 读取线速度、角速度、夹爪角速度（Omega 7 固定 7 关节）
    flag += dhdGetLinearVelocity(&hw_states_velocity_[0], &hw_states_velocity_[1], &hw_states_velocity_[2], interface_id_);
    flag += dhdGetAngularVelocityRad(&hw_states_velocity_[3], &hw_states_velocity_[4], &hw_states_velocity_[5], interface_id_);
    flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[6], interface_id_);

    // 3. 读取力、力矩、夹爪力（Omega 7 固定 7 关节）
    flag += dhdGetForceAndTorqueAndGripperForce(&hw_states_effort_[0], &hw_states_effort_[1], &hw_states_effort_[2], // 力 xyz
                                                &hw_states_effort_[3], &hw_states_effort_[4], &hw_states_effort_[5], // 力矩 rpy
                                                &hw_states_effort_[6],                                               // 夹爪力
                                                interface_id_);

    // 4. 计算惯性矩阵
    double inertia_array[6][6];
    double joint_position[DHD_MAX_DOF];
    flag += dhdEnableExpertMode();
    flag += dhdGetJointAngles(joint_position, interface_id_);
    flag += dhdJointAnglesToInertiaMatrix(joint_position, inertia_array, interface_id_);
    flag += dhdDisableExpertMode();

    // 将上三角矩阵映射到扁平化数组
    for (uint row = 0; row < 6; row++) {
        for (uint col = row; col < 6; col++) {
            hw_states_inertia_[hw_math::flattened_index_from_triangular_index(row, col)] = inertia_array[row][col];
        }
    }

    // 检查整体执行状态
    if (flag >= 0) {
        return hardware_interface::return_type::OK;
    } else {
        RCLCPP_ERROR(logger, "DHD API 调用出现错误");
        return hardware_interface::return_type::ERROR;
    }
}

/**
 * @brief 写入硬件命令
 *
 * 将计算得到的力/力矩命令发送至 DHD 设备。若命令包含 NaN，则发送零力以确保安全。
 */
hardware_interface::return_type FDRightHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    // 检查命令中是否存在 NaN（安全保护）
    bool isNan = false;
    for (const auto &command : hw_commands_effort_) {
        if (std::isnan(command)) {
            isNan = true;
            break;
        }
    }

    if (!isNan) {
        // Omega 7：力 xyz + 力矩 rpy + 夹爪力，一次发全
        dhdSetForceAndTorqueAndGripperForce(hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2], // 力 xyz
                                            hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5], // 力矩 rpy
                                            hw_commands_effort_[6],                                                 // 夹爪力
                                            interface_id_);
    } else {
        // 安全模式：发零力
        dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, interface_id_);
    }

    return hardware_interface::return_type::OK;
}
} // namespace mtm_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mtm_hardware_interface::FDRightHardwareInterface, hardware_interface::SystemInterface)