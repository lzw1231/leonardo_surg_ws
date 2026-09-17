#include "mtm_hardware/fd_left/fd_left_hardware.hpp"
#include "mtm_hardware/common/math_utils.hpp"

namespace fd_left_hardware{
    FDLeftHardwareInterface::~FDLeftHardwareInterface() {
        // 确保析构时断开设备连接
        (void)disconnectFromDevice();
    }

    /**
     * @brief 读取硬件状态
     *
     * 从 DHD 设备获取位置、速度、力、惯性矩阵及按钮状态，并更新至状态缓冲区。
     */
    hardware_interface::return_type FDLeftHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        auto logger = rclcpp::get_logger(logger_name_);
        int flag = 0; // 用于累计 API 调用错误码

        // 1. 读取位置与姿态
        flag += dhdGetPosition(&hw_states_position_[0], &hw_states_position_[1], &hw_states_position_[2], interface_id_);
        if (!ignore_orientation_ && hw_states_position_.size() > 3) {
            flag += dhdGetOrientationRad(&hw_states_position_[3], &hw_states_position_[4], &hw_states_position_[5], interface_id_);
        } else if (ignore_orientation_ && hw_states_position_

            .
            size() > 3
        ) {
            hw_states_position_[3] = 0.0;
            hw_states_position_[4] = 0.0;
            hw_states_position_[5] = 0.0;
        }

        // 读取夹爪角度
        if (dhdHasGripper(interface_id_)) {
            if (hw_states_position_.size() == 4) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[3], interface_id_);
            } else if (hw_states_position_.size() > 6) {
                flag += dhdGetGripperAngleRad(&hw_states_position_[6], interface_id_);
            }
        }

        // 2. 读取线速度与角速度
        flag += dhdGetLinearVelocity(&hw_states_velocity_[0], &hw_states_velocity_[1], &hw_states_velocity_[2], interface_id_);
        if (!ignore_orientation_ && hw_states_velocity_.size() > 3) {
            flag += dhdGetAngularVelocityRad(&hw_states_velocity_[3], &hw_states_velocity_[4], &hw_states_velocity_[5], interface_id_);
        } else if (ignore_orientation_ && hw_states_velocity_

            .
            size() > 3
        ) {
            hw_states_velocity_[3] = 0.0;
            hw_states_velocity_[4] = 0.0;
            hw_states_velocity_[5] = 0.0;
        }

        // 读取夹爪角速度
        if (dhdHasGripper(interface_id_)) {
            if (hw_states_velocity_.size() == 4) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[3], interface_id_);
            } else if (hw_states_velocity_.size() > 6) {
                flag += dhdGetGripperAngularVelocityRad(&hw_states_velocity_[6], interface_id_);
            }
        }

        // 3. 读取力、力矩及夹爪力
        double torque[3];
        double gripper_force;
        flag += dhdGetForceAndTorqueAndGripperForce(
            &hw_states_effort_[0], &hw_states_effort_[1], &hw_states_effort_[2],
            &torque[0], &torque[1], &torque[2],
            &gripper_force,
            interface_id_
        );

        // 处理力矩数据
        if (!ignore_orientation_ && hw_states_effort_.size() > 3) {
            hw_states_effort_[3] = torque[0];
            hw_states_effort_[4] = torque[1];
            hw_states_effort_[5] = torque[2];
        } else if (ignore_orientation_ && hw_states_effort_.size() > 3) {
            hw_states_effort_[3] = 0.0;
            hw_states_effort_[4] = 0.0;
            hw_states_effort_[5] = 0.0;
        }

        // 处理夹爪力数据
        if (dhdHasGripper(interface_id_)) {
            if (hw_states_effort_.size() == 4) {
                hw_states_effort_[3] = gripper_force;
            } else if (hw_states_effort_.size() > 6) {
                hw_states_effort_[6] = gripper_force;
            }
        }

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

        // 5. 读取按钮状态
        int button_status = dhdGetButton(0, interface_id_);
        if (button_status == 1) {
            hw_button_state_[0] = 1.0;
        } else if (button_status == 0) {
            hw_button_state_[0] = 0.0;
        } else {
            RCLCPP_ERROR(logger, "按键状态读取异常");
            flag += -1;
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
    hardware_interface::return_type FDLeftHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        // 检查命令中是否存在 NaN
        bool isNan = false;
        for (auto& command : hw_commands_effort_) {
            if (std::isnan(command)) {
                isNan = true;
                break;
            }
        }

        if (!isNan) {
            // 根据设备配置（是否有夹爪、腕部）选择对应的 SDK 调用参数
            if (dhdHasGripper(interface_id_) && hw_states_effort_.size() > 6) {
                // 7-DOF 设备 (带夹爪)
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    hw_commands_effort_[6], interface_id_);
            } else if (dhdHasWrist(interface_id_) && hw_states_effort_.size() == 4) {
                // 4-DOF 设备 (带夹爪，无力矩)
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0.0, 0.0, 0.0, hw_commands_effort_[3], interface_id_);
            } else if (dhdHasWrist(interface_id_) && hw_states_effort_.size() > 3) {
                // 6-DOF 设备 (带腕部，无夹爪)
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    hw_commands_effort_[3], hw_commands_effort_[4], hw_commands_effort_[5],
                    0, interface_id_);
            } else {
                // 3-DOF 设备 (仅平移)
                dhdSetForceAndTorqueAndGripperForce(
                    hw_commands_effort_[0], hw_commands_effort_[1], hw_commands_effort_[2],
                    0, 0, 0, 0, interface_id_);
            }
        } else {
            // 安全模式：发送零力
            dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, interface_id_);
        }
        return hardware_interface::return_type::OK;
    }
} // namespace fd_left_hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(fd_left_hardware::FDLeftHardwareInterface, hardware_interface::SystemInterface)
