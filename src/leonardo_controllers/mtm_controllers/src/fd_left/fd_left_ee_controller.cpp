#include "mtm_controllers/fd_left/fd_left_ee_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <limits>

namespace mtm_controllers{
    controller_interface::CallbackReturn FDLeftEeController::on_init() {
        try {
            auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
            auto_declare<std::vector<std::string>>("buttons", std::vector<std::string>());
            auto_declare<std::vector<double>>("transform_translation", std::vector<double>());
            auto_declare<std::vector<double>>("transform_rotation", std::vector<double>());
        }
        catch (const std::exception& e) {
            fprintf(stderr, "初始化阶段异常：%s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FDLeftEeController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        joints_ = get_node()->get_parameter("joints").as_string_array();
        buttons_ = get_node()->get_parameter("buttons").as_string_array();

        if (joints_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "请在配置中提供关节列表！");
            return controller_interface::CallbackReturn::ERROR;
        }

        // ---- 固定变换解析 ----
        auto transform_translation_param =
            get_node()->get_parameter("transform_translation").as_double_array();
        auto transform_rotation_param =
            get_node()->get_parameter("transform_rotation").as_double_array();

        Eigen::Quaternion<double> q;
        Eigen::Vector3d trans;

        if (transform_translation_param.empty()) {
            trans << 0.0, 0.0, 0.0;
        } else if (transform_translation_param.size() == 3) {
            trans << transform_translation_param[0], transform_translation_param[1],
                transform_translation_param[2];
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "平移格式错误");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (transform_rotation_param.empty()) {
            q = Eigen::Quaternion<double>(1, 0, 0, 0);
        } else if (transform_rotation_param.size() == 3) {
            const double roll = transform_rotation_param[0];
            const double pitch = transform_rotation_param[1];
            const double yaw = transform_rotation_param[2];
            Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            q = roll_angle * pitch_angle * yaw_angle;
        } else if (transform_rotation_param.size() == 4) {
            q = Eigen::Quaternion<double>(
                transform_rotation_param[0], transform_rotation_param[1],
                transform_rotation_param[2], transform_rotation_param[3]);
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "旋转格式错误：仅支持 RPY 或四元数");
            return controller_interface::CallbackReturn::ERROR;
        }

        transform_ = Eigen::Matrix4d::Identity();
        pose_ = Eigen::Matrix4d::Identity();
        transform_.block<3, 3>(0, 0) = q.matrix();
        transform_.block<3, 1>(0, 3) = trans;

        // ---- 引用接口初始化 ----
        reference_interfaces_.assign(REF_SIZE, std::numeric_limits<double>::quiet_NaN());

        RCLCPP_INFO(get_node()->get_logger(), "FDLeftEePoseController 配置完成");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration FDLeftEeController::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::InterfaceConfiguration FDLeftEeController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto& joint : joints_) {
            config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        }
        for (const auto& button : buttons_) {
            config.names.push_back(button + "/" + hardware_interface::HW_IF_POSITION);
        }
        return config;
    }

    controller_interface::CallbackReturn FDLeftEeController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        if (state_interfaces_.size() != (joints_.size() + buttons_.size())) {
            RCLCPP_ERROR(
                get_node()->get_logger(), "状态接口数量不符：期望 %zu，实际 %zu",
                joints_.size() + buttons_.size(), state_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        std::fill(
            reference_interfaces_.begin(), reference_interfaces_.end(),
            std::numeric_limits<double>::quiet_NaN());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FDLeftEeController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        std::fill(
            reference_interfaces_.begin(), reference_interfaces_.end(),
            std::numeric_limits<double>::quiet_NaN());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FDLeftEeController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/) {
        joints_.clear();
        buttons_.clear();
        transform_ = Eigen::Matrix4d::Identity();
        pose_ = Eigen::Matrix4d::Identity();
        reference_interfaces_.clear();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type FDLeftEeController::update_reference_from_subscribers(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        // 本控制器无上游，不订阅话题。
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type FDLeftEeController::update_and_write_commands(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        // ---- 缓存状态接口值 ----
        for (const auto& state_interface : state_interfaces_) {
            auto val_opt = state_interface.get_optional();
            if (val_opt.has_value()) {
                name_if_value_mapping_[state_interface.get_prefix_name()]
                    [state_interface.get_interface_name()] = val_opt.value();
            }
        }

        // ---- 读 rpy ----
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        if (joints_.size() >= 6) {
            roll = lookup_state_interface_value(
                name_if_value_mapping_, joints_[3], hardware_interface::HW_IF_POSITION);
            pitch = lookup_state_interface_value(
                name_if_value_mapping_, joints_[4], hardware_interface::HW_IF_POSITION);
            yaw = lookup_state_interface_value(
                name_if_value_mapping_, joints_[5], hardware_interface::HW_IF_POSITION);

            if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
                RCLCPP_DEBUG(get_node()->get_logger(), "姿态获取失败！（roll、pitch、yaw）");
                return controller_interface::return_type::ERROR;
            }
        }

        // ---- 计算 pose ----
        pose_ = Eigen::Matrix4d::Identity();

        if (joints_.size() >= 3) {
            double p_x = lookup_state_interface_value(
                name_if_value_mapping_, joints_[0], hardware_interface::HW_IF_POSITION);
            double p_y = lookup_state_interface_value(
                name_if_value_mapping_, joints_[1], hardware_interface::HW_IF_POSITION);
            double p_z = lookup_state_interface_value(
                name_if_value_mapping_, joints_[2], hardware_interface::HW_IF_POSITION);

            if (std::isnan(p_x) || std::isnan(p_y) || std::isnan(p_z)) {
                RCLCPP_DEBUG(get_node()->get_logger(), "位置获取失败！（fd_x、fd_y、fd_z）");
                return controller_interface::return_type::ERROR;
            }
            pose_(0, 3) = p_x;
            pose_(1, 3) = p_y;
            pose_(2, 3) = p_z;
        }

        if (joints_.size() >= 6) {
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

            Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
            pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
        }

        pose_ = transform_ * pose_;
        Eigen::Quaternion<double> q(pose_.block<3, 3>(0, 0));

        // ---- 写引用接口 ----
        reference_interfaces_[EE_X] = pose_(0, 3);
        reference_interfaces_[EE_Y] = pose_(1, 3);
        reference_interfaces_[EE_Z] = pose_(2, 3);
        reference_interfaces_[EE_QW] = q.w();
        reference_interfaces_[EE_QX] = q.x();
        reference_interfaces_[EE_QY] = q.y();
        reference_interfaces_[EE_QZ] = q.z();
        reference_interfaces_[EE_ROLL] = roll;
        reference_interfaces_[EE_PITCH] = pitch;
        reference_interfaces_[EE_YAW] = yaw;

        if (!buttons_.empty()) {
            double button_status = lookup_state_interface_value(
                name_if_value_mapping_, buttons_[0], hardware_interface::HW_IF_POSITION);
            reference_interfaces_[EE_BUTTON] = (button_status > 0.5) ? 1.0 : 0.0;
        }

        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface> FDLeftEeController::on_export_reference_interfaces_list() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.reserve(REF_SIZE);

        const std::string& ctrl_name = get_node()->get_name();

        reference_interfaces.emplace_back(ctrl_name, "ee/x", &reference_interfaces_[EE_X]);
        reference_interfaces.emplace_back(ctrl_name, "ee/y", &reference_interfaces_[EE_Y]);
        reference_interfaces.emplace_back(ctrl_name, "ee/z", &reference_interfaces_[EE_Z]);
        reference_interfaces.emplace_back(ctrl_name, "ee/qw", &reference_interfaces_[EE_QW]);
        reference_interfaces.emplace_back(ctrl_name, "ee/qx", &reference_interfaces_[EE_QX]);
        reference_interfaces.emplace_back(ctrl_name, "ee/qy", &reference_interfaces_[EE_QY]);
        reference_interfaces.emplace_back(ctrl_name, "ee/qz", &reference_interfaces_[EE_QZ]);
        reference_interfaces.emplace_back(ctrl_name, "ee/roll", &reference_interfaces_[EE_ROLL]);
        reference_interfaces.emplace_back(ctrl_name, "ee/pitch", &reference_interfaces_[EE_PITCH]);
        reference_interfaces.emplace_back(ctrl_name, "ee/yaw", &reference_interfaces_[EE_YAW]);
        reference_interfaces.emplace_back(ctrl_name, "ee/button", &reference_interfaces_[EE_BUTTON]);

        return reference_interfaces;
    }
} // namespace mtm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mtm_controllers::FDLeftEeController, controller_interface::ChainableControllerInterface)
