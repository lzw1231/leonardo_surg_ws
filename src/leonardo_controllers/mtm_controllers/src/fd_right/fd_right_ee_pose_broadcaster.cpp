#include "mtm_controllers/fd_right/fd_right_ee_pose_broadcaster.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace mtm_controllers{
    // 初始化阶段：仅声明参数，不读取实际值
    controller_interface::CallbackReturn FDRightEePoseBroadcaster::on_init() {
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

    // 配置阶段：读取参数、构造固定变换、创建发布者
    controller_interface::CallbackReturn FDRightEePoseBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        // 读取关节与按钮名称
        joints_ = get_node()->get_parameter("joints").as_string_array();
        buttons_ = get_node()->get_parameter("buttons").as_string_array();

        if (joints_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "请在配置中提供关节列表！");
            return controller_interface::CallbackReturn::ERROR;
        }

        // 读取并解析固定变换参数
        auto transform_translation_param = get_node()->get_parameter("transform_translation").as_double_array();
        auto transform_rotation_param = get_node()->get_parameter("transform_rotation").as_double_array();
        Eigen::Quaternion<double> q;
        Eigen::Vector3d trans;

        // 平移：空则置零，长度为 3 则取值，否则报错
        if (transform_translation_param.empty()) {
            trans << 0.0, 0.0, 0.0;
        } else if (transform_translation_param.size() == 3) {
            trans << transform_translation_param[0], transform_translation_param[1], transform_translation_param[2];
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "平移格式错误");
            return controller_interface::CallbackReturn::ERROR;
        }

        // 旋转：支持空（单位旋转）、3 元（RPY）、4 元（wxyz 四元数）
        if (transform_rotation_param.empty()) {
            q = Eigen::Quaternion<double>(1, 0, 0, 0);
        } else if (transform_rotation_param.size() == 3) {
            const double roll = transform_rotation_param[0]; // 绕 X
            const double pitch = transform_rotation_param[1]; // 绕 Y
            const double yaw = transform_rotation_param[2]; // 绕 Z

            Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
            q = roll_angle * pitch_angle * yaw_angle;
        } else if (transform_rotation_param.size() == 4) {
            q = Eigen::Quaternion<double>(
                transform_rotation_param[0], // w
                transform_rotation_param[1], // x
                transform_rotation_param[2], // y
                transform_rotation_param[3] // z
            );
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "旋转格式错误：仅支持 RPY 或四元数");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        // 构造 4x4 齐次变换矩阵：左上 3x3 为旋转，右上 3x1 为平移
        transform_ = Eigen::Matrix4d::Identity();
        pose_ = Eigen::Matrix4d::Identity();
        transform_.block<3, 3>(0, 0) = q.matrix();
        transform_.block<3, 1>(0, 3) = trans;

        std::cout << transform_ << std::endl;

        // 创建位姿与按钮发布者及其 realtime 包装
        try {
            ee_pose_publisher_ =
                get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/fd_right_ee_pose", rclcpp::SystemDefaultsQoS());

            realtime_ee_pose_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(ee_pose_publisher_);

            button_publisher_ =
                get_node()->create_publisher<std_msgs::msg::Bool>("~/fd_right_button_state", rclcpp::SystemDefaultsQoS());

            realtime_button_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(button_publisher_);
        }
        catch (const std::exception& e) {
            fprintf(stderr, "配置阶段异常：%s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // 命令接口配置：本控制器为纯 broadcaster，不声明命令接口
    controller_interface::InterfaceConfiguration FDRightEePoseBroadcaster::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    // 状态接口配置：按 joints_ 与 buttons_ 拼接 position 接口名
    controller_interface::InterfaceConfiguration FDRightEePoseBroadcaster::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        if (joints_.empty()) {
            RCLCPP_WARN(get_node()->get_logger(), "未提供关节名称！");
        } else {
            for (const auto& joint : joints_) {
                state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
            }
        }

        if (buttons_.empty()) {
            RCLCPP_WARN(get_node()->get_logger(), "未提供按钮名称！");
        } else {
            for (const auto& button : buttons_) {
                state_interfaces_config.names.push_back(button + "/" + hardware_interface::HW_IF_POSITION);
            }
        }

        return state_interfaces_config;
    }

    // 激活阶段：校验租借到的状态接口数量是否与配置一致
    controller_interface::CallbackReturn FDRightEePoseBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        if (state_interfaces_.size() != (joints_.size() + buttons_.size())) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "状态接口数量不符：期望 %zu，实际 %zu",
                         joints_.size() + buttons_.size(),
                         state_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // 停用阶段：仅停止运行，不清配置（支持再次激活）
    controller_interface::CallbackReturn FDRightEePoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // 清理阶段：释放发布者，重置配置与位姿状态
    controller_interface::CallbackReturn FDRightEePoseBroadcaster::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
        // 释放发布者
        realtime_ee_pose_publisher_.reset();
        ee_pose_publisher_.reset();
        realtime_button_publisher_.reset();
        button_publisher_.reset();

        // 重置配置与位姿
        joints_.clear();
        buttons_.clear();
        transform_ = Eigen::Matrix4d::Identity();
        pose_ = Eigen::Matrix4d::Identity();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    // 周期更新：读取状态接口，计算末端位姿并发布
    controller_interface::return_type FDRightEePoseBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
        // 缓存当前所有状态接口值，供后续按名查找
        for (const auto& state_interface : state_interfaces_) {
            auto val_opt = state_interface.get_optional();
            if (val_opt.has_value()) {
                name_if_value_mapping_[state_interface.get_prefix_name()][state_interface.get_interface_name()] = val_opt.value();
                // RCLCPP_INFO(
                //     get_node()->get_logger(), "%s/%s: %f\n", state_interface.get_prefix_name().c_str(),
                //     state_interface.get_interface_name().c_str(), val_opt.value());
            }
        }

        // 发布末端位姿
        if (realtime_ee_pose_publisher_ && realtime_ee_pose_publisher_
            ->
            trylock()
        ) {
            pose_ = Eigen::Matrix4d::Identity();

            // 平移：从 joints_[0..2] 取 x/y/z
            if (joints_.size() >= 3) {
                double p_x = lookup_state_interface_value(name_if_value_mapping_, joints_[0], hardware_interface::HW_IF_POSITION);
                double p_y = lookup_state_interface_value(name_if_value_mapping_, joints_[1], hardware_interface::HW_IF_POSITION);
                double p_z = lookup_state_interface_value(name_if_value_mapping_, joints_[2], hardware_interface::HW_IF_POSITION);

                if (std::isnan(p_x) || std::isnan(p_y) || std::isnan(p_z)) {
                    RCLCPP_DEBUG(get_node()->get_logger(), "位姿获取失败！（fd_x、fd_y、fd_z）");
                    return controller_interface::return_type::ERROR;
                }
                pose_(0, 3) = p_x;
                pose_(1, 3) = p_y;
                pose_(2, 3) = p_z;
            }

            // 旋转：从 joints_[3..5] 取 roll/pitch/yaw，转四元数后写入旋转矩阵
            if (joints_.size() >= 6) {
                double roll = lookup_state_interface_value(name_if_value_mapping_, joints_[3], hardware_interface::HW_IF_POSITION);
                double pitch = lookup_state_interface_value(name_if_value_mapping_, joints_[4], hardware_interface::HW_IF_POSITION);
                double yaw = lookup_state_interface_value(name_if_value_mapping_, joints_[5], hardware_interface::HW_IF_POSITION);

                if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
                    RCLCPP_DEBUG(
                        get_node()->get_logger(), "位姿获取失败！（fd_x、fd_y、fd_z）");
                    return controller_interface::return_type::ERROR;
                }

                Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

                Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
                pose_.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
            }

            // 左乘固定变换，得到最终位姿
            pose_ = transform_ * pose_;
            Eigen::Quaternion<double> q(pose_.block<3, 3>(0, 0));

            auto& ee_pose_msg = realtime_ee_pose_publisher_->msg_;

            ee_pose_msg.header.stamp = time;
            ee_pose_msg.header.frame_id = "fd_right_base";
            // 填充位置
            ee_pose_msg.pose.position.x = pose_(0, 3);
            ee_pose_msg.pose.position.y = pose_(1, 3);
            ee_pose_msg.pose.position.z = pose_(2, 3);
            // 填充姿态
            ee_pose_msg.pose.orientation.w = q.w();
            ee_pose_msg.pose.orientation.x = q.x();
            ee_pose_msg.pose.orientation.y = q.y();
            ee_pose_msg.pose.orientation.z = q.z();

            realtime_ee_pose_publisher_->unlockAndPublish();
        }

        // 发布按钮状态
        if (!buttons_.empty()) {
            if (realtime_button_publisher_ && realtime_button_publisher_
                ->
                trylock()
            ) {
                auto& ee_button_msg = realtime_button_publisher_->msg_;
                double button_status =
                    lookup_state_interface_value(name_if_value_mapping_, buttons_[0], hardware_interface::HW_IF_POSITION);
                ee_button_msg.data = button_status > 0.5;
                realtime_button_publisher_->unlockAndPublish();
            }
        }

        return controller_interface::return_type::OK;
    }
} // namespace mtm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mtm_controllers::FDRightEePoseBroadcaster, controller_interface::ControllerInterface)
