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

            // ★ 新增：三个话题名
            auto_declare<std::string>("pose_topic", "~/fd_right_ee_pose");
            auto_declare<std::string>("button_topic", "~/fd_right_button_state");
            auto_declare<std::string>("rpy_topic", "~/fd_right_ee_rpy");
        }
        catch (const std::exception& e) {
            fprintf(stderr, "初始化阶段异常：%s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // 配置阶段：读取参数、构造固定变换、创建发布者
    controller_interface::CallbackReturn
    FDRightEePoseBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        joints_ = get_node()->get_parameter("joints").as_string_array();
        buttons_ = get_node()->get_parameter("buttons").as_string_array();

        // ★ 新增：读取三个话题名
        pose_topic_ = get_node()->get_parameter("pose_topic").as_string();
        button_topic_ = get_node()->get_parameter("button_topic").as_string();
        rpy_topic_ = get_node()->get_parameter("rpy_topic").as_string();

        if (joints_.empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "请在配置中提供关节列表！");
            return controller_interface::CallbackReturn::ERROR;
        }

        auto transform_translation_param = get_node()->get_parameter("transform_translation").as_double_array();
        auto transform_rotation_param = get_node()->get_parameter("transform_rotation").as_double_array();
        Eigen::Quaternion<double> q;
        Eigen::Vector3d trans;

        if (transform_translation_param.empty()) {
            trans << 0.0, 0.0, 0.0;
        } else if (transform_translation_param.size() == 3) {
            trans << transform_translation_param[0], transform_translation_param[1], transform_translation_param[2];
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
                transform_rotation_param[0],
                transform_rotation_param[1],
                transform_rotation_param[2],
                transform_rotation_param[3]
            );
        } else {
            RCLCPP_ERROR(get_node()->get_logger(), "旋转格式错误：仅支持 RPY 或四元数");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        transform_ = Eigen::Matrix4d::Identity();
        pose_ = Eigen::Matrix4d::Identity();
        transform_.block<3, 3>(0, 0) = q.matrix();
        transform_.block<3, 1>(0, 3) = trans;

        std::cout << transform_ << std::endl;

        try {
            // ★ 用从 YAML 读到的 topic 名
            ee_pose_publisher_ =
                get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
                    pose_topic_, rclcpp::SystemDefaultsQoS());
            realtime_ee_pose_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(ee_pose_publisher_);

            button_publisher_ =
                get_node()->create_publisher<std_msgs::msg::Bool>(
                    button_topic_, rclcpp::SystemDefaultsQoS());
            realtime_button_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(button_publisher_);

            ee_rpy_publisher_ =
                get_node()->create_publisher<example_interfaces::msg::Float64MultiArray>(
                    rpy_topic_, rclcpp::SystemDefaultsQoS());
            realtime_ee_rpy_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<example_interfaces::msg::Float64MultiArray>>(
                    ee_rpy_publisher_);

            RCLCPP_INFO(get_node()->get_logger(),
                        "发布话题：pose=[%s], button=[%s], rpy=[%s]",
                        pose_topic_.c_str(), button_topic_.c_str(), rpy_topic_.c_str());
        }
        catch (const std::exception& e) {
            fprintf(stderr, "配置阶段异常：%s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    FDRightEePoseBroadcaster::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::InterfaceConfiguration
    FDRightEePoseBroadcaster::state_interface_configuration() const {
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

    controller_interface::CallbackReturn
    FDRightEePoseBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        if (state_interfaces_.size() != (joints_.size() + buttons_.size())) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "状态接口数量不符：期望 %zu，实际 %zu",
                         joints_.size() + buttons_.size(),
                         state_interfaces_.size());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    FDRightEePoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    FDRightEePoseBroadcaster::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
        realtime_ee_pose_publisher_.reset();
        ee_pose_publisher_.reset();

        realtime_ee_rpy_publisher_.reset();
        ee_rpy_publisher_.reset();

        realtime_button_publisher_.reset();
        button_publisher_.reset();

        joints_.clear();
        buttons_.clear();
        pose_topic_.clear();
        button_topic_.clear();
        rpy_topic_.clear();
        transform_ = Eigen::Matrix4d::Identity();
        pose_ = Eigen::Matrix4d::Identity();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type
    FDRightEePoseBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
        for (const auto& state_interface : state_interfaces_) {
            auto val_opt = state_interface.get_optional();
            if (val_opt.has_value()) {
                name_if_value_mapping_[state_interface.get_prefix_name()][state_interface.get_interface_name()] =
                    val_opt.value();
            }
        }

        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        if (joints_.size() >= 6) {
            roll = lookup_state_interface_value(name_if_value_mapping_, joints_[3], hardware_interface::HW_IF_POSITION);
            pitch = lookup_state_interface_value(name_if_value_mapping_, joints_[4], hardware_interface::HW_IF_POSITION);
            yaw = lookup_state_interface_value(name_if_value_mapping_, joints_[5], hardware_interface::HW_IF_POSITION);

            if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
                RCLCPP_DEBUG(get_node()->get_logger(), "姿态获取失败！（roll、pitch、yaw）");
                return controller_interface::return_type::ERROR;
            }
        }

        if (realtime_ee_pose_publisher_ && realtime_ee_pose_publisher_->trylock()) {
            pose_ = Eigen::Matrix4d::Identity();

            if (joints_.size() >= 3) {
                double p_x = lookup_state_interface_value(name_if_value_mapping_, joints_[0], hardware_interface::HW_IF_POSITION);
                double p_y = lookup_state_interface_value(name_if_value_mapping_, joints_[1], hardware_interface::HW_IF_POSITION);
                double p_z = lookup_state_interface_value(name_if_value_mapping_, joints_[2], hardware_interface::HW_IF_POSITION);

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

            auto& ee_pose_msg = realtime_ee_pose_publisher_->msg_;

            ee_pose_msg.header.stamp = time;
            ee_pose_msg.header.frame_id = "fd_right_base";
            ee_pose_msg.pose.position.x = pose_(0, 3);
            ee_pose_msg.pose.position.y = pose_(1, 3);
            ee_pose_msg.pose.position.z = pose_(2, 3);
            ee_pose_msg.pose.orientation.w = q.w();
            ee_pose_msg.pose.orientation.x = q.x();
            ee_pose_msg.pose.orientation.y = q.y();
            ee_pose_msg.pose.orientation.z = q.z();

            realtime_ee_pose_publisher_->unlockAndPublish();
        }

        if (realtime_ee_rpy_publisher_ && realtime_ee_rpy_publisher_->trylock()) {
            auto& rpy_msg = realtime_ee_rpy_publisher_->msg_;
            rpy_msg.data.resize(3);
            rpy_msg.data[0] = roll;
            rpy_msg.data[1] = pitch;
            rpy_msg.data[2] = yaw;
            realtime_ee_rpy_publisher_->unlockAndPublish();
        }

        if (!buttons_.empty()) {
            if (realtime_button_publisher_ && realtime_button_publisher_->trylock()) {
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
