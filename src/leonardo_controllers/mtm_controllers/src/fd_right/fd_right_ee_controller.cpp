#include "mtm_controllers/fd_right/fd_right_ee_controller.hpp"

#include <algorithm>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>

namespace mtm_controllers {

namespace {
constexpr double qNaN = std::numeric_limits<double>::quiet_NaN();
rclcpp::Logger LOGGER = rclcpp::get_logger("FDRightEeController");
} // namespace

// ============================================================================
// on_init：只声明参数 + 打印日志，不给成员矩阵赋值
// ============================================================================
controller_interface::CallbackReturn FDRightEeController::on_init() {
    try {
        if (!get_node()->has_parameter("joints")) {
            get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
        }
        if (!get_node()->has_parameter("transform_translation")) {
            get_node()->declare_parameter<std::vector<double>>("transform_translation", std::vector<double>{});
        }
        if (!get_node()->has_parameter("transform_rotation")) {
            get_node()->declare_parameter<std::vector<double>>("transform_rotation", std::vector<double>{});
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "on_init 异常: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// on_configure：读参数 → 算固定变换
// ============================================================================
controller_interface::CallbackReturn FDRightEeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    joints_name_ = get_node()->get_parameter("joints").as_string_array();

    if (joints_name_.size() < 7) {
        RCLCPP_ERROR(LOGGER,
                     "YAML 参数 joints 项数不足：需要 7 项（依次为 x,y,z,roll,pitch,yaw,gripper），"
                     "当前只有 %zu 项。请检查控制器 YAML 中 ros__parameters.joints 的配置",
                     joints_name_.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < 7; ++i) {
        if (joints_name_[i].empty()) {
            RCLCPP_ERROR(LOGGER, "joints[%zu] 为空字符串", i);
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    const auto transform_translation_param = get_node()->get_parameter("transform_translation").as_double_array();
    const auto transform_rotation_param = get_node()->get_parameter("transform_rotation").as_double_array();

    Eigen::Quaternion<double> q;
    Eigen::Vector3d trans;

    if (transform_translation_param.empty()) {
        trans << 0.0, 0.0, 0.0;
    } else if (transform_translation_param.size() == 3) {
        trans << transform_translation_param[0], transform_translation_param[1], transform_translation_param[2];
    } else {
        RCLCPP_ERROR(LOGGER, "平移格式错误");
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
        q = Eigen::Quaternion<double>(transform_rotation_param[0], transform_rotation_param[1], transform_rotation_param[2],
                                      transform_rotation_param[3]);
    } else {
        RCLCPP_ERROR(LOGGER, "旋转格式错误：仅支持 RPY 或四元数");
        return controller_interface::CallbackReturn::ERROR;
    }

    q.normalize();

    transform_ = Eigen::Matrix4d::Identity();
    pose_ = Eigen::Matrix4d::Identity();
    transform_.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform_.block<3, 1>(0, 3) = trans;

    // 一次性分配，之后只 fill，不 clear，保证导出指针稳定
    reference_interfaces_.assign(REF_SIZE, qNaN);
    state_index_.clear();

    RCLCPP_INFO(LOGGER, "FDRightEeController 配置完成");
    return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 接口配置
// ============================================================================
controller_interface::InterfaceConfiguration FDRightEeController::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FDRightEeController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.reserve(joints_name_.size());
    for (const auto &joint : joints_name_) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

// ============================================================================
// 按名绑定状态接口下标
// ============================================================================
bool FDRightEeController::bind_state_indices() {
    state_index_.assign(joints_name_.size(), -1);

    for (size_t j = 0; j < joints_name_.size(); ++j) {
        const auto &target = joints_name_[j];
        bool found = false;
        for (size_t i = 0; i < state_interfaces_.size(); ++i) {
            const auto &iface = state_interfaces_[i];
            if (iface.get_prefix_name() == target && iface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
                state_index_[j] = static_cast<int>(i);
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_ERROR(LOGGER, "关节 %s 的 position 状态接口未找到", target.c_str());
            return false;
        }
    }
    return true;
}

// ============================================================================
// 生命周期
// ============================================================================
controller_interface::CallbackReturn FDRightEeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    if (!bind_state_indices()) {
        // 绑定失败属于配置错误，不可恢复
        return controller_interface::CallbackReturn::ERROR;
    }
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), qNaN);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FDRightEeController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // 清空绑定，重新激活时 on_activate 会重新 bind
    std::fill(state_index_.begin(), state_index_.end(), -1);
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), qNaN);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FDRightEeController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    joints_name_.clear();
    transform_ = Eigen::Matrix4d::Identity();
    pose_ = Eigen::Matrix4d::Identity();
    // 注意：不要 clear reference_interfaces_，否则已导出的 CommandInterface 指针会悬空；
    // 只重置值，保留容量与地址稳定。
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), qNaN);
    state_index_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// update
// ============================================================================
controller_interface::return_type FDRightEeController::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                                                         const rclcpp::Duration & /*period*/) {
    return controller_interface::return_type::OK;
}

controller_interface::return_type FDRightEeController::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                                                 const rclcpp::Duration & /*period*/) {
    auto logger = LOGGER;

    auto read = [&](size_t j) -> double {
        if (j >= state_index_.size()) {
            return qNaN;
        }
        const int idx = state_index_[j];
        if (idx < 0) {
            return qNaN;
        }
        const auto opt = state_interfaces_[static_cast<size_t>(idx)].get_optional();
        return opt.value_or(qNaN);
    };

    // ---- 位置 joints[0..2] ----
    const double p_x = read(0);
    const double p_y = read(1);
    const double p_z = read(2);
    if (std::isnan(p_x) || std::isnan(p_y) || std::isnan(p_z)) {
        RCLCPP_DEBUG_THROTTLE(logger, *get_node()->get_clock(), 1000, "fd_right 位置数据无效（校准中或读失败），保持引用接口为 NaN");
        return controller_interface::return_type::OK;
    }

    // ---- 姿态 joints[3..5] ----
    const double roll = read(3);
    const double pitch = read(4);
    const double yaw = read(5);
    if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
        RCLCPP_DEBUG_THROTTLE(logger, *get_node()->get_clock(), 1000, "fd_right 姿态数据无效（校准中或读失败），保持引用接口为 NaN");
        return controller_interface::return_type::OK;
    }

    // ---- 组装 pose ----
    pose_ = Eigen::Matrix4d::Identity();
    pose_(0, 3) = p_x;
    pose_(1, 3) = p_y;
    pose_(2, 3) = p_z;

    // 约定：R = Rx(roll) * Ry(pitch) * Rz(yaw)，固定轴 XYZ 外旋
    Eigen::Quaterniond q_rot = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    pose_.block<3, 3>(0, 0) = q_rot.normalized().toRotationMatrix();

    // 应用外参
    pose_ = transform_ * pose_;

    Eigen::Quaterniond q_out(pose_.block<3, 3>(0, 0));
    q_out.normalize();

    // ---- 写引用接口 ----
    reference_interfaces_[EE_X] = pose_(0, 3);
    reference_interfaces_[EE_Y] = pose_(1, 3);
    reference_interfaces_[EE_Z] = pose_(2, 3);
    reference_interfaces_[EE_QW] = q_out.w();
    reference_interfaces_[EE_QX] = q_out.x();
    reference_interfaces_[EE_QY] = q_out.y();
    reference_interfaces_[EE_QZ] = q_out.z();

    // TODO: 当前 EE_ROLL/PITCH/YAW 为变换前的原始 rpy，与 EE_Q*（经过 transform_ 后的姿态）
    // 语义不一致；后续迭代可统一改为从 pose_ 解算的欧拉角（注意保持 XYZ 外旋约定）。
    reference_interfaces_[EE_ROLL] = roll;
    reference_interfaces_[EE_PITCH] = pitch;
    reference_interfaces_[EE_YAW] = yaw;

    // ---- 夹爪 joints[6]：透传，不影响位姿 ----
    reference_interfaces_[EE_GRIPPER] = read(6);

    return controller_interface::return_type::OK;
}

// ============================================================================
// 导出引用接口
// ============================================================================
std::vector<hardware_interface::CommandInterface::SharedPtr> FDRightEeController::on_export_reference_interfaces_list() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces;
    reference_interfaces.reserve(REF_SIZE);

    const std::string &ctrl_name = get_node()->get_name();

    auto make_ci = [&](const std::string &if_name, double *ptr) {
        return std::make_shared<hardware_interface::CommandInterface>(ctrl_name, if_name, ptr);
    };

    reference_interfaces.push_back(make_ci("ee/x", &reference_interfaces_[EE_X]));
    reference_interfaces.push_back(make_ci("ee/y", &reference_interfaces_[EE_Y]));
    reference_interfaces.push_back(make_ci("ee/z", &reference_interfaces_[EE_Z]));
    reference_interfaces.push_back(make_ci("ee/qw", &reference_interfaces_[EE_QW]));
    reference_interfaces.push_back(make_ci("ee/qx", &reference_interfaces_[EE_QX]));
    reference_interfaces.push_back(make_ci("ee/qy", &reference_interfaces_[EE_QY]));
    reference_interfaces.push_back(make_ci("ee/qz", &reference_interfaces_[EE_QZ]));
    reference_interfaces.push_back(make_ci("ee/roll", &reference_interfaces_[EE_ROLL]));
    reference_interfaces.push_back(make_ci("ee/pitch", &reference_interfaces_[EE_PITCH]));
    reference_interfaces.push_back(make_ci("ee/yaw", &reference_interfaces_[EE_YAW]));
    reference_interfaces.push_back(make_ci("ee/gripper", &reference_interfaces_[EE_GRIPPER]));

    return reference_interfaces;
}

} // namespace mtm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mtm_controllers::FDRightEeController, controller_interface::ChainableControllerInterface)