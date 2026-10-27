#include "mtm_controllers/fd_left/fd_left_ee_controller.hpp"

#include <algorithm>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <rclcpp/logging.hpp>

namespace mtm_controllers {

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
rclcpp::Logger LOGGER = rclcpp::get_logger("FDLeftEeController");
} // namespace

controller_interface::CallbackReturn FDLeftEeController::on_init() {
    try {
        // ---- joints ----
        if (!get_node()->has_parameter("joints")) {
            get_node()->declare_parameter<std::vector<std::string>>("joints", {});
        }
        joints_name_ = get_node()->get_parameter("joints").as_string_array();

        if (joints_name_.size() < 7) {
            RCLCPP_ERROR(LOGGER, "joints 参数至少需要 7 个关节（x,y,z,roll,pitch,yaw,gripper），当前 %zu 个", joints_name_.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        // ---- transform_translation / transform_rotation ----
        std::vector<double> t_trans = {0.0, 0.0, 0.0};
        std::vector<double> t_rot = {0.0, 0.0, 0.0};
        get_node()->get_parameter_or<std::vector<double>>("transform_translation", t_trans, t_trans);
        get_node()->get_parameter_or<std::vector<double>>("transform_rotation", t_rot, t_rot);

        if (t_trans.size() != 3 || t_rot.size() != 3) {
            RCLCPP_ERROR(LOGGER, "transform_translation / transform_rotation 必须是 3 元素数组");
            return controller_interface::CallbackReturn::ERROR;
        }

        transform_ = Eigen::Matrix4d::Identity();
        Eigen::Quaterniond q_rot = Eigen::AngleAxisd(t_rot[0], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(t_rot[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(t_rot[2], Eigen::Vector3d::UnitZ());
        transform_.block<3, 3>(0, 0) = q_rot.normalized().toRotationMatrix();
        transform_(0, 3) = t_trans[0];
        transform_(1, 3) = t_trans[1];
        transform_(2, 3) = t_trans[2];

        // ---- 引用接口存储：一次性分配，之后只 fill，不 resize，保证指针稳定 ----
        reference_interfaces_.assign(REF_SIZE, kNaN);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "on_init 失败: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FDLeftEeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    // joints_name_ 已在 on_init 中读取并校验数量
    for (size_t i = 0; i < 7; ++i) {
        if (joints_name_[i].empty()) {
            RCLCPP_ERROR(LOGGER, "joints[%zu] 为空字符串", i);
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // 重置导出值，避免下游读到 0
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), kNaN);
    state_index_.clear();

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FDLeftEeController::command_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration FDLeftEeController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.reserve(joints_name_.size());
    for (const auto &name : joints_name_) {
        cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    }
    return cfg;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> FDLeftEeController::on_export_reference_interfaces_list() {
    static constexpr const char *kNames[REF_SIZE] = {
        "ee/x", "ee/y", "ee/z", "ee/qw", "ee/qx", "ee/qy", "ee/qz", "ee/roll", "ee/pitch", "ee/yaw", "ee/gripper",
    };

    std::vector<hardware_interface::CommandInterface::SharedPtr> out;
    out.reserve(REF_SIZE);
    for (size_t i = 0; i < REF_SIZE; ++i) {
        out.emplace_back(
            std::make_shared<hardware_interface::CommandInterface>(get_node()->get_name(), kNames[i], &reference_interfaces_[i]));
    }
    return out;
}

bool FDLeftEeController::bind_state_indices() {
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

controller_interface::CallbackReturn FDLeftEeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    if (!bind_state_indices()) {
        // 绑定失败属于配置错误，不可恢复
        return controller_interface::CallbackReturn::ERROR;
    }
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), kNaN);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FDLeftEeController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // 清空绑定，重新激活时 on_activate 会重新 bind
    std::fill(state_index_.begin(), state_index_.end(), -1);
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), kNaN);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FDLeftEeController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    state_index_.clear();
    // reference_interfaces_ 不 clear，只重置值，保持导出指针稳定
    std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), kNaN);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type FDLeftEeController::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                                                        const rclcpp::Duration & /*period*/) {
    // 本控制器无订阅者命令来源，保持引用接口由 update_and_write_commands 写
    return controller_interface::return_type::OK;
}

controller_interface::return_type FDLeftEeController::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                                                const rclcpp::Duration & /*period*/) {
    auto read = [&](size_t j) -> double {
        if (j >= state_index_.size()) {
            return kNaN;
        }
        const int idx = state_index_[j];
        if (idx < 0) {
            return kNaN;
        }
        const auto opt = state_interfaces_[static_cast<size_t>(idx)].get_optional();
        return opt.value_or(kNaN);
    };

    // ---- 位置 joints[0..2] ----
    const double p_x = read(0);
    const double p_y = read(1);
    const double p_z = read(2);
    if (std::isnan(p_x) || std::isnan(p_y) || std::isnan(p_z)) {
        RCLCPP_DEBUG_THROTTLE(LOGGER, *get_node()->get_clock(), 1000, "fd_left 位置数据无效（校准中或读失败），保持引用接口为 NaN");
        return controller_interface::return_type::OK;
    }

    // ---- 姿态 joints[3..5] ----
    const double roll = read(3);
    const double pitch = read(4);
    const double yaw = read(5);
    if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
        RCLCPP_DEBUG_THROTTLE(LOGGER, *get_node()->get_clock(), 1000, "fd_left 姿态数据无效（校准中或读失败），保持引用接口为 NaN");
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

} // namespace mtm_controllers