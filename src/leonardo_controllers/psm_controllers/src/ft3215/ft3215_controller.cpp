#include "psm_controllers/ft3215/ft3215_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace psm_controllers {

controller_interface::CallbackReturn FT3215Controller::on_init() {
    try {
        // ---- 声明参数（若已由 --params-file 作为 override 传入，则沿用其值）----
        if (!get_node()->has_parameter("joints")) {
            get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
        }
        if (!get_node()->has_parameter("interface_name")) {
            get_node()->declare_parameter<std::string>("interface_name", std::string{"position"});
        }
        if (!get_node()->has_parameter("coefficient")) {
            get_node()->declare_parameter<double>("coefficient", double{1.0});
        }
        if (!get_node()->has_parameter("upstream_prefix")) {
            get_node()->declare_parameter<std::string>("upstream_prefix", std::string{});
        }

    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init 异常: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FT3215Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {

    // ---- 读取最终值 ----
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    interface_name_ = get_node()->get_parameter("interface_name").as_string();
    coefficient_ = get_node()->get_parameter("coefficient").as_double();
    upstream_prefix_ = get_node()->get_parameter("upstream_prefix").as_string();

    if (joint_names_.size() != 6) {
        RCLCPP_ERROR(get_node()->get_logger(), "joints 参数长度必须为 6，当前为 %zu", joint_names_.size());
        return CallbackReturn::ERROR;
    }
    if (upstream_prefix_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "upstream_prefix 参数不能为空");
        return CallbackReturn::ERROR;
    }
    if (coefficient_ < 0.0 || coefficient_ > 1.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "coefficient 必须在 [0, 1] 区间，当前为 %f", coefficient_);
        return CallbackReturn::ERROR;
    }

    // 拼上游 6 个接口完整名
    upstream_interfaces_.clear();
    upstream_interfaces_.reserve(6);
    for (const auto *suffix : kUpstreamSuffixes) {
        upstream_interfaces_.push_back(upstream_prefix_ + "/" + suffix);
    }

    RCLCPP_INFO(get_node()->get_logger(), "FT3215Controller 配置完成，upstream_prefix=%s，joints=%zu",
                upstream_prefix_.c_str(), joint_names_.size());
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FT3215Controller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // 前 6：上游链式控制器导出的 reference interfaces（command 类型）
    for (const auto &name : upstream_interfaces_) {
        config.names.push_back(name);
    }
    // 后 6：从手硬件命令接口
    for (const auto &joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
}

controller_interface::InterfaceConfiguration FT3215Controller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // 只有从手硬件位置（用于平滑）
    for (const auto &joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
}

controller_interface::CallbackReturn FT3215Controller::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    const size_t n = joint_names_.size();
    const size_t expected_cmd = upstream_interfaces_.size() + n;
    if (command_interfaces_.size() != expected_cmd) {
        RCLCPP_ERROR(get_node()->get_logger(), "command 接口数量不符：期望 %zu，实际 %zu", expected_cmd,
                     command_interfaces_.size());
        return CallbackReturn::ERROR;
    }
    if (state_interfaces_.size() != n) {
        RCLCPP_ERROR(get_node()->get_logger(), "state 接口数量不符：期望 %zu，实际 %zu", n, state_interfaces_.size());
        return CallbackReturn::ERROR;
    }

    // 预置从手命令为当前位置
    for (size_t i = 0; i < n; ++i) {
        const auto hw_opt = state_interfaces_[i].get_optional();
        if (!hw_opt.has_value() || !std::isfinite(hw_opt.value())) {
            RCLCPP_WARN(get_node()->get_logger(), "从手位置无效：%s", joint_names_[i].c_str());
            continue;
        }
        if (!command_interfaces_[n + i].set_value(hw_opt.value())) {
            RCLCPP_ERROR(get_node()->get_logger(), "预置 command 失败：%s", joint_names_[i].c_str());
            return CallbackReturn::ERROR;
        }
    }

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FT3215Controller::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // 链末端，不需要额外处理。硬件侧的保持由硬件接口负责。
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FT3215Controller::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    joint_names_.clear();
    upstream_interfaces_.clear();
    upstream_prefix_.clear();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type FT3215Controller::update(const rclcpp::Time & /*time*/,
                                                           const rclcpp::Duration & /*period*/) {
    const size_t n = joint_names_.size();

    for (size_t i = 0; i < n; ++i) {
        // 上游参考值：从 command 接口的前 6 个读
        const auto ref_opt = command_interfaces_[i].get_optional();
        if (!ref_opt.has_value() || !std::isfinite(ref_opt.value())) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "上游接口 %s 无效",
                                 upstream_interfaces_[i].c_str());
            continue;
        }
        const double target = ref_opt.value();

        // 从手硬件位置：从 state 接口读
        const auto hw_opt = state_interfaces_[i].get_optional();
        if (!hw_opt.has_value() || !std::isfinite(hw_opt.value())) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "从手位置 %s 无效",
                                 joint_names_[i].c_str());
            continue;
        }
        const double current = hw_opt.value();

        const double new_cmd = target * coefficient_ + current * (1.0 - coefficient_);

        // 写从手命令：command 接口的后 6 个
        if (!command_interfaces_[n + i].set_value(new_cmd)) {
            RCLCPP_ERROR(get_node()->get_logger(), "写 command 失败：%s", joint_names_[i].c_str());
            return controller_interface::return_type::ERROR;
        }
    }

    return controller_interface::return_type::OK;
}

} // namespace psm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::FT3215Controller, controller_interface::ControllerInterface)