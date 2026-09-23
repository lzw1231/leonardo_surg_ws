#include "psm_controllers/ft3215/ft3215_controller.hpp"

#include <algorithm>
#include <cmath>

namespace psm_controllers {

// ============================================================================
// on_init
// ============================================================================
controller_interface::CallbackReturn FT3215Controller::on_init() {
    try {
        if (!get_node()->has_parameter("joints")) {
            get_node()->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
        }
        if (!get_node()->has_parameter("interface_name")) {
            get_node()->declare_parameter<std::string>("interface_name", std::string{"position"});
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

// ============================================================================
// on_configure
// ============================================================================
controller_interface::CallbackReturn
FT3215Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    interface_name_ = get_node()->get_parameter("interface_name").as_string();
    upstream_prefix_ = get_node()->get_parameter("upstream_prefix").as_string();

    if (joint_names_.size() != kN) {
        RCLCPP_ERROR(get_node()->get_logger(), "joints 参数长度必须为 %zu，当前为 %zu", kN, joint_names_.size());
        return CallbackReturn::ERROR;
    }
    if (upstream_prefix_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "upstream_prefix 参数不能为空");
        return CallbackReturn::ERROR;
    }

    // 拼上游 6 个数据接口完整名
    upstream_interfaces_.clear();
    upstream_interfaces_.reserve(kN);
    for (const auto *suffix : kUpstreamSuffixes) {
        upstream_interfaces_.push_back(upstream_prefix_ + "/" + suffix);
    }

    RCLCPP_INFO(get_node()->get_logger(), "FT3215Controller 配置完成，upstream_prefix=%s，joints=%zu，kScalePos=%.2f",
                upstream_prefix_.c_str(), joint_names_.size(), kScalePos);
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// 接口配置
// ============================================================================
controller_interface::InterfaceConfiguration FT3215Controller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // 前 6：上游数据接口
    for (const auto &name : upstream_interfaces_) {
        config.names.push_back(name);
    }
    // 第 7：上游 epoch
    config.names.push_back(upstream_prefix_ + "/epoch");
    // 后 6：从手硬件命令接口
    for (const auto &joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
}

controller_interface::InterfaceConfiguration FT3215Controller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // 从手硬件位置（用于 epoch 变化时建立基准）
    for (const auto &joint_name : joint_names_) {
        config.names.push_back(joint_name + "/" + interface_name_);
    }
    return config;
}

// ============================================================================
// 生命周期
// ============================================================================
controller_interface::CallbackReturn FT3215Controller::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    const size_t expected_cmd = kN + 1 + kN; // 6 上游 + 1 epoch + 6 从手
    if (command_interfaces_.size() != expected_cmd) {
        RCLCPP_ERROR(get_node()->get_logger(), "command 接口数量不符：期望 %zu，实际 %zu", expected_cmd,
                     command_interfaces_.size());
        return CallbackReturn::ERROR;
    }
    if (state_interfaces_.size() != kN) {
        RCLCPP_ERROR(get_node()->get_logger(), "state 接口数量不符：期望 %zu，实际 %zu", kN, state_interfaces_.size());
        return CallbackReturn::ERROR;
    }

    initialized_ = false;
    last_epoch_ = -1.0;
    slave_cmd_.fill(0.0);
    last_target_.fill(0.0);

    RCLCPP_INFO(get_node()->get_logger(), "FT3215Controller 已激活");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
FT3215Controller::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FT3215Controller::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    joint_names_.clear();
    upstream_interfaces_.clear();
    upstream_prefix_.clear();
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// update：epoch 检测 + 增量累加
// ============================================================================
controller_interface::return_type FT3215Controller::update(const rclcpp::Time & /*time*/,
                                                           const rclcpp::Duration & /*period*/) {
    // ---- 1. 读 epoch ----
    const auto ep_opt = command_interfaces_[kEpochCmdIdx].get_optional();
    if (!ep_opt.has_value() || !std::isfinite(ep_opt.value())) {
        return controller_interface::return_type::OK;
    }
    const double ep = ep_opt.value();

    // ---- 2. 判断是否需要重置基准 ----
    const bool need_reset = (!initialized_) || (ep != last_epoch_);

    // ---- 3. 重置时，确认所有轴的上游值都有效 ----
    if (need_reset) {
        bool all_valid = true;
        for (size_t i = 0; i < kN; ++i) {
            const auto ref_opt = command_interfaces_[i].get_optional();
            if (!ref_opt.has_value() || !std::isfinite(ref_opt.value())) {
                all_valid = false;
                break;
            }
        }
        if (!all_valid) {
            return controller_interface::return_type::OK;
        }
    }

    // ---- 4. 逐轴处理 ----
    for (size_t i = 0; i < kN; ++i) {
        const auto ref_opt = command_interfaces_[i].get_optional();
        if (!ref_opt.has_value() || !std::isfinite(ref_opt.value())) {
            continue;
        }
        const double target = ref_opt.value();

        if (need_reset) {
            // 重置：从硬件读当前位置作为从手起始
            const auto hw_opt = state_interfaces_[i].get_optional();
            slave_cmd_[i] = (hw_opt.has_value() && std::isfinite(hw_opt.value())) ? hw_opt.value() : 0.0;
            last_target_[i] = target;
        } else {
            // 正常增量：主手动多少，从手跟多少（位置轴放大 kScale[i] 倍）
            const double delta = target - last_target_[i];
            slave_cmd_[i] += delta * kScale[i];
            last_target_[i] = target;
        }

        // 写从手命令
        if (!command_interfaces_[kSlaveCmdStart + i].set_value(slave_cmd_[i])) {
            RCLCPP_ERROR(get_node()->get_logger(), "写 command 失败：%s", joint_names_[i].c_str());
            return controller_interface::return_type::ERROR;
        }
    }

    // ---- 5. 更新状态 ----
    if (need_reset) {
        initialized_ = true;
        last_epoch_ = ep;
        RCLCPP_INFO(get_node()->get_logger(), "基准已重置，epoch=%f", ep);
    }

    return controller_interface::return_type::OK;
}

} // namespace psm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::FT3215Controller, controller_interface::ControllerInterface)