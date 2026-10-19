#include "psm_controllers/intelligent_can/intelligent_can0_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace psm_controllers {

// ============================================================================
// on_init：声明参数（psm1_prefix、psm2_prefix），不做业务校验
//
// 生命周期：控制器创建后的第一个钩子。
// 职责：确保参数在 node 上存在且类型正确；配置有效性（判空、拼接口名）留给 on_configure。
// ============================================================================
controller_interface::CallbackReturn IntelligentCan0Controller::on_init() {
    try {
        for (size_t i = 0; i < kArms; ++i) {
            // 防重复声明：YAML 预填时 has_parameter 为 true，跳过 declare 保留 YAML 值
            if (!get_node()->has_parameter(kPrefixParamNames[i])) {
                // 默认空串：缺失由 on_configure 判空拦截，不在此处兜底
                get_node()->declare_parameter<std::string>(kPrefixParamNames[i], std::string{});
            }
        }
    } catch (const std::exception &e) {
        // 捕获参数名非法、类型冲突等异常，返回 ERROR 拒绝加载
        RCLCPP_ERROR(get_node()->get_logger(), "on_init 异常: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_configure：读参数 + 拼 24 个 state 接口名
//
// 生命周期：on_init 成功后调用，可重复（cleanup 后可再 configure）。
// 职责：读取 psm1_prefix / psm2_prefix 并判空，按 StateIdx 顺序拼出完整接口名。
// ============================================================================
controller_interface::CallbackReturn IntelligentCan0Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    auto node = get_node();

    psm1_prefix_ = node->get_parameter("psm1_prefix").as_string();
    psm2_prefix_ = node->get_parameter("psm2_prefix").as_string();

    if (psm1_prefix_.empty() || psm2_prefix_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "YAMAL 中的 psm1_prefix / psm2_prefix 不能为空");
        return CallbackReturn::ERROR;
    }

    // 拼接口名，顺序与 StateIdx 严格一致：
    //   [0, 11) : psm_1 的 11 个数据
    //   [11, 22): psm_2 的 11 个数据
    //   [22]    : psm_1/epoch
    //   [23]    : psm_2/epoch
    state_interfaces_names_.clear();
    state_interfaces_names_.reserve(STATE_SIZE);

    for (size_t i = 0; i < kUpstreamPerArm; ++i) {
        state_interfaces_names_.push_back(psm1_prefix_ + "/" + kSuffixes[i]);
    }
    for (size_t i = 0; i < kUpstreamPerArm; ++i) {
        state_interfaces_names_.push_back(psm2_prefix_ + "/" + kSuffixes[i]);
    }
    state_interfaces_names_.push_back(psm1_prefix_ + "/epoch");
    state_interfaces_names_.push_back(psm2_prefix_ + "/epoch");

    // 保险：数量必须等于 STATE_SIZE
    if (state_interfaces_names_.size() != STATE_SIZE) {
        RCLCPP_ERROR(node->get_logger(), "接口名数量错误：%zu != %zu", state_interfaces_names_.size(), STATE_SIZE);
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(node->get_logger(), "IntelligentCan0Controller 配置完成");
    RCLCPP_INFO(node->get_logger(), "  psm_1 上游: %s", psm1_prefix_.c_str());
    RCLCPP_INFO(node->get_logger(), "  psm_2 上游: %s", psm2_prefix_.c_str());
    RCLCPP_INFO(node->get_logger(), "  订阅 state 接口 %zu 个", state_interfaces_names_.size());
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// 接口配置
// ============================================================================
controller_interface::InterfaceConfiguration IntelligentCan0Controller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = state_interfaces_names_;
    return config;
}

controller_interface::InterfaceConfiguration IntelligentCan0Controller::command_interface_configuration() const {
    // 当前阶段：command 未接入
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

// ============================================================================
// 生命周期
// ============================================================================
controller_interface::CallbackReturn IntelligentCan0Controller::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    if (state_interfaces_.size() != STATE_SIZE) {
        RCLCPP_ERROR(get_node()->get_logger(), "state 接口数量不符：期望 %zu，实际 %zu", STATE_SIZE, state_interfaces_.size());
        return CallbackReturn::ERROR;
    }

    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());
    RCLCPP_INFO(get_node()->get_logger(), "IntelligentCan0Controller 已激活");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan0Controller::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan0Controller::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    state_interfaces_names_.clear();
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// update：只读 24 个 state 到缓存，映射计算后续接入
// ============================================================================
controller_interface::return_type IntelligentCan0Controller::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    for (size_t i = 0; i < STATE_SIZE; ++i) {
        const auto opt = state_interfaces_[i].get_optional();
        upstream_state_[i] = (opt.has_value() && std::isfinite(opt.value())) ? opt.value() : std::numeric_limits<double>::quiet_NaN();
    }

    // TODO: 映射计算 —— 由 upstream_state_ 算出从手命令
    return controller_interface::return_type::OK;
}

} // namespace psm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::IntelligentCan0Controller, controller_interface::ControllerInterface)