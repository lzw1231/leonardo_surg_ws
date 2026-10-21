#include "psm_controllers/aloe_controller/aloe_controller.hpp"

#include <cmath>
#include <limits>

namespace psm_controllers {

namespace {
rclcpp::Logger LOGGER = rclcpp::get_logger("AloeController");
const double qNaN = std::numeric_limits<double>::quiet_NaN();
} // namespace

// ============================================================================
// 接口声明
// ============================================================================
controller_interface::InterfaceConfiguration AloeController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = command_interface_names_; // 51 个：s1/s2/s3 各 17
    return config;
}

controller_interface::InterfaceConfiguration AloeController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = state_interfaces_names_; // 45 个：4 臂 × 11 + 1 epoch
    return config;
}

// ============================================================================
// on_init
// ============================================================================
controller_interface::CallbackReturn AloeController::on_init() {
    try {
        auto node = get_node();
        for (const auto *name : kPrefixParamNames) {
            if (!node->has_parameter(name)) {
                node->declare_parameter<std::string>(name, "");
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "on_init 异常: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_configure
// ============================================================================
controller_interface::CallbackReturn AloeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    auto node = get_node();

    // ---- 1. 读四臂上游前缀（state 侧需要 ecm1）----
    for (size_t a = 0; a < kArmsState; ++a) {
        prefixes_[a] = node->get_parameter(kPrefixParamNames[a]).as_string();
        if (prefixes_[a].empty()) {
            RCLCPP_ERROR(LOGGER, "YAML %s 不能为空", kPrefixParamNames[a]);
            return CallbackReturn::ERROR;
        }
    }

    // ---- 2. state 接口名：4 臂 × 11 + 1 epoch ----
    state_interfaces_names_.clear();
    state_interfaces_names_.reserve(kStateSize);
    for (size_t a = 0; a < kArmsState; ++a) {
        for (const auto *suffix : kSuffixes) {
            state_interfaces_names_.push_back(prefixes_[a] + "/" + suffix);
        }
    }
    {
        const auto &p = prefixes_[0];
        const auto pos = p.rfind('/');
        state_interfaces_names_.push_back((pos != std::string::npos) ? p.substr(0, pos) + "/epoch" : "epoch");
    }
    if (state_interfaces_names_.size() != kStateSize) {
        RCLCPP_ERROR(LOGGER, "state 接口数量错误 %zu != %zu", state_interfaces_names_.size(), kStateSize);
        return CallbackReturn::ERROR;
    }

    // ---- 3. command 接口名：3 臂 × 17 ----
    command_interface_names_.clear();
    command_interface_names_.reserve(kCommandSize);
    for (size_t a = 0; a < kArmsCommand; ++a) {
        for (const auto *suffix : kArmCommandSuffix) {
            command_interface_names_.push_back(std::string(kCmdPrefixes[a]) + suffix);
        }
    }
    if (command_interface_names_.size() != kCommandSize) {
        RCLCPP_ERROR(LOGGER, "command 接口数量错误 %zu != %zu", command_interface_names_.size(), kCommandSize);
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(LOGGER, "AloeController 配置完成");
    for (size_t a = 0; a < kArmsState; ++a) {
        RCLCPP_INFO(LOGGER, "  %s: %s", kPrefixParamNames[a], prefixes_[a].c_str());
    }
    RCLCPP_INFO(LOGGER, "  state 接口 %zu 个, command 接口 %zu 个", state_interfaces_names_.size(), command_interface_names_.size());
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_activate
// ============================================================================
controller_interface::CallbackReturn AloeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // ---- 1. 数量校验 ----
    if (state_interfaces_.size() != kStateSize) {
        RCLCPP_ERROR(LOGGER, "state 接口数量不符：期望 %zu，实际 %zu", kStateSize, state_interfaces_.size());
        return CallbackReturn::ERROR;
    }
    if (command_interfaces_.size() != kCommandSize) {
        RCLCPP_ERROR(LOGGER, "command 接口数量不符：期望 %zu，实际 %zu", kCommandSize, command_interfaces_.size());
        return CallbackReturn::ERROR;
    }

    // ---- 2. command 名字逐一校验 ----
    for (size_t i = 0; i < kCommandSize; ++i) {
        if (command_interfaces_[i].get_name() != command_interface_names_[i]) {
            RCLCPP_ERROR(LOGGER, "command 接口名不符 idx=%zu expect=%s actual=%s", i, command_interface_names_[i].c_str(),
                         command_interfaces_[i].get_name().c_str());
            return CallbackReturn::ERROR;
        }
    }

    // ---- 3. command 全部置 qNaN（安全位），基准同步 qNaN ----
    for (auto &ci : command_interfaces_) {
        (void)ci.set_value(qNaN);
    }
    for (auto *rt : arms_) {
        rt->master_ref_ci.fill(qNaN);
        rt->slave_ref_ci.fill(qNaN);
        rt->last_cmd_ci.fill(qNaN);
    }

    // ---- 4. 重置基准 / epoch / state ----
    ref_ready_ = false;
    last_epoch_ = std::numeric_limits<double>::quiet_NaN();
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(LOGGER, "AloeController 已激活：state %zu, command %zu", state_interfaces_.size(), command_interfaces_.size());
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// update
// ============================================================================
controller_interface::return_type AloeController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // ---- 1. 读上游 state，非法值填 NaN ----
    for (size_t i = 0; i < kStateSize; ++i) {
        const auto opt = state_interfaces_[i].get_optional();
        upstream_state_[i] = (opt.has_value() && std::isfinite(opt.value())) ? opt.value() : qNaN;
    }

    // ---- 2. 打印验证（1 秒一次，避免刷屏）----
    auto S = [this](size_t arm, size_t field) { return upstream_state_[arm * kUpstreamPerArm + field]; };
    // field: 0=x 1=y 2=z 3=qw 4=qx 5=qy 6=qz 7=roll 8=pitch 9=yaw 10=gripper
    const double epoch = upstream_state_[kArmsState * kUpstreamPerArm];

    RCLCPP_INFO_THROTTLE(LOGGER, *get_node()->get_clock(), 1000,
                         "psm1 x=%.3f y=%.3f z=%.3f qw=%.3f roll=%.3f gripper=%.3f | "
                         "psm2 x=%.3f y=%.3f z=%.3f qw=%.3f roll=%.3f gripper=%.3f | "
                         "psm3 x=%.3f y=%.3f z=%.3f qw=%.3f roll=%.3f gripper=%.3f | "
                         "ecm1 x=%.3f y=%.3f z=%.3f qw=%.3f roll=%.3f gripper=%.3f | epoch=%.0f",
                         S(0, 0), S(0, 1), S(0, 2), S(0, 3), S(0, 7), S(0, 10), S(1, 0), S(1, 1), S(1, 2), S(1, 3), S(1, 7), S(1, 10),
                         S(2, 0), S(2, 1), S(2, 2), S(2, 3), S(2, 7), S(2, 10), S(3, 0), S(3, 1), S(3, 2), S(3, 3), S(3, 7), S(3, 10),
                         epoch);

    return controller_interface::return_type::OK;
}

// ============================================================================
// on_deactivate
// ============================================================================
controller_interface::CallbackReturn AloeController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {

    for (auto &ci : command_interfaces_) {
        (void)ci.set_value(qNaN);
    }
    for (auto *rt : arms_) {
        rt->master_ref_ci.fill(qNaN);
        rt->slave_ref_ci.fill(qNaN);
        rt->last_cmd_ci.fill(qNaN);
    }

    ref_ready_ = false;
    last_epoch_ = std::numeric_limits<double>::quiet_NaN();
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// on_cleanup（待实现）
// ============================================================================
controller_interface::CallbackReturn AloeController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    state_interfaces_names_.clear();
    command_interface_names_.clear();
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());
    ref_ready_ = false;
    last_epoch_ = std::numeric_limits<double>::quiet_NaN();
    return CallbackReturn::SUCCESS;
}

} // namespace psm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::AloeController, controller_interface::ControllerInterface)