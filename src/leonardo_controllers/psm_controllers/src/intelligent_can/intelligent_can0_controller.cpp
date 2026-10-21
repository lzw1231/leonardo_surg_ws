#include "psm_controllers/intelligent_can/intelligent_can0_controller.hpp"
#include "psm_controllers/common/psm_kinematics.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

namespace {

// q(7) → 11 个接口瞬时值（含符号）；与 kPerArmCommand 前 11 项严格对齐
constexpr size_t kMappedPerArmFn = 11;

std::array<double, kMappedPerArmFn> q_to_mapped_ci(const ::Dof &d) {
    return {
        d.q[0],  // s?_j01_base
        d.q[1],  // s?_j03_oc
        d.q[2],  // s?_j04_oc
        -d.q[2], // s?_j07_oc
        -d.q[1], // s?_j08_oc
        d.q[3],  // s?_j11_oc
        d.q[4],  // s?_j12_oc
        d.q[3],  // s?_j13_oc
        d.q[4],  // s?_j14_oc
        d.q[5],  // s?_j17_18
        d.q[6],  // s?_j20_21
    };
}

} // namespace

namespace psm_controllers {

// ============================================================================
// 生命周期前段
// ============================================================================
controller_interface::CallbackReturn IntelligentCan0Controller::on_init() {
    try {
        auto node = get_node();
        for (size_t i = 0; i < kArms; ++i) {
            if (!node->has_parameter(kPrefixParamNames[i])) {
                node->declare_parameter<std::string>(kPrefixParamNames[i], "");
            }
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_node()->get_logger(), "on_init 异常: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan0Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    auto node = get_node();
    psm1_prefix_ = node->get_parameter("psm1_prefix").as_string();
    psm2_prefix_ = node->get_parameter("psm2_prefix").as_string();

    if (psm1_prefix_.empty() || psm2_prefix_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "YAML psm1_prefix / psm2_prefix 不能为空");
        return CallbackReturn::ERROR;
    }

    // ========= 构建 state 接口名 =========
    state_interfaces_names_.clear();
    state_interfaces_names_.reserve(STATE_SIZE);
    const std::array<std::string, kArms> arm_prefixes{psm1_prefix_, psm2_prefix_};

    for (const auto &prefix : arm_prefixes) {
        for (size_t i = 0; i < kUpstreamPerArm; ++i) {
            state_interfaces_names_.push_back(prefix + "/" + kSuffixes[i]);
        }
    }
    state_interfaces_names_.push_back(psm1_prefix_ + "/epoch");
    state_interfaces_names_.push_back(psm2_prefix_ + "/epoch");

    if (state_interfaces_names_.size() != STATE_SIZE) {
        RCLCPP_ERROR(node->get_logger(), "state接口数量错误 %zu != %zu", state_interfaces_names_.size(), STATE_SIZE);
        return CallbackReturn::ERROR;
    }

    // ========= 构建 command 接口名 =========
    command_interface_names_.clear();
    command_interface_names_.reserve(kCommandSize);
    const std::array<std::string, kArms> cmd_prefixes{"s1", "s2"};
    for (const auto &pre : cmd_prefixes) {
        for (size_t i = 0; i < kPerArmCommand; ++i) {
            command_interface_names_.push_back(pre + kArmCommandSuffix[i]);
        }
    }

    RCLCPP_INFO(node->get_logger(), "IntelligentCan0Controller 配置完成");
    RCLCPP_INFO(node->get_logger(), "  psm_1 上游: %s", psm1_prefix_.c_str());
    RCLCPP_INFO(node->get_logger(), "  psm_2 上游: %s", psm2_prefix_.c_str());
    RCLCPP_INFO(node->get_logger(), "  state接口 %zu 个, command接口 %zu 个", state_interfaces_names_.size(), command_interface_names_.size());
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration IntelligentCan0Controller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = state_interfaces_names_;
    return config;
}

controller_interface::InterfaceConfiguration IntelligentCan0Controller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = command_interface_names_;
    return config;
}

bool IntelligentCan0Controller::validate_command_interfaces(const std::vector<hardware_interface::LoanedCommandInterface> &cmd_if) const {
    auto logger = get_node()->get_logger();
    for (size_t i = 0; i < cmd_if.size(); ++i) {
        if (cmd_if[i].get_name() != command_interface_names_[i]) {
            RCLCPP_ERROR(logger, "Command接口校验失败 idx=%zu, expect=%s, actual=%s", i, command_interface_names_[i].c_str(),
                         cmd_if[i].get_name().c_str());
            return false;
        }
    }
    return true;
}

// ============================================================================
// 生命周期：激活 / 去激活 / 清理
// ============================================================================
controller_interface::CallbackReturn IntelligentCan0Controller::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    auto logger = get_node()->get_logger();
    if (state_interfaces_.size() != STATE_SIZE) {
        RCLCPP_ERROR(logger, "state接口数量不符，期望 %zu，实际 %zu", STATE_SIZE, state_interfaces_.size());
        return CallbackReturn::ERROR;
    }
    if (command_interfaces_.size() != kCommandSize) {
        RCLCPP_ERROR(logger, "command接口数量不符，期望 %zu，实际 %zu", kCommandSize, command_interfaces_.size());
        return CallbackReturn::ERROR;
    }
    if (!validate_command_interfaces(command_interfaces_)) {
        return CallbackReturn::ERROR;
    }

    // 初始安全位 0，并同步 last_cmd 作为从端基准起点
    for (auto &ci : command_interfaces_) {
        (void)ci.set_value(0.0);
    }
    for (size_t a = 0; a < kArms; ++a) {
        last_cmd_ci_[a].fill(0.0);
        master_ref_ci_[a].fill(0.0);
        slave_ref_ci_[a].fill(0.0);
    }

    // 重置基准与 epoch（防止 cleanup→activate 后首帧误触发）
    ref_ready_ = false;
    last_epoch_[0] = last_epoch_[1] = std::numeric_limits<double>::quiet_NaN();
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(logger, "IntelligentCan0Controller 已激活，command接口 %zu 校验通过", command_interfaces_.size());
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan0Controller::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());
    ref_ready_ = false;
    last_epoch_[0] = last_epoch_[1] = std::numeric_limits<double>::quiet_NaN();
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IntelligentCan0Controller::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    state_interfaces_names_.clear();
    command_interface_names_.clear();
    upstream_state_.fill(std::numeric_limits<double>::quiet_NaN());
    ref_ready_ = false;
    last_epoch_[0] = last_epoch_[1] = std::numeric_limits<double>::quiet_NaN();
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// 辅助函数
// ============================================================================
IntelligentCan0Controller::ArmStateSnapshot IntelligentCan0Controller::read_arm_snapshot(size_t arm_idx) const {
    ArmStateSnapshot snap{};
    const size_t base = arm_idx * kUpstreamPerArm;
    const size_t epoch_idx = PSM_1_EPOCH + arm_idx;

    snap.x = upstream_state_[base + 0];
    snap.y = upstream_state_[base + 1];
    snap.z = upstream_state_[base + 2];
    snap.qw = upstream_state_[base + 3];
    snap.qx = upstream_state_[base + 4];
    snap.qy = upstream_state_[base + 5];
    snap.qz = upstream_state_[base + 6];
    snap.roll = upstream_state_[base + 7];
    snap.pitch = upstream_state_[base + 8];
    snap.yaw = upstream_state_[base + 9];
    snap.gripper = upstream_state_[base + 10];
    snap.epoch = upstream_state_[epoch_idx];
    return snap;
}

// ============================================================================
// update：基准式增量 + 踏板刷新 + NaN 保持
// ============================================================================
controller_interface::return_type IntelligentCan0Controller::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    if (command_interfaces_.size() != kCommandSize || state_interfaces_.size() != STATE_SIZE) {
        return controller_interface::return_type::ERROR;
    }
    auto logger = get_node()->get_logger();

    // ---------- 1) 读 state ----------
    for (size_t i = 0; i < STATE_SIZE; ++i) {
        const auto opt = state_interfaces_[i].get_optional();
        upstream_state_[i] = (opt.has_value() && std::isfinite(opt.value())) ? opt.value() : std::numeric_limits<double>::quiet_NaN();
    }
    const auto snap1 = read_arm_snapshot(0);
    const auto snap2 = read_arm_snapshot(1);

    // ---------- 2) 踏板检测（两臂 epoch 应相同，任一有效同步刷两个）----------
    bool c1 = false, c2 = false;
    if (std::isfinite(snap1.epoch) && std::isfinite(last_epoch_[0]) && std::abs(snap1.epoch - last_epoch_[0]) > 0.5) {
        c1 = true;
    }
    if (std::isfinite(snap2.epoch) && std::isfinite(last_epoch_[1]) && std::abs(snap2.epoch - last_epoch_[1]) > 0.5) {
        c2 = true;
    }
    const bool clutch = c1 || c2;

    const double cur_epoch = std::isfinite(snap1.epoch) ? snap1.epoch : snap2.epoch;
    if (std::isfinite(cur_epoch)) {
        last_epoch_[0] = cur_epoch;
        last_epoch_[1] = cur_epoch;
    }

    // ---------- 写接口的统一助手（set_value 返回 bool，true=成功）----------
    auto put = [&](size_t idx, double v) {
        if (!command_interfaces_[idx].set_value(v)) {
            RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000, "写 %s 失败", command_interfaces_[idx].get_name().c_str());
        }
    };

    auto send_all_nan = [&]() {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (auto &ci : command_interfaces_) {
            if (!ci.set_value(nan)) {
                RCLCPP_WARN_THROTTLE(logger, *get_node()->get_clock(), 1000, "写 %s 失败 (NaN)", ci.get_name().c_str());
            }
        }
    };

    // ---------- 3) 计算 q_now ----------
    auto build_post = [](const ArmStateSnapshot &s) -> Post {
        Post p{};
        p.iPosX = s.x;
        p.iPosY = s.y;
        p.iPosZ = s.z;
        p.nx = 2.0 * (s.qx * s.qz + s.qw * s.qy);
        p.ny = 2.0 * (s.qy * s.qz - s.qw * s.qx);
        p.nz = 1.0 - 2.0 * (s.qx * s.qx + s.qy * s.qy);
        p.DofFive = s.roll;
        p.iEndAngle = s.gripper;
        return p;
    };

    Dof dof1{}, dof2{};
    const bool ok1 = compute_psm(build_post(snap1), dof1);
    const bool ok2 = compute_psm(build_post(snap2), dof2);

    // ---------- 4) q_now 算不出：本帧发 NaN，不刷任何基准/缓存 ----------
    if (!ok1 || !ok2) {
        send_all_nan();
        return controller_interface::return_type::OK;
    }

    // ---------- 5) 首帧 或 踩踏板：刷新基准 + 34 接口发 NaN ----------
    if (!ref_ready_ || clutch) {
        master_ref_ci_[0] = q_to_mapped_ci(dof1);
        master_ref_ci_[1] = q_to_mapped_ci(dof2);
        // 从端基准 = 上一正常帧的指令；last_cmd 不更新（防 NaN 污染）
        slave_ref_ci_[0] = last_cmd_ci_[0];
        slave_ref_ci_[1] = last_cmd_ci_[1];
        ref_ready_ = true;

        send_all_nan();

        if (clutch) {
            RCLCPP_INFO_THROTTLE(logger, *get_node()->get_clock(), 500, "踏板触发，刷新基准，本帧下发 NaN (epoch=%.0f)", cur_epoch);
        }
        return controller_interface::return_type::OK;
    }

    // ---------- 6) 正常帧：Q_out = S_ref + (M_now − M_ref) ----------
    const auto mapped1 = q_to_mapped_ci(dof1);
    const auto mapped2 = q_to_mapped_ci(dof2);

    for (size_t i = 0; i < kMappedPerArm; ++i) {
        const double out1 = slave_ref_ci_[0][i] + (mapped1[i] - master_ref_ci_[0][i]);
        const double out2 = slave_ref_ci_[1][i] + (mapped2[i] - master_ref_ci_[1][i]);
        put(i, out1);
        put(kPerArmCommand + i, out2);
        last_cmd_ci_[0][i] = out1;
        last_cmd_ci_[1][i] = out2;
    }
    // motor 接口 [11..16]/[28..33] 保持 on_activate 时的 0，不写

    return controller_interface::return_type::OK;
}

} // namespace psm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::IntelligentCan0Controller, controller_interface::ControllerInterface)