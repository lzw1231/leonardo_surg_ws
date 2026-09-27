#include "teleop_controllers/pedal_mapping/pedal_mapping_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace teleop_controllers {

// ============================================================================
// on_init
// ============================================================================
controller_interface::CallbackReturn PedalMappingController::on_init() {
    try {
        if (!get_node()->has_parameter("pedal_state_topic")) {
            get_node()->declare_parameter<std::string>("pedal_state_topic", "/leonardo/pedal_state");
        }
        if (!get_node()->has_parameter("left_prefix")) {
            get_node()->declare_parameter<std::string>("left_prefix", "fd_left_ee_controller/ee");
        }
        if (!get_node()->has_parameter("right_prefix")) {
            get_node()->declare_parameter<std::string>("right_prefix", "fd_right_ee_controller/ee");
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
PedalMappingController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    auto node = get_node();

    pedal_state_topic_ = node->get_parameter("pedal_state_topic").as_string();
    left_prefix_ = node->get_parameter("left_prefix").as_string();
    right_prefix_ = node->get_parameter("right_prefix").as_string();

    if (pedal_state_topic_.empty() || left_prefix_.empty() || right_prefix_.empty()) {
        RCLCPP_ERROR(node->get_logger(), "参数不能为空");
        return CallbackReturn::ERROR;
    }

    // 拼上游接口全名
    left_interfaces_.clear();
    right_interfaces_.clear();
    for (const auto *suffix : kSuffixes) {
        left_interfaces_.push_back(left_prefix_ + "/" + suffix);
        right_interfaces_.push_back(right_prefix_ + "/" + suffix);
    }

    // 导出内存：4 组数据 + 4 个 epoch
    refs_.assign(REFS_SIZE, std::numeric_limits<double>::quiet_NaN());
    for (size_t g = 0; g < GROUP_COUNT; ++g) {
        refs_[epoch_idx(g)] = 0.0;
    }
    last_event_counter_ = 0;

    // 订阅踏板话题（回调：记录 key + 计数事件）
    pedal_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        pedal_state_topic_, rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Int32::ConstSharedPtr msg) {
            pedal_key_.store(static_cast<int>(msg->data), std::memory_order_relaxed);
            pedal_event_counter_.fetch_add(1, std::memory_order_relaxed);
        });

    RCLCPP_INFO(node->get_logger(), "PedalMappingController 配置完成");
    RCLCPP_INFO(node->get_logger(), "  踏板话题: %s", pedal_state_topic_.c_str());
    RCLCPP_INFO(node->get_logger(), "  左手: %s", left_prefix_.c_str());
    RCLCPP_INFO(node->get_logger(), "  右手: %s", right_prefix_.c_str());
    RCLCPP_INFO(node->get_logger(), "  导出 %zu 组（每组含 epoch）:", GROUP_COUNT);
    for (size_t g = 0; g < GROUP_COUNT; ++g) {
        RCLCPP_INFO(node->get_logger(), "    %s  (+ %s/epoch)", kGroups[g], kGroups[g]);
    }

    pedal_key_.store(0, std::memory_order_relaxed);
    pedal_event_counter_.store(0, std::memory_order_relaxed);
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// 接口配置
// ============================================================================
controller_interface::InterfaceConfiguration PedalMappingController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(left_interfaces_.size() + right_interfaces_.size());
    for (const auto &n : left_interfaces_)
        config.names.push_back(n);
    for (const auto &n : right_interfaces_)
        config.names.push_back(n);
    return config;
}

controller_interface::InterfaceConfiguration PedalMappingController::state_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

// ============================================================================
// 生命周期
// ============================================================================
controller_interface::CallbackReturn
PedalMappingController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    const size_t expected = left_interfaces_.size() + right_interfaces_.size();
    if (command_interfaces_.size() != expected) {
        RCLCPP_ERROR(get_node()->get_logger(), "command 接口数量不符：期望 %zu，实际 %zu", expected,
                     command_interfaces_.size());
        return CallbackReturn::ERROR;
    }

    std::fill(refs_.begin(), refs_.end(), std::numeric_limits<double>::quiet_NaN());
    for (size_t g = 0; g < GROUP_COUNT; ++g) {
        refs_[epoch_idx(g)] = 0.0;
    }
    last_event_counter_ = 0;
    pedal_key_.store(0, std::memory_order_relaxed);
    pedal_event_counter_.store(0, std::memory_order_relaxed);

    RCLCPP_INFO(get_node()->get_logger(), "PedalMappingController 已激活");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PedalMappingController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    std::fill(refs_.begin(), refs_.end(), std::numeric_limits<double>::quiet_NaN());
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PedalMappingController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    pedal_sub_.reset();
    left_interfaces_.clear();
    right_interfaces_.clear();
    refs_.clear();
    last_event_counter_ = 0;
    pedal_key_.store(0, std::memory_order_relaxed);
    pedal_event_counter_.store(0, std::memory_order_relaxed);
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// update
// ============================================================================
controller_interface::return_type
PedalMappingController::update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                          const rclcpp::Duration & /*period*/) {
    return controller_interface::return_type::OK;
}

controller_interface::return_type
PedalMappingController::update_and_write_commands(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // ---- 1. 检查是否有新的按键事件 ----
    const int ev = pedal_event_counter_.load(std::memory_order_relaxed);
    if (ev != last_event_counter_) {
        last_event_counter_ = ev;
        // 有按键，所有组 epoch +1（通知下游重新定位）
        for (size_t g = 0; g < GROUP_COUNT; ++g) {
            refs_[epoch_idx(g)] += 1.0;
        }
        // RCLCPP_INFO(get_node()->get_logger(), "按键事件 #%d，%zu 组 epoch 全部 +1", ev, GROUP_COUNT);
    }

    // ---- 2. 根据当前 key 决定写什么 ----
    const int key = pedal_key_.load(std::memory_order_relaxed);

    switch (key) {

    case 1:
        copy_source_to_dest(SRC_LEFT_IDX, DST_PSM_1_IDX);
        break;

    case 2:
        break;

    case 3:
        copy_source_to_dest(SRC_RIGHT_IDX, DST_PSM_1_IDX);
        break;

    case 4:
        break;

    case 5:
        break;

    case 6:
        break;

    default:
        break;
    }

    return controller_interface::return_type::OK;
}

// ============================================================================
// 辅助函数
// ============================================================================
void PedalMappingController::copy_source_to_dest(size_t src_base, size_t dst_base) {
    for (size_t i = 0; i < REF_SIZE; ++i) {
        const auto opt = command_interfaces_[src_base + i].get_optional();
        if (opt.has_value() && std::isfinite(opt.value())) {
            refs_[dst_base + i] = opt.value();
        }
    }
}

// ============================================================================
// 导出 4 组数据 + 4 个 epoch
// ============================================================================
std::vector<hardware_interface::CommandInterface::SharedPtr>
PedalMappingController::on_export_reference_interfaces_list() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
    interfaces.reserve(REFS_SIZE);

    const std::string &ctrl_name = get_node()->get_name();

    // 4 组数据
    for (size_t g = 0; g < GROUP_COUNT; ++g) {
        for (size_t i = 0; i < REF_SIZE; ++i) {
            std::string if_name = std::string(kGroups[g]) + "/" + kSuffixes[i];
            interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(ctrl_name, if_name, &refs_[g * REF_SIZE + i]));
        }
    }

    // 4 个 epoch
    for (size_t g = 0; g < GROUP_COUNT; ++g) {
        std::string if_name = std::string(kGroups[g]) + "/epoch";
        interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(ctrl_name, if_name, &refs_[epoch_idx(g)]));
    }

    return interfaces;
}

} // namespace teleop_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(teleop_controllers::PedalMappingController, controller_interface::ChainableControllerInterface)