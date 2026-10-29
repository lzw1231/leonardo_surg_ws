#include "teleop_controllers/pedal_mapping/pedal_6keys_mapping_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

namespace teleop_controllers {
namespace {
rclcpp::Logger LOGGER = rclcpp::get_logger("Pedal6KeysMappingController");
const double qNaN = std::numeric_limits<double>::quiet_NaN();
} // namespace

controller_interface::CallbackReturn Pedal6KeysMappingController::on_init() {
    try {
        if (!get_node()->has_parameter("pedal_pressed_key_topic")) {
            get_node()->declare_parameter<std::string>("pedal_pressed_key_topic", "/leonardo/pedal_pressed_key");
        }
        if (!get_node()->has_parameter("left_prefix")) {
            get_node()->declare_parameter<std::string>("left_prefix", "fd_left_ee_controller/ee");
        }
        if (!get_node()->has_parameter("right_prefix")) {
            get_node()->declare_parameter<std::string>("right_prefix", "fd_right_ee_controller/ee");
        }
        // 新增输出topic参数
        if (!get_node()->has_parameter("teleop_mapping_topic")) {
            get_node()->declare_parameter<std::string>("teleop_mapping_topic", "/leonardo/pedal_mapping_cmd");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "on_init 异常: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Pedal6KeysMappingController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    auto node = get_node();

    msg_.data.assign(REFS_SIZE, qNaN);
    pedal_state_topic_ = node->get_parameter("pedal_pressed_key_topic").as_string();
    left_prefix_ = node->get_parameter("left_prefix").as_string();
    right_prefix_ = node->get_parameter("right_prefix").as_string();
    teleop_mapping_topic_ = node->get_parameter("teleop_mapping_topic").as_string();

    if (pedal_state_topic_.empty() || left_prefix_.empty() || right_prefix_.empty() || teleop_mapping_topic_.empty()) {
        RCLCPP_ERROR(LOGGER, "参数不能为空");
        return CallbackReturn::ERROR;
    }

    left_interfaces_.clear();
    right_interfaces_.clear();
    for (const auto *suffix : kSuffixes) {
        left_interfaces_.push_back(left_prefix_ + "/" + suffix);
        right_interfaces_.push_back(right_prefix_ + "/" + suffix);
    }

    refs_.assign(REFS_SIZE, qNaN);
    refs_[EPOCH_IDX] = 0.0;
    last_event_counter_ = 0;

    // 踏板订阅
    pedal_sub_ = node->create_subscription<std_msgs::msg::Int32>(
        pedal_state_topic_, rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Int32::ConstSharedPtr msg) {
            pedal_key_.store(static_cast<int>(msg->data), std::memory_order_relaxed);
            pedal_event_counter_.fetch_add(1, std::memory_order_relaxed);
        });

    // 新增：遥操作指令发布器
    teleop_pub_ =
        node->create_publisher<std_msgs::msg::Float64MultiArray>(teleop_mapping_topic_, rclcpp::QoS(1).reliable().durability_volatile());

    RCLCPP_INFO(LOGGER, "Pedal6KeysMappingController(普通控制器) 已配置完成");
    RCLCPP_INFO(LOGGER, "  踏板话题: %s", pedal_state_topic_.c_str());
    RCLCPP_INFO(LOGGER, "  左手: %s", left_prefix_.c_str());
    RCLCPP_INFO(LOGGER, "  右手: %s", right_prefix_.c_str());
    RCLCPP_INFO(LOGGER, "  输出遥操作topic: %s", teleop_mapping_topic_.c_str());

    pedal_key_.store(0, std::memory_order_relaxed);
    pedal_event_counter_.store(0, std::memory_order_relaxed);
    return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Pedal6KeysMappingController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(left_interfaces_.size() + right_interfaces_.size());
    for (const auto &n : left_interfaces_) {
        config.names.push_back(n);
    }
    for (const auto &n : right_interfaces_) {
        config.names.push_back(n);
    }
    return config;
}

controller_interface::InterfaceConfiguration Pedal6KeysMappingController::state_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn Pedal6KeysMappingController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    const size_t expected = left_interfaces_.size() + right_interfaces_.size();
    if (command_interfaces_.size() != expected) {
        RCLCPP_ERROR(LOGGER, "command 接口数量不符：期望 %zu，实际 %zu", expected, command_interfaces_.size());
        return CallbackReturn::ERROR;
    }

    std::fill(refs_.begin(), refs_.end(), std::numeric_limits<double>::quiet_NaN());
    refs_[EPOCH_IDX] = 0.0;
    last_event_counter_ = 0;
    pedal_key_.store(0, std::memory_order_relaxed);
    pedal_event_counter_.store(0, std::memory_order_relaxed);

    RCLCPP_INFO(LOGGER, "Pedal6KeysMappingController 已激活成功");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Pedal6KeysMappingController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    std::fill(refs_.begin(), refs_.end(), qNaN);
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Pedal6KeysMappingController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    pedal_sub_.reset();
    teleop_pub_.reset();
    left_interfaces_.clear();
    right_interfaces_.clear();
    refs_.clear();
    last_event_counter_ = 0;
    pedal_key_.store(0, std::memory_order_relaxed);
    pedal_event_counter_.store(0, std::memory_order_relaxed);
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type Pedal6KeysMappingController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // 1. 检查踏板事件，更新epoch
    const int ev = pedal_event_counter_.load(std::memory_order_relaxed);
    if (ev != last_event_counter_) {
        last_event_counter_ = ev;
        refs_[EPOCH_IDX] += 1.0;
    }

    // 2. 根据key拷贝源数据到目标缓存
    const int key = pedal_key_.load(std::memory_order_relaxed);
    switch (key) {
    case 1:
        copy_source_to_dest(SRC_LEFT_IDX, DST_PSM_1_IDX);
        copy_source_to_dest(SRC_RIGHT_IDX, DST_PSM_2_IDX);
        break;
    case 3:
        copy_source_to_dest(SRC_LEFT_IDX, DST_PSM_2_IDX);
        copy_source_to_dest(SRC_RIGHT_IDX, DST_PSM_3_IDX);
        break;
    default:
        break;
    }

    // 3. 打包发布topic
    std::copy(refs_.begin(), refs_.end(), msg_.data.begin());
    teleop_pub_->publish(msg_);

    return controller_interface::return_type::OK;
}

void Pedal6KeysMappingController::copy_source_to_dest(size_t src_base, size_t dst_base) {
    for (size_t i = 0; i < SRC_SIZE; ++i) {
        const auto opt = command_interfaces_[src_base + i].get_optional();
        if (opt.has_value() && std::isfinite(opt.value())) {
            refs_[dst_base + i] = opt.value();
        }
    }
}

} // namespace teleop_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(teleop_controllers::Pedal6KeysMappingController, controller_interface::ControllerInterface)
