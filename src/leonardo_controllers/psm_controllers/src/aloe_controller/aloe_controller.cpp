#include "psm_controllers/aloe_controller/aloe_controller.hpp"

#include <cmath>
#include <iomanip>

namespace psm_controllers {

namespace {
rclcpp::Logger LOGGER = rclcpp::get_logger("AloeController");
const double qNaN = std::numeric_limits<double>::quiet_NaN();
} // namespace

// ============================================================================
// 接口配置
// ============================================================================

controller_interface::InterfaceConfiguration AloeController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    std::vector<std::string> suffixes = {"_j01_base/position",       "_j03_oc/position",         "_j04_oc/position",
                                         "_j07_oc/position",         "_j08_oc/position",         "_j11_oc/position",
                                         "_j12_oc/position",         "_j13_oc/position",         "_j14_oc/position",
                                         "_j17_18/position",         "_j20_21/position",         "_joint_motor_01/position",
                                         "_joint_motor_02/position", "_joint_motor_03/position", "_joint_motor_04/position",
                                         "_joint_motor_05/position", "_joint_motor_06/position"};

    for (const auto &prefix : arm_prefixes_) {
        for (const auto &suffix : suffixes) {
            config.names.push_back(prefix + suffix);
        }
    }
    return config;
}

controller_interface::InterfaceConfiguration AloeController::state_interface_configuration() const {
    return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE};
}

// ============================================================================
// 生命周期管理
// ============================================================================

controller_interface::CallbackReturn AloeController::on_init() {
    try {
        auto node = get_node();
        if (!node->has_parameter("teleop_sub_topic")) {
            node->declare_parameter<std::string>("teleop_sub_topic", "/leonardo/pedal_mapping_cmd");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Init error: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();
    teleop_topic_name_ = node->get_parameter("teleop_sub_topic").as_string();

    for (int i = 0; i < NUM_ARMS; ++i) {
        arms_[i].name = arm_prefixes_[i];
        arms_[i].is_initialized = false;
    }

    teleop_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        teleop_topic_name_, rclcpp::QoS(1).reliable(), [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (msg->data.size() == TOPIC_DATA_SIZE) {
                latest_topic_data_ = msg->data;
            } else {
                RCLCPP_WARN_THROTTLE(LOGGER, *get_node()->get_clock(), 2000, "Invalid topic size: %zu", msg->data.size());
            }
        });

    RCLCPP_INFO(LOGGER, "Configured. Subscribing to: %s", teleop_topic_name_.c_str());
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(LOGGER, "Activating... Total command interfaces: %zu", command_interfaces_.size());

    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        RCLCPP_DEBUG(LOGGER, "  Cmd[%zu]: %s", i, command_interfaces_[i].get_name().c_str());
    }

    for (auto &ci : command_interfaces_) {
        (void)ci.set_value(qNaN);
    }

    // 【新增】初始化位置保持缓存，初始为 0.0 防止 RViz 初始状态乱跳
    for (int i = 0; i < NUM_ARMS; ++i) {
        last_valid_commands_[i].fill(0.0);
    }

    last_epoch_ = qNaN;
    latest_topic_data_.clear();

    RCLCPP_INFO(LOGGER, "Activated.");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_deactivate(const rclcpp_lifecycle::State &) {
    for (auto &ci : command_interfaces_) {
        (void)ci.set_value(qNaN);
    }
    RCLCPP_INFO(LOGGER, "Deactivated.");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_cleanup(const rclcpp_lifecycle::State &) {
    teleop_sub_.reset();
    latest_topic_data_.clear();
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// 核心逻辑辅助函数
// ============================================================================

bool AloeController::compute_arm_joints(int arm_index, const std::vector<double> &topic_data,
                                        std::array<double, JOINTS_PER_ARM> &out_joints) {
    int offset = arm_index * DATA_PER_ARM_IN_TOPIC;

    Post post;
    post.iPosX = topic_data[offset + 0];
    post.iPosY = topic_data[offset + 1];
    post.iPosZ = topic_data[offset + 2];

    double qw = topic_data[offset + 3];
    double qx = topic_data[offset + 4];
    double qy = topic_data[offset + 5];
    double qz = topic_data[offset + 6];

    post.nx = 2.0 * (qx * qz + qw * qy);
    post.ny = 2.0 * (qy * qz - qw * qx);
    post.nz = 1.0 - 2.0 * (qx * qx + qy * qy);

    post.DofFive = topic_data[offset + 7];
    post.iEndAngle = topic_data[offset + 10];

    Dof dof_result;
    if (!compute_psm(post, dof_result)) {
        return false;
    }

    out_joints[0] = dof_result.q[0];
    out_joints[1] = dof_result.q[1];
    out_joints[2] = dof_result.q[2];
    out_joints[3] = dof_result.q[3];
    out_joints[4] = dof_result.q[4];
    out_joints[5] = dof_result.q[5];
    out_joints[6] = dof_result.q[6];

    return true;
}

void AloeController::write_to_hardware(int arm_index, const std::array<double, JOINTS_PER_ARM> &joints, bool send_nan) {
    size_t base_idx = static_cast<size_t>(arm_index) * CMD_INTERFACES_PER_ARM;

    if (base_idx >= command_interfaces_.size()) {
        RCLCPP_ERROR(LOGGER, "Invalid arm_index %d, base_idx %zu >= size %zu", arm_index, base_idx, command_interfaces_.size());
        return;
    }

    for (size_t i = 0; i < CMD_INTERFACES_PER_ARM; ++i) {
        double val = qNaN;

        if (!send_nan && i < 11) {
            // 正常计算逻辑
            switch (i) {
            case 0:
                val = joints[0];
                break;
            case 1:
                val = joints[1];
                break;
            case 2:
                val = joints[2];
                break;
            case 3:
                val = -joints[2];
                break;
            case 4:
                val = -joints[1];
                break;
            case 5:
                val = joints[3];
                break;
            case 6:
                val = joints[4];
                break;
            case 7:
                val = joints[3];
                break;
            case 8:
                val = joints[4];
                break;
            case 9:
                val = joints[5];
                break;
            case 10:
                val = joints[6];
                break;
            default:
                break;
            }
            // 【关键修改】计算成功后，更新缓存，供后续“保持”使用
            last_valid_commands_[arm_index][i] = val;
        } else {
            // 【关键修改】当 send_nan 为 true (离合器/异常) 时，不再发送真正的 NaN，
            // 而是发送上一帧缓存的有效值，从而实现 RViz 和物理电机的“位置保持”
            val = last_valid_commands_[arm_index][i];
        }

        size_t target_idx = base_idx + i;
        if (target_idx < command_interfaces_.size()) {
            (void)command_interfaces_[target_idx].set_value(val);
        } else {
            RCLCPP_ERROR(LOGGER, "Target index %zu out of bounds!", target_idx);
        }
    }
}

void AloeController::reset_all_references() {
    for (int i = 0; i < NUM_ARMS; ++i) {
        arms_[i].master_ref_joints = arms_[i].current_master_joints;
        arms_[i].is_initialized = true;
    }
    RCLCPP_INFO(LOGGER, "References reset due to Clutch.");
}

// ============================================================================
// Update 循环
// ============================================================================
controller_interface::return_type AloeController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    std::vector<double> topic_data;
    double current_epoch = qNaN;

    // --- 1. 线程安全地获取最新话题数据 ---
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (latest_topic_data_.empty()) {
            static bool warned_no_data = false;
            if (!warned_no_data) {
                RCLCPP_WARN(LOGGER, "No topic data received yet. Holding position.");
                warned_no_data = true;
            }
            // 无数据时，触发保持逻辑
            for (int i = 0; i < NUM_ARMS; ++i)
                write_to_hardware(i, {}, true);

            return controller_interface::return_type::OK;
        }

        topic_data = latest_topic_data_;
        current_epoch = topic_data[44];
    }

    // --- 2. 离合器（Clutch）检测逻辑 ---
    bool clutch_active = std::isfinite(current_epoch) && std::isfinite(last_epoch_) && (current_epoch - last_epoch_) > 0.5;

    if (std::isfinite(current_epoch)) {
        last_epoch_ = current_epoch;
    }

    if (clutch_active) {
        RCLCPP_INFO_THROTTLE(LOGGER, *get_node()->get_clock(), 1000, "Clutch Triggered! Holding Position.");

        reset_all_references();

        // 【核心修复】传入 true，write_to_hardware 内部会自动使用 last_valid_commands_ 进行保持
        for (int a = 0; a < NUM_ARMS; ++a)
            write_to_hardware(a, {}, true);

        return controller_interface::return_type::OK;
    }

    // --- 3. 主控制循环：遍历每个机械臂 ---
    for (int i = 0; i < NUM_ARMS; ++i) {
        std::array<double, JOINTS_PER_ARM> current_joints;

        if (!compute_arm_joints(i, topic_data, current_joints)) {
            RCLCPP_WARN_THROTTLE(LOGGER, *get_node()->get_clock(), 1000, "IK Failed for arm %d. Holding position.", i);
            // IK 失败时，触发保持逻辑
            write_to_hardware(i, {}, true);
            continue;
        }

        arms_[i].current_master_joints = current_joints;

        if (!arms_[i].is_initialized) {
            RCLCPP_INFO(LOGGER, "Arm %d initialized. Ref joints set.", i);

            arms_[i].master_ref_joints = current_joints;
            arms_[i].slave_ref_joints.fill(0.0);
            arms_[i].is_initialized = true;

            // 初始化阶段触发一次保持（此时缓存里是 0.0）
            write_to_hardware(i, {}, true);
            continue;
        }

        // 3.3 主从映射计算 (Delta Control)
        std::array<double, JOINTS_PER_ARM> output_joints;
        for (int j = 0; j < JOINTS_PER_ARM; ++j) {
            double delta = current_joints[j] - arms_[i].master_ref_joints[j];
            output_joints[j] = arms_[i].slave_ref_joints[j] + delta;
        }

        // 3.4 发送控制指令 (传入 false，正常计算并更新缓存)
        write_to_hardware(i, output_joints, false);
    }

    return controller_interface::return_type::OK;
}

} // namespace psm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::AloeController, controller_interface::ControllerInterface)
