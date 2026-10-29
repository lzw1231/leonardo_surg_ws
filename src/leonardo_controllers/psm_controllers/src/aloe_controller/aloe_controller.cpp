#include "psm_controllers/aloe_controller/aloe_controller.hpp"

#include <algorithm>
#include <cmath>

namespace psm_controllers {

// --- 常量定义 (移至此处以避免编译依赖问题) ---
namespace {
rclcpp::Logger LOGGER = rclcpp::get_logger("aloe_controller_node");
const double qNaN = std::numeric_limits<double>::quiet_NaN();

static constexpr int NUM_ARMS = 3;
static constexpr int JOINTS_PER_ARM = 7;          // IK 输出的 7 个自由度
static constexpr int CMD_INTERFACES_PER_ARM = 17; // 硬件实际的 17 个电机接口
static constexpr int TOPIC_DATA_SIZE = 45;
static constexpr int DATA_PER_ARM_IN_TOPIC = 11;

// 缩放系数：前3个为位置轴（放大），后4个为姿态轴（1:1）
static constexpr std::array<double, JOINTS_PER_ARM> kScale = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
} // namespace

// ============================================================================
// 接口配置
// ============================================================================
controller_interface::InterfaceConfiguration AloeController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const std::vector<std::string> suffixes = {"_j01_base/position",       "_j03_oc/position",         "_j04_oc/position",
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
    // 状态接口顺序必须与命令接口完全一致
    return command_interface_configuration();
}

// ============================================================================
// 生命周期管理
// ============================================================================
controller_interface::CallbackReturn AloeController::on_init() {
    try {
        auto node = get_node();
        if (!node->has_parameter("arm_mapping_cmd_topic")) {
            node->declare_parameter<std::string>("arm_mapping_cmd_topic", "/leonardo/arm_mapping/arm_mapping_cmd");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Init error: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_configure(const rclcpp_lifecycle::State &) {
    auto node = get_node();
    arm_mapping_cmd_topic_name_ = node->get_parameter("arm_mapping_cmd_topic").as_string();

    // 初始化手臂上下文
    for (int i = 0; i < NUM_ARMS; ++i) {
        arms_[i].name = arm_prefixes_[i];
        arms_[i].is_base_locked = false;
        arms_[i].base_master_joints.fill(0.0);
        arms_[i].base_slave_pos.fill(0.0);
        arms_[i].current_master_joints.fill(0.0);
    }

    // 订阅遥操作数据
    topic_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(arm_mapping_cmd_topic_name_, rclcpp::QoS(1).reliable(),
                                                                             [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                                                                                 std::lock_guard<std::mutex> lock(data_mutex_);
                                                                                 if (msg->data.size() == TOPIC_DATA_SIZE) {
                                                                                     latest_topic_data_ = msg->data;
                                                                                 }
                                                                             });

    RCLCPP_INFO(LOGGER, "Configured with Base-Frame Mapping Logic.");
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(LOGGER, "Activating...");
    last_epoch_ = -1.0;
    latest_topic_data_.clear();
    for (auto &ci : command_interfaces_) {
        (void)ci.set_value(qNaN);
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_deactivate(const rclcpp_lifecycle::State &) {
    for (auto &ci : command_interfaces_) {
        (void)ci.set_value(qNaN);
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AloeController::on_cleanup(const rclcpp_lifecycle::State &) {
    topic_sub_.reset();
    latest_topic_data_.clear();
    return CallbackReturn::SUCCESS;
}

// ============================================================================
// 核心逻辑辅助函数
// ============================================================================

/**
 * @brief 从 Topic 数据中解算指定手臂的 7 个关节角
 */
bool AloeController::compute_arm_joints(int arm_index, const std::vector<double> &topic_data,
                                        std::array<double, JOINTS_PER_ARM> &out_joints) {
    int offset = arm_index * DATA_PER_ARM_IN_TOPIC;
    Post post;
    post.iPosX = topic_data[offset + 0];
    post.iPosY = topic_data[offset + 1];
    post.iPosZ = topic_data[offset + 2];

    // 四元数转方向向量 (简化版，假设特定旋转顺序)
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
    if (!SurgicalKinematics::solvePsm(post, dof_result)) {
        return false;
    }
    for (int k = 0; k < JOINTS_PER_ARM; ++k) {
        out_joints[k] = dof_result.q[k];
    }
    return true;
}

/**
 * @brief 重置基准帧：锁定当前主手位置和从手硬件位置
 */
void AloeController::resetBaseFrame(ArmContext &arm, const std::array<double, JOINTS_PER_ARM> &current_master, size_t state_offset) {
    // 1. 锁定主手基准
    arm.base_master_joints = current_master;

    // 2. 读取并锁定从手硬件基准
    bool hw_state_valid = true;
    for (size_t k = 0; k < CMD_INTERFACES_PER_ARM; ++k) {
        if (state_offset + k < state_interfaces_.size()) {
            auto val_opt = state_interfaces_[state_offset + k].get_optional();
            if (val_opt.has_value() && std::isfinite(val_opt.value())) {
                arm.base_slave_pos[k] = val_opt.value();
            } else {
                hw_state_valid = false;
                break;
            }
        } else {
            hw_state_valid = false;
            break;
        }
    }

    // 3. 处理无效状态
    if (!hw_state_valid) {
        if (!arm.is_base_locked) {
            // 首次初始化且无硬件状态，安全归零
            arm.base_slave_pos.fill(0.0);
        }
        // 若非首次初始化，保持上一次锁定的基准不变，防止跳变
    }

    arm.is_base_locked = true;
    RCLCPP_INFO(get_node()->get_logger(), "Arm [%s] Base Locked. HW Valid: %s", arm.name.c_str(), hw_state_valid ? "true" : "false");
}

/**
 * @brief 根据基准映射计算当前的 17 个目标命令
 * 公式: Target = Base_Slave + Scale * (Current_Master - Base_Master)
 */
std::array<double, CMD_INTERFACES_PER_ARM> AloeController::computeTargetCommands(const ArmContext &arm) const {
    std::array<double, CMD_INTERFACES_PER_ARM> target_cmd;

    // 1. 计算 7 个独立关节的目标值
    double target_joints[JOINTS_PER_ARM];
    for (int j = 0; j < JOINTS_PER_ARM; ++j) {
        double delta = arm.current_master_joints[j] - arm.base_master_joints[j];

        // 获取该独立关节对应的硬件基准值
        // 映射关系:
        // j0->idx0, j1->idx1, j2->idx2, j3->idx5, j4->idx6, j5->idx9, j6->idx10
        int hw_idx = 0;
        switch (j) {
        case 0:
            hw_idx = 0;
            break;
        case 1:
            hw_idx = 1;
            break;
        case 2:
            hw_idx = 2;
            break;
        case 3:
            hw_idx = 5;
            break;
        case 4:
            hw_idx = 6;
            break;
        case 5:
            hw_idx = 9;
            break;
        case 6:
            hw_idx = 10;
            break;
        default:
            hw_idx = 0;
            break;
        }

        double base_val = arm.base_slave_pos[hw_idx];
        target_joints[j] = base_val + delta * kScale[j];
    }

    // 2. 将 7 个独立关节映射回 17 个硬件接口
    // 首先将所有接口初始化为基准值（针对非运动关节或虚拟电机）
    target_cmd = arm.base_slave_pos;

    // 然后根据机械结构应用对称/重复逻辑
    // idx 0,1,2 直接对应
    target_cmd[0] = target_joints[0];
    target_cmd[1] = target_joints[1];
    target_cmd[2] = target_joints[2];

    // idx 3,4 是 2,1 的负对称
    target_cmd[3] = -target_joints[2];
    target_cmd[4] = -target_joints[1];

    // idx 5,6 对应 3,4
    target_cmd[5] = target_joints[3];
    target_cmd[6] = target_joints[4];

    // idx 7,8 是 3,4 的重复
    target_cmd[7] = target_joints[3];
    target_cmd[8] = target_joints[4];

    // idx 9,10 对应 5,6
    target_cmd[9] = target_joints[5];
    target_cmd[10] = target_joints[6];

    // idx 11-16 保持基准值 (已在初始化时赋值)

    return target_cmd;
}

// ============================================================================
// 主循环 Update
// ============================================================================
controller_interface::return_type AloeController::update(const rclcpp::Time &, const rclcpp::Duration &) {
    // 1. 获取最新数据
    std::vector<double> topic_data;
    double current_epoch = qNaN;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (latest_topic_data_.empty()) {
            return controller_interface::return_type::OK;
        }
        topic_data = latest_topic_data_;
        current_epoch = topic_data[44];
    }

    // 2. 检测 Epoch 变化 (主从切换信号)
    bool need_reset = false;
    if (std::isfinite(current_epoch)) {
        if (last_epoch_ < 0 || std::abs(current_epoch - last_epoch_) > 0.5) {
            need_reset = true;
        }
        last_epoch_ = current_epoch;
    } else {
        return controller_interface::return_type::OK;
    }

    // 3. 逐臂处理
    for (int i = 0; i < NUM_ARMS; ++i) {
        ArmContext &arm = arms_[i];
        size_t interface_offset = static_cast<size_t>(i) * CMD_INTERFACES_PER_ARM;

        // 3.1 解算主手关节角
        if (!compute_arm_joints(i, topic_data, arm.current_master_joints)) {
            continue;
        }

        // 3.2 如果需要重置，则锁定新的基准帧
        if (need_reset || !arm.is_base_locked) {
            resetBaseFrame(arm, arm.current_master_joints, interface_offset);
        }

        // 3.3 计算目标命令并下发
        auto target_cmd = computeTargetCommands(arm);

        for (size_t k = 0; k < CMD_INTERFACES_PER_ARM; ++k) {
            size_t cmd_idx = interface_offset + k;
            if (cmd_idx < command_interfaces_.size()) {
                command_interfaces_[cmd_idx].set_value(target_cmd[k]);
            }
        }
    }

    return controller_interface::return_type::OK;
}

} // namespace psm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::AloeController, controller_interface::ControllerInterface)