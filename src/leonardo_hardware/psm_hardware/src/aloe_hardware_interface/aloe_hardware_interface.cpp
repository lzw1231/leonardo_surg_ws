#include "psm_hardware/aloe_hardware_interface/aloe_hardware_interface.hpp"

#include <chrono>
#include <stdexcept>
#include <system_error>
#include <thread>

namespace psm_hardware_interface {

namespace {
rclcpp::Logger LOGGER = rclcpp::get_logger("AloeHardwareInterface");
const double qNAN = std::numeric_limits<double>::quiet_NaN();
} // namespace

// ============================================================================
// 初始化与配置
// ============================================================================

hardware_interface::CallbackReturn AloeHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &hp = info_.hardware_parameters;

    try {
        // 1. 解析基础参数
        can0_interface_ = hp.at("can0_interface");
        can1_interface_ = hp.at("can1_interface");

        uint32_t id1 = std::stoul(hp.at("slave_1_id"), nullptr, 0);
        uint32_t id2 = std::stoul(hp.at("slave_2_id"), nullptr, 0);
        uint32_t id3 = std::stoul(hp.at("slave_3_id"), nullptr, 0);

        max_current_ = std::stoi(hp.at("max_current"));
        speed_level_ = std::stoi(hp.at("speed_level"));

        // 参数校验
        if (max_current_ < 0 || max_current_ > 255) {
            throw std::invalid_argument("max_current must be 0-255");
        }
        if (speed_level_ < 1 || speed_level_ > 3) {
            throw std::invalid_argument("speed_level must be 1-3");
        }

        // 2. 定义机械臂结构 (数据驱动核心)
        // 这里集中定义了所有关节的后缀，清晰易读，修改只需改这里
        std::vector<std::string> common_suffixes = {
            "_j01_base",       "_j03_oc",         "_j04_oc",         "_j07_oc",         "_j08_oc",        "_j11_oc",
            "_j12_oc",         "_j13_oc",         "_j14_oc",         "_j17_18",         "_j20_21",        "_joint_motor_01",
            "_joint_motor_02", "_joint_motor_03", "_joint_motor_04", "_joint_motor_05", "_joint_motor_06"};

        arm_definitions_.emplace_back("s1", id1, common_suffixes);
        arm_definitions_.emplace_back("s2", id2, common_suffixes);
        arm_definitions_.emplace_back("s3", id3, common_suffixes);

        // 3. 动态构建关节映射
        if (!build_dynamic_joint_map()) {
            return hardware_interface::CallbackReturn::ERROR;
        }

    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Init failed: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AloeHardwareInterface::on_configure(const rclcpp_lifecycle::State &) {
    try {
        can0_.open(can0_interface_);
        can1_.open(can1_interface_);
        RCLCPP_INFO(LOGGER, "CAN sockets opened: %s, %s", can0_.ifname().c_str(), can1_.ifname().c_str());
    } catch (const std::system_error &e) {
        RCLCPP_ERROR(LOGGER, "Failed to open CAN: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 激活与去激活
// ============================================================================

hardware_interface::CallbackReturn AloeHardwareInterface::on_activate(const rclcpp_lifecycle::State &) {
    try {
        // 遍历所有定义的臂，执行使能、设置参数、回零
        for (const auto &arm : arm_definitions_) {
            CanSocket &can = get_can_socket_for_arm(arm.prefix);

            // 1. 使能
            can.send(CanProtocol::make_enable_all(arm.can_id));

            // 2. 设置电流/速度
            can.send(
                CanProtocol::make_set_current_speed(arm.can_id, static_cast<uint8_t>(max_current_), static_cast<uint8_t>(speed_level_)));

            // 3. 回零
            can.send(CanProtocol::make_home_all(arm.can_id));
        }

        // 等待回零完成 (假设需要 3秒)
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // 初始化状态：回零后假设所有关节为 0
        for (auto &[name, joint] : joint_map_) {
            joint->state = 0.0;
            joint->command = qNAN;
        }

        RCLCPP_INFO(LOGGER, "All arms activated and homed.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Activation error: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AloeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) {
    try {
        for (const auto &arm : arm_definitions_) {
            CanSocket &can = get_can_socket_for_arm(arm.prefix);
            can.send(CanProtocol::make_stop_all(arm.can_id));
            can.send(CanProtocol::make_disable_all(arm.can_id));
        }

        // 重置数据
        for (auto &[name, joint] : joint_map_) {
            joint->state = qNAN;
            joint->command = qNAN;
        }
        RCLCPP_INFO(LOGGER, "All arms deactivated.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "Deactivation error: %s", e.what());
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 接口导出 (基于 Map 自动匹配)
// ============================================================================

std::vector<hardware_interface::StateInterface::ConstSharedPtr> AloeHardwareInterface::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
    interfaces.reserve(info_.joints.size());

    for (const auto &ji : info_.joints) {
        auto it = joint_map_.find(ji.name);
        if (it != joint_map_.end()) {
            interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(ji.name, hardware_interface::HW_IF_POSITION, &it->second->state));
        } else {
            RCLCPP_ERROR(LOGGER, "State interface missing for joint: %s", ji.name.c_str());
        }
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> AloeHardwareInterface::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
    interfaces.reserve(info_.joints.size());

    for (const auto &ji : info_.joints) {
        auto it = joint_map_.find(ji.name);
        if (it != joint_map_.end()) {
            interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(ji.name, hardware_interface::HW_IF_POSITION, &it->second->command));
        } else {
            RCLCPP_ERROR(LOGGER, "Command interface missing for joint: %s", ji.name.c_str());
        }
    }
    return interfaces;
}

// ============================================================================
// 数据读写
// ============================================================================

hardware_interface::return_type AloeHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    int valid_cmd_count = 0;

    // 用于构建日志字符串
    std::string debug_log;

    for (auto &[name, joint] : joint_map_) {
        // 检查 command 是否为有效数值
        if (std::isfinite(joint->command)) {
            valid_cmd_count++;

            // 将 Command 同步给 State (用于 RViz 可视化)
            // 注意：如果这里不赋值，RViz 中的模型将不会动
            joint->state = joint->command;

            // 仅记录前几个关节的值用于调试，避免日志过长
            if (debug_log.length() < 200) {
                char buffer[64];
                snprintf(buffer, sizeof(buffer), "[%s: %.3f] ", name.c_str(), joint->command);
                debug_log += buffer;
            }
        } else {
            // 如果 command 是 NaN，保持 state 不变（或者根据需要设为 NaN）
            // 当前逻辑：保持上一次的有效 state
        }
    }

    // 每 1000ms (1秒) 打印一次接收到的命令状态
    RCLCPP_INFO_THROTTLE(LOGGER, steady_clock, 1000, "Read Cycle: Received %d valid commands. Sample: %s", valid_cmd_count,
                         debug_log.empty() ? "None" : debug_log.c_str());

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AloeHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
    // 暂时留空，不发送 CAN 数据
    return hardware_interface::return_type::OK;
}

// ============================================================================
// 私有辅助方法实现
// ============================================================================

bool AloeHardwareInterface::build_dynamic_joint_map() {
    joint_map_.clear();

    // 遍历所有臂的定义
    for (const auto &arm : arm_definitions_) {
        // 遍历该臂的所有关节后缀
        for (const auto &suffix : arm.joint_suffixes) {
            std::string joint_name = arm.prefix + suffix;

            // 创建新的 JointData 并存入 Map
            // 使用 make_unique 自动管理内存
            joint_map_[joint_name] = std::make_unique<JointData>();
        }
    }

    // 校验：确保 URDF 中声明的所有关节都在我们的 Map 中
    if (info_.joints.size() != joint_map_.size()) {
        RCLCPP_ERROR(LOGGER, "Joint count mismatch! URDF: %zu, Code: %zu", info_.joints.size(), joint_map_.size());
        // 打印缺失或多余的关节以便调试
        for (const auto &ji : info_.joints) {
            if (joint_map_.find(ji.name) == joint_map_.end()) {
                RCLCPP_ERROR(LOGGER, "  Missing in code: %s", ji.name.c_str());
            }
        }
        return false;
    }

    RCLCPP_INFO(LOGGER, "Joint map built successfully with %zu joints.", joint_map_.size());
    return true;
}

CanSocket &AloeHardwareInterface::get_can_socket_for_arm(const std::string &prefix) {
    // 根据前缀判断使用哪个 CAN 口
    // s1, s2 -> can0_
    // s3 -> can1_
    if (prefix == "s1" || prefix == "s2") {
        return can0_;
    } else if (prefix == "s3") {
        return can1_;
    }
    throw std::runtime_error("Unknown arm prefix: " + prefix);
}

} // namespace psm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_hardware_interface::AloeHardwareInterface, hardware_interface::SystemInterface)