#include "psm_hardware/aloe_hardware_interface/aloe_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <system_error>
#include <thread>

namespace psm_hardware_interface {

namespace {
rclcpp::Logger LOGGER = rclcpp::get_logger("AloeHardwareInterface");
const double qNAN = std::numeric_limits<double>::quiet_NaN();
} // namespace

// ---------------------------------------------------------------------------
// 生命周期回调
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn AloeHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {

    if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto &hp = info_.hardware_parameters;

    try {
        can0_interface_ = hp.at("can0_interface");
        can1_interface_ = hp.at("can1_interface");
        snake_1_id_ = std::stoul(hp.at("slave_1_id"), nullptr, 0);
        snake_2_id_ = std::stoul(hp.at("slave_2_id"), nullptr, 0);
        snake_3_id_ = std::stoul(hp.at("slave_3_id"), nullptr, 0);
        max_current_ = std::stoi(hp.at("max_current"));
        speed_level_ = std::stoi(hp.at("speed_level"));

        if (max_current_ < 0 || max_current_ > 255) {
            RCLCPP_ERROR(LOGGER, "max_current=%d 超出 0~255（0.01A 单位）", max_current_);
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (speed_level_ < 1 || speed_level_ > 3) {
            RCLCPP_ERROR(LOGGER, "speed_level=%d 必须为 1/2/3", speed_level_);
            return hardware_interface::CallbackReturn::ERROR;
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(LOGGER, "硬件参数解析失败: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 建立关节名 → Joint* 映射并校验 URDF
    if (!build_joint_map()) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AloeHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    try {
        can0_.open(can0_interface_);
        can1_.open(can1_interface_);
    } catch (const std::system_error &e) {
        RCLCPP_ERROR(LOGGER, "CAN 接口初始化失败: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR; // 成员析构自动 close
    }

    // 缓存重置：Joint 构造时已是 NaN，无需再 fill
    RCLCPP_INFO(LOGGER, "CAN 已绑定: %s(handle=%d), %s(handle=%d)", can0_.ifname().c_str(), can0_.handle(), can1_.ifname().c_str(),
                can1_.handle());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AloeHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

    try {
        // 1. 先使能三个从臂
        can0_.send(CanProtocol::make_enable_all(snake_1_id_));
        can0_.send(CanProtocol::make_enable_all(snake_2_id_));
        can1_.send(CanProtocol::make_enable_all(snake_3_id_));

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 2. 使能后再设置电流/速度
        const auto current = static_cast<uint8_t>(max_current_);
        const auto speed = static_cast<uint8_t>(speed_level_);

        can0_.send(CanProtocol::make_set_current_speed(snake_1_id_, current, speed));
        can0_.send(CanProtocol::make_set_current_speed(snake_2_id_, current, speed));
        can1_.send(CanProtocol::make_set_current_speed(snake_3_id_, current, speed));

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 3. 全部轴回零
        can0_.send(CanProtocol::make_home_all(snake_1_id_));
        can0_.send(CanProtocol::make_home_all(snake_2_id_));
        can1_.send(CanProtocol::make_home_all(snake_3_id_));

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // 回零后位置同步为 0，避免下一次 write() 从旧值突跳到 0
        for (auto &[name, j] : joint_map_) {
            j->state = 0.0;
            j->command = qNAN;
        }

    } catch (const std::system_error &e) {
        RCLCPP_ERROR(LOGGER, "CAN 发送失败: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    } catch (const std::invalid_argument &e) {
        RCLCPP_ERROR(LOGGER, "参数非法: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AloeHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {

    try {
        // 1. 先停止：按当前位置保持位置伺服
        can0_.send(CanProtocol::make_stop_all(snake_1_id_));
        can0_.send(CanProtocol::make_stop_all(snake_2_id_));
        can1_.send(CanProtocol::make_stop_all(snake_3_id_));

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 2. 再全部轴去使能，释放电机进入待机
        can0_.send(CanProtocol::make_disable_all(snake_1_id_));
        can0_.send(CanProtocol::make_disable_all(snake_2_id_));
        can1_.send(CanProtocol::make_disable_all(snake_3_id_));

        for (auto &[name, j] : joint_map_) {
            j->state = qNAN;
            j->command = qNAN;
        }

        RCLCPP_INFO(LOGGER, "3 条从臂已停止并去使能");
    } catch (const std::system_error &e) {
        // 去激活失败不应阻止生命周期继续，记录错误后仍返回 SUCCESS 让框架完成清理
        RCLCPP_ERROR(LOGGER, "发送停止/去使能指令失败: %s", e.what());
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// 接口导出
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface::ConstSharedPtr> AloeHardwareInterface::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
    state_interfaces.reserve(info_.joints.size());

    for (const auto &ji : info_.joints) {
        Joint *j = joint_map_.at(ji.name);
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(ji.name, hardware_interface::HW_IF_POSITION, &j->state));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> AloeHardwareInterface::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
    command_interfaces.reserve(info_.joints.size());

    for (const auto &ji : info_.joints) {
        Joint *j = joint_map_.at(ji.name);
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(ji.name, hardware_interface::HW_IF_POSITION, &j->command));
    }
    return command_interfaces;
}

// ---------------------------------------------------------------------------
// 数据读写
// ---------------------------------------------------------------------------
hardware_interface::return_type AloeHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AloeHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// 关节映射表
// ---------------------------------------------------------------------------

bool AloeHardwareInterface::build_joint_map() {
    joint_map_.clear();
    auto reg = [this](const char *name, Joint &j) { joint_map_.emplace(name, &j); };

    // s1
    reg("s1_j01_base", s1_j01_base);
    reg("s1_j03_oc", s1_j03_oc);
    reg("s1_j04_oc", s1_j04_oc);
    reg("s1_j07_oc", s1_j07_oc);
    reg("s1_j08_oc", s1_j08_oc);
    reg("s1_j11_oc", s1_j11_oc);
    reg("s1_j12_oc", s1_j12_oc);
    reg("s1_j13_oc", s1_j13_oc);
    reg("s1_j14_oc", s1_j14_oc);
    reg("s1_j17_18", s1_j17_18);
    reg("s1_j20_21", s1_j20_21);
    reg("s1_joint_motor_01", s1_joint_motor_01);
    reg("s1_joint_motor_02", s1_joint_motor_02);
    reg("s1_joint_motor_03", s1_joint_motor_03);
    reg("s1_joint_motor_04", s1_joint_motor_04);
    reg("s1_joint_motor_05", s1_joint_motor_05);
    reg("s1_joint_motor_06", s1_joint_motor_06);

    // s2
    reg("s2_j01_base", s2_j01_base);
    reg("s2_j03_oc", s2_j03_oc);
    reg("s2_j04_oc", s2_j04_oc);
    reg("s2_j07_oc", s2_j07_oc);
    reg("s2_j08_oc", s2_j08_oc);
    reg("s2_j11_oc", s2_j11_oc);
    reg("s2_j12_oc", s2_j12_oc);
    reg("s2_j13_oc", s2_j13_oc);
    reg("s2_j14_oc", s2_j14_oc);
    reg("s2_j17_18", s2_j17_18);
    reg("s2_j20_21", s2_j20_21);
    reg("s2_joint_motor_01", s2_joint_motor_01);
    reg("s2_joint_motor_02", s2_joint_motor_02);
    reg("s2_joint_motor_03", s2_joint_motor_03);
    reg("s2_joint_motor_04", s2_joint_motor_04);
    reg("s2_joint_motor_05", s2_joint_motor_05);
    reg("s2_joint_motor_06", s2_joint_motor_06);

    // s3
    reg("s3_j01_base", s3_j01_base);
    reg("s3_j03_oc", s3_j03_oc);
    reg("s3_j04_oc", s3_j04_oc);
    reg("s3_j07_oc", s3_j07_oc);
    reg("s3_j08_oc", s3_j08_oc);
    reg("s3_j11_oc", s3_j11_oc);
    reg("s3_j12_oc", s3_j12_oc);
    reg("s3_j13_oc", s3_j13_oc);
    reg("s3_j14_oc", s3_j14_oc);
    reg("s3_j17_18", s3_j17_18);
    reg("s3_j20_21", s3_j20_21);
    reg("s3_joint_motor_01", s3_joint_motor_01);
    reg("s3_joint_motor_02", s3_joint_motor_02);
    reg("s3_joint_motor_03", s3_joint_motor_03);
    reg("s3_joint_motor_04", s3_joint_motor_04);
    reg("s3_joint_motor_05", s3_joint_motor_05);
    reg("s3_joint_motor_06", s3_joint_motor_06);

    // 校验：数量一致 + 每个 URDF 关节名都能查到
    if (info_.joints.size() != joint_map_.size()) {
        RCLCPP_ERROR(LOGGER, "URDF 关节数 %zu != 代码映射表 %zu", info_.joints.size(), joint_map_.size());
        return false;
    }
    for (const auto &ji : info_.joints) {
        if (joint_map_.find(ji.name) == joint_map_.end()) {
            RCLCPP_ERROR(LOGGER, "URDF 关节 [%s] 在代码映射表中不存在", ji.name.c_str());
            return false;
        }
    }
    RCLCPP_INFO(LOGGER, "关节映射表建立完成，共 %zu 个关节", joint_map_.size());
    return true;
}

} // namespace psm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_hardware_interface::AloeHardwareInterface, hardware_interface::SystemInterface)