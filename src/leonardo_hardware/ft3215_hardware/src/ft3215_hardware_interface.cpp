#include "ft3215_hardware/ft3215_hardware_interface.hpp"

namespace ft3215_hardware {
namespace {
// 位置转换：计数 → rad
constexpr double POSITION_COUNT_TO_RAD = 2.0 * M_PI / 4095;
// 位置转换：rad → 计数
constexpr double RAD_TO_POSITION_COUNT = 4095 / (2.0 * M_PI);
// 舵机中立位置
constexpr s16 POSITION_ZERO = 2047;
// 速度转换：step/s → rad/s
constexpr double VELOCITY_STEP_TO_RAD_PER_SEC = 2.0 * M_PI / 4095;
// 速度转换：rad/s → step/s
constexpr double VELOCITY_RAD_PER_SEC_TO_STEP = 4095 / (2.0 * M_PI);
} // namespace

hardware_interface::CallbackReturn
FT3215HardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (auto ret = hardware_interface::SystemInterface::on_init(params);
        ret != hardware_interface::CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(get_logger(),
                     "基类 on_init 初始化失败，错误码 %d。 请确认 URDF <hardware> 标签内硬件参数配置正确! ",
                     static_cast<int>(ret));
        return ret;
    }

    const auto &hw_params = params.hardware_info.hardware_parameters;

    motor_ids_[joint_11_12] = static_cast<u8>(std::stoi(hw_params.at("joint_ft_11_12")));
    motor_ids_[joint_12_13] = static_cast<u8>(std::stoi(hw_params.at("joint_ft_12_13")));
    motor_ids_[joint_13_14] = static_cast<u8>(std::stoi(hw_params.at("joint_ft_13_14")));
    motor_ids_[joint_14_15] = static_cast<u8>(std::stoi(hw_params.at("joint_ft_14_15")));
    motor_ids_[joint_15_16] = static_cast<u8>(std::stoi(hw_params.at("joint_ft_15_16")));
    motor_ids_[joint_16_17] = static_cast<u8>(std::stoi(hw_params.at("joint_ft_16_17")));

    ft3215_port_ = hw_params.at("ft3215_port");
    baud_ = static_cast<int>(std::stoi(hw_params.at("baud")));

    sms_sts_ = std::make_shared<SMS_STS>();

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FT3215HardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state) {
    (void)previous_state;

    if (!sms_sts_->begin(baud_, ft3215_port_.c_str())) {
        RCLCPP_ERROR(get_logger(), "串口初始化失败: 无法连接设备 %s", ft3215_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_logger(), "[ST3215] 串口已初始化: %s @ %d bps", ft3215_port_.c_str(), baud_);

    // 舵机 Ping 检测
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        const u8 id = motor_ids_[i];
        const u8 ping_id = sms_sts_->Ping(id);
        if (motor_ids_[i] != sms_sts_->Ping(id)) {
            RCLCPP_ERROR(get_logger(), "ft3215舵机Ping检测失败: 期望ID %u, 实际返回 %u", static_cast<unsigned>(id),
                         static_cast<unsigned>(ping_id));
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FT3215HardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) {
    if (sms_sts_) {
        sms_sts_->end();
        RCLCPP_INFO(get_logger(), "[ST3215] 串口已释放: %s", ft3215_port_.c_str());
    } else {
        RCLCPP_WARN(get_logger(), "[ST3215] cleanup 时 sms_sts_ 为空，可能未配置或已清理");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FT3215HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        sms_sts_->ServoMode(motor_ids_[i]);
    }

    RCLCPP_INFO(get_logger(), "ft3215舵机已设为位置模式");

    // 准备5个电机的目标位置、速度、加速度数组
    std::array<s16, 6> positions;
    std::array<u16, 6> speeds;
    std::array<u8, 6> accs;

    positions.fill(POSITION_ZERO);
    speeds.fill(0);
    accs.fill(0);

    // 舵机同步运行
    sms_sts_->SyncWritePosEx(motor_ids_.data(), motor_ids_.size(), positions.data(), speeds.data(), accs.data());

    rclcpp::sleep_for(std::chrono::milliseconds(3000));

    RCLCPP_INFO(get_logger(), "ft3215舵机已到达中立位2047...");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
FT3215HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // 准备6个电机的目标位置、速度、加速度数组
    std::array<s16, 6> positions{};
    std::array<u16, 6> speeds{};
    std::array<u8, 6> accs{};

    positions.fill(0);
    speeds.fill(0);
    accs.fill(0);

    // 舵机同步运行
    sms_sts_->SyncWritePosEx(motor_ids_.data(), motor_ids_.size(), positions.data(), speeds.data(), accs.data());

    rclcpp::sleep_for(std::chrono::milliseconds(3000));

    RCLCPP_INFO(get_logger(), "ft3215舵机已回机械零位...");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FT3215HardwareInterface::read(const rclcpp::Time & /*time*/,
                                                              const rclcpp::Duration & /*period*/) {

    auto pos_11_12 =
        static_cast<double>((sms_sts_->ReadPos(motor_ids_[joint_11_12]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
    auto pos_12_13 =
        static_cast<double>((sms_sts_->ReadPos(motor_ids_[joint_12_13]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
    auto pos_13_14 =
        static_cast<double>((sms_sts_->ReadPos(motor_ids_[joint_13_14]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
    auto pos_14_15 =
        static_cast<double>((sms_sts_->ReadPos(motor_ids_[joint_14_15]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
    auto pos_15_16 =
        static_cast<double>((sms_sts_->ReadPos(motor_ids_[joint_15_16]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);
    auto pos_16_17 =
        static_cast<double>((sms_sts_->ReadPos(motor_ids_[joint_16_17]) - POSITION_ZERO) * POSITION_COUNT_TO_RAD);

    // 更新状态接口
    set_state("joint_ft_11_12/position", pos_11_12);
    set_state("joint_ft_12_13/position", pos_12_13);
    set_state("joint_ft_13_14/position", pos_13_14);
    set_state("joint_ft_14_15/position", pos_14_15);
    set_state("joint_ft_15_16/position", pos_15_16);
    set_state("joint_ft_16_17/position", pos_16_17);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FT3215HardwareInterface::write(const rclcpp::Time & /*time*/,
                                                               const rclcpp::Duration & /*period*/) {

    double pos_11_12_step = get_command("joint_ft_11_12/position") * RAD_TO_POSITION_COUNT + POSITION_ZERO;
    double pos_12_13_step = get_command("joint_ft_12_13/position") * RAD_TO_POSITION_COUNT + POSITION_ZERO;
    double pos_13_14_step = get_command("joint_ft_13_14/position") * RAD_TO_POSITION_COUNT + POSITION_ZERO;
    double pos_14_15_step = get_command("joint_ft_14_15/position") * RAD_TO_POSITION_COUNT + POSITION_ZERO;
    double pos_15_16_step = get_command("joint_ft_15_16/position") * RAD_TO_POSITION_COUNT + POSITION_ZERO;
    double pos_16_17_step = get_command("joint_ft_16_17/position") * RAD_TO_POSITION_COUNT + POSITION_ZERO;

    // 检查是否有任何命令值为 NaN（可能是未初始化的命令）
    if (std::isnan(pos_11_12_step) || std::isnan(pos_12_13_step) || std::isnan(pos_13_14_step) ||
        std::isnan(pos_14_15_step) || std::isnan(pos_15_16_step) || std::isnan(pos_16_17_step)) {
        return hardware_interface::return_type::OK;
    }

    std::array<s16, 6> positions = {
        static_cast<s16>(pos_11_12_step), static_cast<s16>(pos_12_13_step), static_cast<s16>(pos_13_14_step),
        static_cast<s16>(pos_14_15_step), static_cast<s16>(pos_15_16_step), static_cast<s16>(pos_16_17_step),
    };
    std::array<u16, 6> speeds{};
    std::array<u8, 6> accs{};
    speeds.fill(0);
    accs.fill(0);

    sms_sts_->SyncWritePosEx(motor_ids_.data(), motor_ids_.size(), positions.data(), speeds.data(), accs.data());

    return hardware_interface::return_type::OK;
}
} // namespace ft3215_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ft3215_hardware::FT3215HardwareInterface, hardware_interface::SystemInterface)
