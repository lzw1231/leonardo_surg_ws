#include "mtm_hardware/fd_base/fd_hardware_base.hpp"
#include "mtm_hardware/common/math_utils.hpp"


namespace fd_hardware_base{
    hardware_interface::CallbackReturn FDHardwareBase::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
        auto based_logger = rclcpp::get_logger(logger_name_);

        auto ret = hardware_interface::SystemInterface::on_init(params);
        if (ret != hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(based_logger, "SystemInterface 初始化失败");
            return ret;
        }

        info_ = get_hardware_info();
        logger_name_ = info_.name;
        auto logger = rclcpp::get_logger(logger_name_);

        for (const auto& joint : info_.joints) {
            if (!validateJointInterfaces(joint)) {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        for (const auto& gpio : info_.gpios) {
            if (!validateGpioInterfaces(gpio)) {
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        parseHardwareParameters();

        hw_math::resize_and_fill(hw_states_position_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_states_velocity_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_states_effort_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_commands_effort_, info_.joints.size(), BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_states_inertia_, INERTIA_MATRIX_FLATTEN_SIZE, BUF_INIT_NAN);
        hw_math::resize_and_fill(hw_button_state_, info_.gpios.size(), BUF_INIT_NAN);

        if (emulate_button_ && (info_.joints.size() == 4 || info_.joints.size() > 6)) {
            hw_commands_effort_[info_.joints.size() - 1U] = 0.0;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn FDHardwareBase::on_configure(const rclcpp_lifecycle::State& previous_state) {
        auto logger = rclcpp::get_logger(logger_name_);
        auto ret = hardware_interface::SystemInterface::on_configure(previous_state);
        if (ret != hardware_interface::CallbackReturn::SUCCESS) {
            RCLCPP_ERROR(logger, "SystemInterface 配置失败");
            return ret;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> FDHardwareBase::on_export_state_interfaces() {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
        state_if_storage_.clear();

        const size_t inertia_elements = INERTIA_MATRIX_FLATTEN_SIZE;
        size_t total_size = info_.joints.size() * 3 + info_.gpios.size() + inertia_elements;
        state_interfaces.reserve(total_size);
        state_if_storage_.reserve(total_size);

        for (size_t i = 0; i < info_.joints.size(); ++i) {
            const auto& joint_name = info_.joints[i].name;

            auto if_pos = std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]);
            state_if_storage_.push_back(if_pos);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_pos));

            auto if_vel = std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]);
            state_if_storage_.push_back(if_vel);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_vel));

            auto if_eff = std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]);
            state_if_storage_.push_back(if_eff);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_eff));
        }

        for (size_t i = 0; i < info_.gpios.size(); ++i) {
            const auto& gpio_name = info_.gpios[i].name;
            auto if_btn = std::make_shared<hardware_interface::StateInterface>(
                gpio_name, hardware_interface::HW_IF_POSITION, &hw_button_state_[i]);
            state_if_storage_.push_back(if_btn);
            state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_btn));
        }

        for (uint row = 0; row < 6; ++row) {
            for (uint col = row; col < 6; ++col) {
                size_t idx = hw_math::flattened_index_from_triangular_index(row, col);
                std::string iname = inertia_interface_name_ + "_" + std::to_string(row) + "_" + std::to_string(col);
                auto if_inert = std::make_shared<hardware_interface::StateInterface>(
                    inertia_interface_name_, iname, &hw_states_inertia_[idx]);
                state_if_storage_.push_back(if_inert);
                state_interfaces.push_back(std::const_pointer_cast<const hardware_interface::StateInterface>(if_inert));
            }
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> FDHardwareBase::on_export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface::SharedPtr> cmds;
        command_storage_.clear();
        cmds.reserve(info_.joints.size());
        for (uint i = 0; i < info_.joints.size(); ++i) {
            auto c = std::make_shared<hardware_interface::CommandInterface>(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]);
            command_storage_.push_back(c);
            cmds.push_back(c);
        }
        return cmds;
    }

    hardware_interface::CallbackReturn FDHardwareBase::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        auto logger = rclcpp::get_logger(logger_name_);

        RCLCPP_INFO(logger, "正在激活硬件接口...");
        if (!connectToDevice()) {
            RCLCPP_ERROR(logger, "硬件设备连接失败");
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(logger, "硬件设备连接成功");

        // 连接成功后，对主动轴执行自动校准（仅平移轴，腕部仍需手动校准）
        if (drdAutoInit(interface_id_) < 0) {
            RCLCPP_WARN(logger, "drdAutoInit 失败: %s", dhdErrorGetLastStr());
        } else if (drdCheckInit(interface_id_) < 0) {
            RCLCPP_WARN(logger, "drdCheckInit 失败: %s", dhdErrorGetLastStr());
        } else {
            RCLCPP_INFO(logger, "主动轴自动校准完成");
        }

        if (drdStop(true, interface_id_) < 0) {
            RCLCPP_WARN(logger, "drdStop 失败: %s", dhdErrorGetLastStr());
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn
    FDHardwareBase::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        auto logger = rclcpp::get_logger(logger_name_);
        RCLCPP_INFO(logger, "正在停用硬件接口...");
        if (disconnectFromDevice()) {
            RCLCPP_INFO(logger, "硬件设备断开成功");
            return hardware_interface::CallbackReturn::SUCCESS;
        } else {
            RCLCPP_ERROR(logger, "硬件设备断开失败");
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    bool FDHardwareBase::connectToDevice() {
        auto logger = rclcpp::get_logger(logger_name_);

        int major, minor, release, revision;
        dhdGetSDKVersion(&major, &minor, &release, &revision);
        RCLCPP_INFO(logger, "DHD SDK 版本: %d.%d (Release %d / Revision %d)", major, minor, release, revision);

        bool dhd_open_success = false;
        if (interface_sn_ >= 0) {
            RCLCPP_INFO(logger, "尝试通过序列号 %d 打开设备...", interface_sn_);
            interface_id_ = static_cast<char>(dhdOpenSerial(interface_sn_));
            dhd_open_success = (static_cast<int>(interface_id_) >= 0);
        }

        if (!dhd_open_success) {
            RCLCPP_ERROR(logger, "打开设备失败");
            isConnected_ = false;
            return false;
        }

        // 让 DRD 接管这台设备
        int drd_id = drdOpenID(interface_id_);
        if (drd_id < 0) {
            RCLCPP_WARN(logger, "drdOpenID 失败: %s", dhdErrorGetLastStr());
        } else {
            RCLCPP_INFO(logger, "drdOpenID 成功, drd_id=%d", drd_id);
            interface_id_ = static_cast<char>(drd_id);
        }

        RCLCPP_INFO(logger, "设备名称: %s", dhdGetSystemName(interface_id_));

        uint16_t serialNumber = 0;
        if (dhdGetSerialNumber(&serialNumber, interface_id_) != DHD_NO_ERROR) {
            RCLCPP_WARN(logger, "无法获取序列号: %s", dhdErrorGetLastStr());
        } else {
            RCLCPP_INFO(logger, "设备序列号: %d", serialNumber);
        }
        RCLCPP_INFO(logger, "内部接口 ID: %d", static_cast<int>(interface_id_));

        if (dhdHasWrist(interface_id_)) {
            RCLCPP_INFO(logger, "检测到腕部自由度");
        } else {
            RCLCPP_INFO(logger, "未检测到腕部自由度");
        }

        double current_effector_mass = 0.0;
        if (dhdGetEffectorMass(&current_effector_mass, interface_id_) == DHD_NO_ERROR) {
            RCLCPP_INFO(logger, "当前末端质量: %.2f g", current_effector_mass * 1000.0);
        } else {
            RCLCPP_WARN(logger, "无法获取末端质量");
        }

        if (dhdSetMaxForce(DEFAULT_MAX_FORCE, interface_id_) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "设置最大力失败");
            disconnectFromDevice();
            return false;
        }

        dhdSetBrakes(DHD_OFF, interface_id_);

        if (dhdEnableForce(DHD_ON, interface_id_) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "启用力反馈失败");
            disconnectFromDevice();
            return false;
        }

        if (effector_mass_ > 0.0) {
            RCLCPP_INFO(logger, "更新末端质量: %.2f g -> %.2f g", current_effector_mass * 1000.0, effector_mass_ * 1000.0);
            if (dhdSetEffectorMass(effector_mass_, interface_id_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(logger, "设置末端质量失败");
                disconnectFromDevice();
                return false;
            }
        }

        if (dhdSetGravityCompensation(DHD_ON, interface_id_) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "开启重力补偿失败");
            disconnectFromDevice();
            return false;
        }
        RCLCPP_INFO(logger, "重力补偿已开启");

        if (emulate_button_ && !dhdHasGripper(interface_id_)) {
            RCLCPP_ERROR(logger, "启用按键模拟但设备无夹爪");
        } else if (emulate_button_ && dhdHasGripper(interface_id_)) {
            RCLCPP_INFO(logger, "设备带有夹爪，启用按键模拟");
            if (dhdEmulateButton(DHD_ON, interface_id_) < DHD_NO_ERROR) {
                RCLCPP_ERROR(logger, "启用按键模拟失败");
                disconnectFromDevice();
                return false;
            }
            RCLCPP_INFO(logger, "按键模拟功能已激活");
        }

        if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, interface_id_) < DHD_NO_ERROR) {
            RCLCPP_ERROR(logger, "初始化力输出失败");
            disconnectFromDevice();
            return false;
        }

        ignore_orientation_ |= !dhdHasWrist(interface_id_);
        if (ignore_orientation_) {
            RCLCPP_INFO(logger, "因硬件限制，忽略姿态读数");
        }

        dhdSleep(0.1);
        isConnected_ = true;
        RCLCPP_INFO(logger, "设备基本配置完成");
        return true;
    }

    // 用 DRD 关闭设备（设备已归 DRD 接管）
    bool FDHardwareBase::disconnectFromDevice() {
        auto logger = rclcpp::get_logger(logger_name_);
        if (!isConnected_) {
            return true;
        }

        if (drdClose(interface_id_) < 0) {
            RCLCPP_ERROR(logger, "drdClose 失败: %s", dhdErrorGetLastStr());
            return false;
        }

        RCLCPP_INFO(logger, "DHD 设备已关闭");
        interface_id_ = -1;
        isConnected_ = false;
        return true;
    }

    bool FDHardwareBase::validateJointInterfaces(const hardware_interface::ComponentInfo& joint) {
        auto logger = rclcpp::get_logger(logger_name_);
        if (joint.command_interfaces.size() != 1U) {
            RCLCPP_FATAL(logger, "Joint '%s' 期望 1 个命令接口，实际获取 %zu 个", joint.name.c_str(),
                         joint.command_interfaces.size());
            return false;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(logger, "Joint '%s' 命令接口类型错误: 期望 '%s', 实际 '%s'", joint.name.c_str(),
                         hardware_interface::HW_IF_EFFORT,
                         joint.command_interfaces[0].name.c_str());
            return false;
        }
        if (joint.state_interfaces.size() != 3U) {
            RCLCPP_FATAL(logger, "Joint '%s' 期望 3 个状态接口，实际获取 %zu 个", joint.name.c_str(), joint.state_interfaces.size());
            return false;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(logger, "Joint '%s' 状态接口[0]类型错误: 期望 '%s', 实际 '%s'", joint.name.c_str(),
                         hardware_interface::HW_IF_POSITION,
                         joint.state_interfaces[0].name.c_str());
            return false;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(logger, "Joint '%s' 状态接口[1]类型错误: 期望 '%s', 实际 '%s'", joint.name.c_str(),
                         hardware_interface::HW_IF_VELOCITY,
                         joint.state_interfaces[1].name.c_str());
            return false;
        }
        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            RCLCPP_FATAL(logger, "Joint '%s' 状态接口[2]类型错误: 期望 '%s', 实际 '%s'", joint.name.c_str(),
                         hardware_interface::HW_IF_EFFORT,
                         joint.state_interfaces[2].name.c_str());
            return false;
        }
        return true;
    }

    bool FDHardwareBase::validateGpioInterfaces(const hardware_interface::ComponentInfo& button) {
        auto logger = rclcpp::get_logger(logger_name_);
        if (button.state_interfaces.size() != 1U) {
            RCLCPP_FATAL(logger, "GPIO '%s' 拥有 %zu 个 State 接口，期望值为 1", button.name.c_str(),
                         button.state_interfaces.size());
            return false;
        }
        if (button.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(logger, "GPIO '%s' State 类型为 [%s]，期望值为 [%s]", button.name.c_str(),
                         button.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return false;
        }
        return true;
    }

    void FDHardwareBase::parseHardwareParameters() {
        auto logger = rclcpp::get_logger(logger_name_);

        auto it_id = info_.hardware_parameters.find("interface_id");
        if (it_id != info_.hardware_parameters.end()) {
            interface_id_ = static_cast<char>(std::stoi(it_id->second));
            RCLCPP_INFO(logger, "配置接口 ID: %d", static_cast<int>(interface_id_));
        } else {
            interface_id_ = -1;
        }

        auto it_sn = info_.hardware_parameters.find("interface_serial_number");
        if (it_sn != info_.hardware_parameters.end()) {
            interface_sn_ = std::stoi(it_sn->second);
            RCLCPP_INFO(logger, "配置序列号 SN: %d", interface_sn_);
        } else {
            interface_sn_ = -1;
            RCLCPP_WARN(logger, "hardware_parameters 中未找到 interface_serial_number");
        }

        auto it_emu_btn = info_.hardware_parameters.find("emulate_button");
        if (it_emu_btn != info_.hardware_parameters.end()) {
            emulate_button_ = hardware_interface::parse_bool(it_emu_btn->second);
        } else {
            emulate_button_ = false;
        }
        RCLCPP_INFO(logger, "按键模拟开启: %s", emulate_button_ ? "true" : "false");

        auto it_inertia_name = info_.hardware_parameters.find("inertia_interface_name");
        if (it_inertia_name != info_.hardware_parameters.end()) {
            inertia_interface_name_ = it_inertia_name->second;
        } else {
            inertia_interface_name_ = "fd_inertia";
        }
        RCLCPP_INFO(logger, "惯性矩阵接口名: %s", inertia_interface_name_.c_str());

        auto it_mass = info_.hardware_parameters.find("effector_mass");
        if (it_mass != info_.hardware_parameters.end()) {
            effector_mass_ = hardware_interface::stod(it_mass->second);
            RCLCPP_INFO(logger, "末端执行器质量: %lf Kg", effector_mass_);
        } else {
            effector_mass_ = -1.0;
        }

        auto it_ignore_ori = info_.hardware_parameters.find("ignore_orientation_readings");
        if (it_ignore_ori != info_.hardware_parameters.end()) {
            ignore_orientation_ = hardware_interface::parse_bool(it_ignore_ori->second);
        } else {
            ignore_orientation_ = false;
        }
        RCLCPP_INFO(logger, "忽略姿态读数: %s", ignore_orientation_ ? "true" : "false");
    }
} // namespace fd_hardware_base
