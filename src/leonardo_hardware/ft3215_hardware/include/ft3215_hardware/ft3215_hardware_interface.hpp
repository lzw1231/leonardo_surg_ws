#pragma once

#include <hardware_interface/system_interface.hpp>
#include <sts_vendor/SCServo.hpp>


namespace ft3215_hardware{
    class FT3215HardwareInterface : public hardware_interface::SystemInterface {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<SMS_STS> sms_sts_;
        std::string ft3215_port_;
        int baud_ = 0;

        std::array<u8, 6> motor_ids_ = {0, 0, 0, 0, 0, 0};

        // 电机名的枚举
        enum MotorIdx : size_t {
            joint_11_12,
            joint_12_13,
            joint_13_14,
            joint_14_15,
            joint_15_16,
            joint_16_17,
            COUNT
        };
    };
} //namespace ft3215_hardware
