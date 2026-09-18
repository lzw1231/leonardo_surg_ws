#pragma once


#include <hardware_interface/system_interface.hpp>
#include "SMS_STS.h"


namespace ft3215{
    class FT3215 : public hardware_interface::SystemInterface {
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
        std::string st3215_port_;
        int baud_ = 0;

        std::array<u8, 5> motor_ids_ = {0, 0, 0, 0, 0};

        // 电机名的枚举
        enum MotorIdx : size_t {
            S1_06_07 = 0,
            S1_07_08,
            S1_08_09,
            S1_09B_10,
            S1_10_11,
            COUNT
        };
    };
} //namespace snake_hardware
