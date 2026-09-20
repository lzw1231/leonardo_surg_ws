#pragma once

#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include "psm_controllers/common/visibility_control.hpp"

using Float64MultiArray = example_interfaces::msg::Float64MultiArray;

namespace psm_controllers{
    class FT3215AController : public controller_interface::ControllerInterface {
    public:
        PSM_CONTROLLERS_PUBLIC
        FT3215AController() = default;

        PSM_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        PSM_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        PSM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_init() override;
        PSM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        PSM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        PSM_CONTROLLERS_PUBLIC
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    protected:
        std::vector<std::string> joint_names_;
        std::string interface_name_;
        std::string topic_;
        double coefficient_;
        realtime_tools::RealtimeBuffer<std::vector<double>> rt_command_buffer_;

        rclcpp::Subscription<Float64MultiArray>::SharedPtr command_subscriber_;
    }; // class FT3215AController
} // psm_controllers
