#pragma once
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include "mtm_controllers/common/visibility_control.hpp"

namespace mtm_controllers{
    class FDLeftEePoseBroadcaster : public controller_interface::ControllerInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftEePoseBroadcaster);

        MTM_CONTROLLERS_PUBLIC
        FDLeftEePoseBroadcaster();

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    };
} // namespace mtm_controllers



