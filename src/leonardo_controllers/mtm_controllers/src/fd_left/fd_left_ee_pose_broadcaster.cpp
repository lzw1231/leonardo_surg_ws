#include "mtm_controllers/fd_left/fd_left_ee_pose_broadcaster.hpp"


namespace mtm_controllers{
    FDLeftEePoseBroadcaster::FDLeftEePoseBroadcaster() {}

    controller_interface::CallbackReturn FDLeftEePoseBroadcaster::on_init() {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration FDLeftEePoseBroadcaster::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::InterfaceConfiguration FDLeftEePoseBroadcaster::state_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {}
        };
    }

    controller_interface::CallbackReturn FDLeftEePoseBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FDLeftEePoseBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FDLeftEePoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type FDLeftEePoseBroadcaster::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        return controller_interface::return_type::OK;
    }
} // namespace mtm_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mtm_controllers::FDLeftEePoseBroadcaster, controller_interface::ControllerInterface)
