#include "psm_controllers/ft3215/ft3215_a_controller.hpp"
#include <cmath>
#include <algorithm>

namespace psm_controllers{
    controller_interface::CallbackReturn FT3215AController::on_init() {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        interface_name_ = auto_declare<std::string>("interface_name", "position");
        coefficient_ = auto_declare<double>("coefficient", 1.0);
        topic_ = auto_declare<std::string>("topic", "~/joints_command");

        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration FT3215AController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (auto joint_name : joint_names_) {
            config.names.push_back(joint_name + "/" + interface_name_);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration FT3215AController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (auto joint_name : joint_names_) {
            config.names.push_back(joint_name + "/" + interface_name_);
        }
        return config;
    }


    controller_interface::CallbackReturn FT3215AController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        rt_command_buffer_.initRT(std::vector<double>(joint_names_.size(), 0.0));

        rclcpp::QoS qos(10);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        auto callback = [this](const Float64MultiArray::SharedPtr msg) {
            // 1. 检查长度
            if (msg->data.size() != joint_names_.size()) {
                RCLCPP_WARN_THROTTLE(
                    get_node()->get_logger(),
                    *get_node()->get_clock(),
                    1000, // 每秒最多打印一次
                    "Received command size (%zu) != joint count (%zu). Ignoring.",
                    msg->data.size(), joint_names_.size()
                );
                return;
            }

            // 2. 检查所有值是否都是有限数（非 NaN、非 Inf）
            bool all_finite = std::all_of(
                msg->data.begin(),
                msg->data.end(),
                [](double v) { return std::isfinite(v); }
            );
            if (!all_finite) {
                RCLCPP_WARN_THROTTLE(
                    get_node()->get_logger(),
                    *get_node()->get_clock(),
                    1000,
                    "Non-finite value (NaN or Inf) received in command. Ignoring."
                );
                return;
            }

            // 3. 通过检查，写入实时缓冲区

            rt_command_buffer_.writeFromNonRT(msg->data);
        };

        command_subscriber_ = get_node()->create_subscription<Float64MultiArray>(topic_, qos, callback);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn FT3215AController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        (void)previous_state;

        std::vector<double> initial_cmd(joint_names_.size());

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            auto state_opt = state_interfaces_[i].get_optional();
            if (!state_opt.has_value()) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "State interface for joint '%s' not available on activation.",
                             joint_names_[i].c_str());
                return CallbackReturn::FAILURE;
            }
            initial_cmd[i] = state_opt.value();
        }
        rt_command_buffer_.writeFromNonRT(initial_cmd);

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type FT3215AController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        (void)time;
        (void)period;

        const auto& cmd = *rt_command_buffer_.readFromRT();

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            double state = state_interfaces_[i].get_optional().value();
            double new_cmd = cmd[i] * coefficient_ + state * (1.0 - coefficient_);

            if (!command_interfaces_[i].set_value(new_cmd)) {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Failed to set command for joint '%s'. Aborting update.",
                             joint_names_[i].c_str());
                return controller_interface::return_type::ERROR;
            }
        }
        return controller_interface::return_type::OK;
    }
} // namespace ft3215_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(psm_controllers::FT3215AController, controller_interface::ControllerInterface)
