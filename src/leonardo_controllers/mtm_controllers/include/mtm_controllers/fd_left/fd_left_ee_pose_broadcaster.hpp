#pragma once

#include <controller_interface/controller_interface.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/bool.hpp>

#include "mtm_controllers/common/state_interface_utils.hpp"
#include "mtm_controllers/common/visibility_control.hpp"

namespace mtm_controllers{
    /**
     * @brief Force Dimension 左末端执行器位姿广播控制器。
     */
    class FDLeftEePoseBroadcaster : public controller_interface::ControllerInterface {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftEePoseBroadcaster);

        // 构造函数
        MTM_CONTROLLERS_PUBLIC
        FDLeftEePoseBroadcaster() = default;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        MTM_CONTROLLERS_PUBLIC
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

    protected:
        std::vector<std::string> joints_;
        std::vector<std::string> buttons_;

        Eigen::Matrix4d pose_;
        Eigen::Matrix4d transform_;

        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> ee_pose_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> realtime_ee_pose_publisher_;

        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> button_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> realtime_button_publisher_;

        std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;
    };
} // namespace mtm_controllers
