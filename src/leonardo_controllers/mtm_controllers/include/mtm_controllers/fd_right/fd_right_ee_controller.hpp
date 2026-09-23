#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <controller_interface/chainable_controller_interface.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "mtm_controllers/common/state_interface_utils.hpp"
#include "mtm_controllers/common/visibility_control.hpp"

namespace mtm_controllers {
/**
 * FDRightEeController
 *
 * 链式控制器：读取右侧主手关节状态，计算末端执行器的位姿、RPY、按钮状态，
 * 通过引用接口（ee/）导出给下游从端控制器。
 *
 * 导出的引用接口（相对名，完整名为 <控制器名>/<相对名>）：
 *   ee/x, ee/y, ee/z
 *   ee/qw, ee/qx, ee/qy, ee/qz
 *   ee/roll, ee/pitch, ee/yaw
 *   ee/button
 */
class FDRightEeController : public controller_interface::ChainableControllerInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FDRightEeController);

    MTM_CONTROLLERS_PUBLIC
    FDRightEeController() = default;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  protected:
    MTM_CONTROLLERS_PUBLIC
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_reference_interfaces_list() override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time &time,
                                                                        const rclcpp::Duration &period) override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::return_type update_and_write_commands(const rclcpp::Time &time,
                                                                const rclcpp::Duration &period) override;

    // 引用接口索引
    enum RefIdx : size_t {
        EE_X = 0,
        EE_Y = 1,
        EE_Z = 2,
        EE_QW = 3,
        EE_QX = 4,
        EE_QY = 5,
        EE_QZ = 6,
        EE_ROLL = 7,
        EE_PITCH = 8,
        EE_YAW = 9,
        EE_BUTTON = 10,
        REF_SIZE = 11,
    };

    // ---- 参数 ----
    std::vector<std::string> joints_;
    std::vector<std::string> buttons_;
    Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

    // ---- 引用接口存储 ----
    std::vector<double> reference_interfaces_;

    // ---- 状态接口缓存 ----
    std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;
};
} // namespace mtm_controllers
