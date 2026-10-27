#pragma once

#include "mtm_controllers/common/visibility_control.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <controller_interface/chainable_controller_interface.hpp>
#include <memory>
#include <string>
#include <vector>

namespace mtm_controllers {
/**
 * FDLeftEeController
 *
 * 链式控制器：读取左侧主手关节状态，计算末端执行器的位姿、RPY、夹爪状态，
 * 通过引用接口（ee/前缀）导出给下游从端控制器。
 *
 * 导出的引用接口（相对名，完整名为 <控制器名>/<相对名>）：
 *   ee/x, ee/y, ee/z
 *   ee/qw, ee/qx, ee/qy, ee/qz
 *   ee/roll, ee/pitch, ee/yaw
 *   ee/gripper
 *
 * 状态接口绑定策略：
 *   - on_init 读取 joints 参数并做基本校验；
 *   - on_activate 时按 (joint_name, HW_IF_POSITION) 在 state_interfaces_ 中查找下标，
 *     绑定到 state_index_；任一关节绑定失败则激活失败；
 *   - update 循环只按下标直读，不再做字符串比较 / 哈希查找。
 */
class FDLeftEeController : public controller_interface::ChainableControllerInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(FDLeftEeController);

    MTM_CONTROLLERS_PUBLIC
    FDLeftEeController() = default;

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
    controller_interface::return_type update_reference_from_subscribers(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    MTM_CONTROLLERS_PUBLIC
    controller_interface::return_type update_and_write_commands(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /**
     * 按 joints_name_ 在 state_interfaces_ 中查找 position 接口下标，填充 state_index_。
     * 任一关节找不到则返回 false。
     */
    bool bind_state_indices();

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
        EE_GRIPPER = 10,
        REF_SIZE = 11,
    };

    // ---- 参数 ----
    std::vector<std::string> joints_name_;
    Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

    // ---- 引用接口存储（大小固定 REF_SIZE，on_init 中一次性分配，之后不 resize）----
    std::vector<double> reference_interfaces_;

    // ---- 状态接口下标缓存（-1 表示未绑定）----
    std::vector<int> state_index_;
};
} // namespace mtm_controllers