import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动双 Omega.7 力反馈设备、FT3215 从手、ros2_control 与 RViz。"""

    # ==========================================================================
    # 资源路径
    # ==========================================================================
    robot_desc_file_path = os.path.join(
        get_package_share_directory("leonardo_description"), "urdf", "leonardo_urdf.xacro"
    )
    controller_manager_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"), "config", "manager", "leonardo_controller_manager.yaml"
    )
    mtm_controllers_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"), "config", "mtm", "fd_controllers.yaml"
    )
    psm_controllers_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"), "config", "psm", "ft3215_controllers.yaml"
    )
    teleop_params_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"), "config", "teleop", "teleop_params.yaml"
    )
    rviz_config_file_path = os.path.join(
        get_package_share_directory("leonardo_description"), "mtm_description", "rviz", "fd_bimanual_config.rviz"
    )

    ns = "leonardo"
    cm_name = f"/{ns}/controller_manager"

    # ==========================================================================
    # 基础节点
    # ==========================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", robot_desc_file_path]), value_type=str)}],
        remappings=[("/tf", "/leonardo/tf"), ("/tf_static", "/leonardo/tf_static")],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[controller_manager_yaml_path],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["joint_state_broadcaster"],
    )

    # ==========================================================================
    # 主手链式控制器
    # ==========================================================================
    fd_left_ee_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "fd_left_ee_controller",
            "--controller-manager",
            cm_name,
            "--controller-ros-args",
            f"--params-file {mtm_controllers_yaml_path}",
        ],
    )

    fd_right_ee_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "fd_right_ee_controller",
            "--controller-manager",
            cm_name,
            "--controller-ros-args",
            f"--params-file {mtm_controllers_yaml_path}",
        ],
    )

    # ==========================================================================
    # 从手控制器（不直接进 LaunchDescription，等事件触发）
    # ==========================================================================
    ft3215_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "ft3215_controller",
            "--controller-manager",
            cm_name,
            "--controller-ros-args",
            f"--params-file {psm_controllers_yaml_path}",
        ],
    )

    # ==========================================================================
    # 事件链：等左右主手 spawner 都成功退出后，才启动 ft3215
    # ==========================================================================
    _done = {"left": False, "right": False, "ft3215": False}

    def _start_ft3215_if_both_done(_context):
        if _done["left"] and _done["right"] and not _done["ft3215"]:
            _done["ft3215"] = True
            return [ft3215_controller_node]
        return []

    def _on_left_exit(_context):
        _done["left"] = True
        return [OpaqueFunction(function=_start_ft3215_if_both_done)]

    def _on_right_exit(_context):
        _done["right"] = True
        return [OpaqueFunction(function=_start_ft3215_if_both_done)]

    ft3215_trigger = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=fd_left_ee_controller_node,
                on_exit=[OpaqueFunction(function=_on_left_exit)],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=fd_right_ee_controller_node,
                on_exit=[OpaqueFunction(function=_on_right_exit)],
            )
        ),
    ]

    # ==========================================================================
    # 辅助节点
    # ==========================================================================
    topic_router_node = Node(
        package="leonardo_teleop",
        executable="topic_router",
        namespace=ns,
        parameters=[teleop_params_yaml_path],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        arguments=["-d", rviz_config_file_path],
        remappings=[
            ("/tf", "/leonardo/tf"),
            ("/tf_static", "/leonardo/tf_static"),
            ("robot_description", "/leonardo/robot_description"),
        ],
    )

    # ==========================================================================
    # LaunchDescription
    # ==========================================================================
    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_node,
            fd_left_ee_controller_node,
            fd_right_ee_controller_node,
            *ft3215_trigger,
            topic_router_node,
            rviz_node,
        ]
    )
