import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动双 Omega.7 力反馈设备、PSM 从手、ros2_control 与 RViz。"""

    share_desc = get_package_share_directory("leonardo_description")
    share_bringup = get_package_share_directory("leonardo_bringup")

    robot_desc_file_path = os.path.join(share_desc, "urdf", "leonardo.urdf.xacro")
    rviz_config_file_path = os.path.join(share_bringup, "rviz", "leonardo_config.rviz")

    # YAML 路径
    controllers_yaml_path = os.path.join(share_bringup, "config", "leonardo_controllers.yaml")

    ns = "leonardo"
    cm = f"/{ns}/controller_manager"

    # =========================================================================
    # 基础节点：robot_state_publisher + controller_manager
    # =========================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", robot_desc_file_path]), value_type=str)}],
        remappings=[("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")],
        output="screen",
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[controllers_yaml_path],
        output="screen",
    )

    # =========================================================================
    # 阶段 1：一次性加载所有控制器，全部 --inactive（只 load + configure）
    # 不再用多个并行 spawner，避免"上游早于下游激活"的时序竞争。
    # =========================================================================
    load_all_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "joint_state_broadcaster",
            "fd_left_ee_controller",
            "fd_right_ee_controller",
            "pedal_mapping_controller",
            "--controller-manager",
            cm,
            "--inactive",
            "--controller-ros-args",
            f"--params-file {controllers_yaml_path}",
        ],
        output="screen",
    )

    # =========================================================================
    # 阶段 2：按依赖顺序一次性激活
    #   jsb（独立）
    #   → fd_left / fd_right（下游，导出 reference interfaces）
    #   → pedal_mapping（上游，claim fd 导出的 ee/* 接口）
    # controller_manager 内部会按链式依赖正确处理。
    # =========================================================================
    activate_all_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "-c",
            cm,
            "--strict",
            "--activate",
            "joint_state_broadcaster",
            "fd_left_ee_controller",
            "fd_right_ee_controller",
            "pedal_mapping_controller",
        ],
        output="screen",
    )

    # load 完成 → 再执行激活
    activate_after_load = RegisterEventHandler(
        OnProcessExit(
            target_action=load_all_controllers_spawner,
            on_exit=[activate_all_controllers],
        )
    )

    # =========================================================================
    # 其它节点：踏板发布、RViz
    # =========================================================================
    pedal_publisher_node = Node(
        package="leonardo_teleop",
        executable="pedal_publisher",
        namespace=ns,
        parameters=[controllers_yaml_path],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        arguments=["-d", rviz_config_file_path],
        remappings=[
            ("/tf", f"/{ns}/tf"),
            ("/tf_static", f"/{ns}/tf_static"),
            ("robot_description", f"/{ns}/robot_description"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager_node,
            load_all_controllers_spawner,
            activate_after_load,
            pedal_publisher_node,
            rviz_node,
        ]
    )
