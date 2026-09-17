import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 双手 xacro 顶层入口
    robot_description_xacro = os.path.join(
        get_package_share_directory("mtm_description"),
        "urdf",
        "fd_bimanual.config.xacro",
    )

    # RViz 配置文件
    robot_config_rviz = os.path.join(
        get_package_share_directory("mtm_description"),
        "rviz",
        "fd_bimanual_config.rviz",
    )

    # robot_state_publisher：解析 xacro 并发布 robot_description 与 TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_description_xacro]),
                    value_type=str,
                )
            }
        ],
        output="screen",
    )

    # 调试用：把展开后的 URDF 落盘，方便检查
    dump_urdf_node = ExecuteProcess(
        cmd=["xacro", robot_description_xacro, "-o", "/tmp/fd_bimanual_expanded.urdf"],
        output="screen",
    )

    # RViz2 可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", robot_config_rviz],
        output="screen",
    )

    # 关节角 GUI：手动拖动关节，观察姿态（仅可视化用，不接硬件）
    joint_state_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            dump_urdf_node,
            rviz_node,
            joint_state_gui_node,
        ]
    )
