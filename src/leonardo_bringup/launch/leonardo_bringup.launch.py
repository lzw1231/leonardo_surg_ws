import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(__file__))
from launch_utils import spawner, run_after_all


def generate_launch_description():
    """启动双 Omega.7 力反馈设备、FT3215 从手、ros2_control 与 RViz。"""

    # ==========================================================================
    # 资源路径
    # ==========================================================================
    share_desc = get_package_share_directory("leonardo_description")
    share_bringup = get_package_share_directory("leonardo_bringup")

    robot_desc_file_path = os.path.join(share_desc, "urdf", "leonardo.urdf.xacro")
    rviz_config_file_path = os.path.join(share_desc, "mtm_description", "rviz", "fd_bimanual_config.rviz")

    controller_manager_yaml_path = os.path.join(share_bringup, "config", "manager", "leonardo_controller_manager.yaml")
    mtm_controllers_yaml_path = os.path.join(share_bringup, "config", "mtm", "fd_controllers.yaml")
    mapping_controller_yaml_path = os.path.join(share_bringup, "config", "mapping", "pedal_mapping_controller.yaml")
    psm_controllers_yaml_path = os.path.join(share_bringup, "config", "psm", "ft3215_controllers.yaml")
    teleop_params_yaml_path = os.path.join(share_bringup, "config", "teleop", "teleop_params.yaml")

    ns = "leonardo"

    # ==========================================================================
    # 基础节点
    # ==========================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", robot_desc_file_path]), value_type=str)}],
        remappings=[("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[controller_manager_yaml_path],
    )

    # ==========================================================================
    # Spawner 节点
    # ==========================================================================
    spawner_jsb = spawner("joint_state_broadcaster", ns=ns)
    spawner_fd_left = spawner("fd_left_ee_controller", mtm_controllers_yaml_path, ns=ns)
    spawner_fd_right = spawner("fd_right_ee_controller", mtm_controllers_yaml_path, ns=ns)
    spawner_mapping = spawner("pedal_mapping_controller", mapping_controller_yaml_path, ns=ns)
    spawner_ft3215 = spawner("ft3215_controller", psm_controllers_yaml_path, ns=ns)

    # ==========================================================================
    # 事件链：fd_left + fd_right → mapping → ft3215
    # ==========================================================================
    chain_1 = run_after_all([spawner_fd_left, spawner_fd_right], spawner_mapping)
    chain_2 = run_after_all([spawner_mapping], spawner_ft3215)

    # ==========================================================================
    # 踏板发布
    # ==========================================================================
    pedal_publisher_node = Node(
        package="leonardo_teleop",
        executable="pedal_publisher",
        namespace=ns,
        parameters=[teleop_params_yaml_path],
        output="screen",
    )

    # ==========================================================================
    # RViz
    # ==========================================================================
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
    )

    # ==========================================================================
    # LaunchDescription
    # ==========================================================================
    return LaunchDescription(
        [
            # 基础设施
            robot_state_publisher_node,
            controller_manager_node,
            # 状态广播
            spawner_jsb,
            # 左右主手（并行）
            spawner_fd_left,
            spawner_fd_right,
            # 事件链：fd_left+fd_right → mapping → ft3215
            *chain_1,
            *chain_2,
            # 踏板
            pedal_publisher_node,
            # RViz
            rviz_node,
        ]
    )
