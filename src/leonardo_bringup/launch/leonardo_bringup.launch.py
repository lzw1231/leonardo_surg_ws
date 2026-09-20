import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动双 Omega.7 力反馈设备、FT3215 从手、ros2_control 与 RViz。"""

    # ==========================================================================
    # 资源路径
    # ==========================================================================
    robot_desc_file_path = os.path.join(
        get_package_share_directory("leonardo_description"),
        "urdf",
        "leonardo_urdf.xacro",
    )

    # controller_manager 的全局参数（update_rate + 所有控制器的 type）
    controller_manager_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"),
        "config",
        "leonardo_controller_manager.yaml",
    )
    # MTM 控制器参数
    mtm_controllers_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"),
        "config",
        "mtm_controllers.yaml",
    )
    # PSM 控制器参数
    psm_controllers_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"),
        "config",
        "psm_controllers.yaml",
    )
    # 遥操作辅助节点参数（topic_router）
    teleop_params_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"),
        "config",
        "teleop_params.yaml",
    )

    rviz_config_file_path = os.path.join(
        get_package_share_directory("leonardo_description"),
        "mtm_description",
        "rviz",
        "fd_bimanual_config.rviz",
    )

    # ==========================================================================
    # 命名空间：统一为 leonardo
    # ==========================================================================
    ns = "leonardo"

    # ==========================================================================
    # robot_state_publisher
    # ==========================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_desc_file_path]),
                    value_type=str,
                )
            }
        ],
        remappings=[
            ("/tf", "/leonardo/tf"),
            ("/tf_static", "/leonardo/tf_static"),
        ],
    )

    # ==========================================================================
    # controller_manager（唯一一个，加载全部 YAML）
    # ==========================================================================
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[
            controller_manager_yaml_path,
            mtm_controllers_yaml_path,
            psm_controllers_yaml_path,
        ],
    )

    # ==========================================================================
    # Spawner：MTM 相关
    # ==========================================================================
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["joint_state_broadcaster"],
    )

    fd_left_effort_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["fd_left_effort_controller"],
    )

    fd_left_ee_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["fd_left_ee_broadcaster"],
    )

    fd_right_effort_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["fd_right_effort_controller"],
    )

    fd_right_ee_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["fd_right_ee_broadcaster"],
    )

    # ==========================================================================
    # Spawner：PSM 相关
    # ==========================================================================
    ft3215_controller_a_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["ft3215_controller_a"],
    )

    ft3215_controller_b_node = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["ft3215_controller_b"],
    )

    # ==========================================================================
    # 话题路由器（独立节点，不走 controller_manager）
    # ==========================================================================
    topic_router_node = Node(
        package="leonardo_teleop",
        executable="topic_router",
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
            # MTM spawner
            joint_state_broadcaster_node,
            fd_left_effort_controller_node,
            fd_left_ee_broadcaster_node,
            fd_right_effort_controller_node,
            fd_right_ee_broadcaster_node,
            # PSM spawner
            ft3215_controller_a_node,
            ft3215_controller_b_node,
            # TOPIC 路由器
            topic_router_node,
            # RViz
            rviz_node,
        ]
    )
