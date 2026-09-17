import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动双 Omega.7 力反馈设备、ros2_control 与 RViz。"""

    # 资源路径
    robot_desc_file_path = os.path.join(
        get_package_share_directory("mtm_description"),
        "urdf",
        "fd_bimanual.config.xacro",
    )
    robot_controllers_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"),
        "config",
        "mtm_controllers.yaml",
    )
    rviz_config_file_path = os.path.join(
        get_package_share_directory("mtm_description"),
        "rviz",
        "fd_bimanual_config.rviz",
    )

    # 命名空间：统一所有节点，规避跨命名空间的 remap 依赖
    ns = "mtm_controllers"

    # 机器人模型发布
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
            ('/tf', '/mtm_controllers/tf'),
            ('/tf_static', '/mtm_controllers/tf_static'),
        ]
    )

    # 控制器管理器
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[robot_controllers_yaml_path],
    )

    # 控制器 spawner：与控制器管理器同命名空间，无需指定服务地址
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

    # RViz：内部使用绝对话题名，需重映射至命名空间
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        arguments=["-d", rviz_config_file_path],
        remappings=[
            ("/tf", "/mtm_controllers/tf"),
            ("/tf_static", "/mtm_controllers/tf_static"),
            ("robot_description", "/mtm_controllers/robot_description"),
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_node,
            fd_left_effort_controller_node,
            fd_left_ee_broadcaster_node,
            fd_right_effort_controller_node,
            fd_right_ee_broadcaster_node,
            rviz_node,
        ]
    )
