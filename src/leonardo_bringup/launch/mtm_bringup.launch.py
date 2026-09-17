import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # 文件路径配置
    robot_desc_file_path = os.path.join(
        get_package_share_directory("mtm_description"),
        "urdf",
        "fd_bimanual.urdf.xacro"
    )
    robot_controllers_yaml_path = os.path.join(
        get_package_share_directory("leonardo_bringup"),
        "config",
        "mtm_controllers.yaml"
    )
    rviz_config_file_path = os.path.join(
        get_package_share_directory("mtm_description"),
        "rviz",
        "fd_bimanual_config.rviz"
    )

    # 1. Robot State Publisher：解析xacro发布robot_description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", robot_desc_file_path]),
                    value_type=str
                )
            }
        ]
    )

    # 2. ros2_control 控制器管理器
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers_yaml_path]
    )

    # 3. 首先启动 joint_state_broadcaster
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10.0"
        ]
    )

    # -------- 以下控制器由事件触发，不直接加到LaunchDescription --------
    spawn_fd_left_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fd_left_effort_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10.0"
        ]
    )

    spawn_fd_left_ee_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fd_left_ee_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10.0"
        ]
    )

    spawn_fd_right_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fd_right_effort_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10.0"
        ]
    )

    spawn_fd_right_ee_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "fd_right_ee_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "10.0"
        ]
    )

    # 事件：joint_state_broadcaster进程结束后，启动剩余控制器
    controllers_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[
                spawn_fd_left_effort_controller,
                spawn_fd_left_ee_broadcaster,
                spawn_fd_right_effort_controller,
                spawn_fd_right_ee_broadcaster
            ]
        )
    )

    # 4. RViz可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        spawn_joint_state_broadcaster,
        controllers_event_handler,
        rviz_node
    ])
