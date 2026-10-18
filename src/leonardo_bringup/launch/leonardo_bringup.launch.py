import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动双 Omega.7 力反馈设备、PSM 从手、ros2_control 与 RViz。"""

    share_desc = get_package_share_directory("leonardo_description")
    share_bringup = get_package_share_directory("leonardo_bringup")

    robot_desc_file_path = os.path.join(share_desc, "urdf", "leonardo.urdf.xacro")
    rviz_config_file_path = os.path.join(share_desc, "mtm_description", "rviz", "fd_bimanual_config.rviz")

    # YAML 路径
    controllers_yaml_path = os.path.join(share_bringup, "config", "leonardo_controllers.yaml")

    ns = "leonardo"
    cm = f"/{ns}/controller_manager"

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

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=["joint_state_broadcaster", "--controller-manager", cm],
        output="screen",
    )

    fd_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "fd_left_ee_controller",
            "--controller-manager",
            cm,
            "--controller-ros-args",
            f"--params-file {controllers_yaml_path}",
        ],
        output="screen",
    )

    fd_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "fd_right_ee_controller",
            "--controller-manager",
            cm,
            "--controller-ros-args",
            f"--params-file {controllers_yaml_path}",
        ],
        output="screen",
    )

    mapping_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "pedal_mapping_controller",
            "--controller-manager",
            cm,
            "--controller-ros-args",
            f"--params-file {controllers_yaml_path}",
        ],
        output="screen",
    )

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
            jsb_spawner,
            fd_left_spawner,
            fd_right_spawner,
            mapping_spawner,
            pedal_publisher_node,
            rviz_node,
        ]
    )
