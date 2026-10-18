import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 1. 构建 xacro 文件的完整路径
    xacro_file = os.path.join(get_package_share_directory("leonardo_description"), "urdf", "leonardo.urdf.xacro")

    # 2. 构建 rviz 配置文件的完整路径
    rviz_config_file = os.path.join(get_package_share_directory("leonardo_bringup"), "rviz", "view_snakes_des.rviz")

    # 3. 启动 robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", xacro_file]), value_type=str)}],
    )

    # 4. 启动 joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # 5. 启动 RViz2 并加载指定配置
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],  # 👈 关键：-d 参数指定配置文件
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
