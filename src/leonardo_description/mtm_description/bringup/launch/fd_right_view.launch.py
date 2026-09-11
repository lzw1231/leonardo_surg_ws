#!/usr/bin/env python3
"""
display_fd_right.launch.py
用途：Force‑Dimension右手主手(MTM)离线可视化调试
功能：加载xacro模型、robot_state_publisher、RViz、关节调试面板
注意：仅可视化，不启动硬件驱动与ros2_control
ROS2 Jazzy
"""
import os
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # fd_right 的 xacro 模型路径
    robot_description_xacro = os.path.join(get_package_share_path("raphael_description"), 'urdf', 'MTM', 'fd_right', 'fd_right.config.xacro')

    # RViz 的 config 路径
    robot_config_rviz = os.path.join(get_package_share_path("raphael_description"), 'rviz', 'fd_right_config.rviz')

    # 机器人状态发布：编译xacro，输出robot_description与TF树
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(Command(["xacro ", robot_description_xacro]), value_type=str)}],
        output="screen"
    )

    # RViz2 可视化配置
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', robot_config_rviz],
        output="screen"
    )

    # 关节状态调试GUI：手动修改关节角度，用于校验模型、连杆、坐标系
    joint_state_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_gui_node
    ])
