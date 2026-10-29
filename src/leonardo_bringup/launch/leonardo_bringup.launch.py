import os
import signal
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import Command
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


# =========================================================================
# 0. 工具函数
# =========================================================================
def kill_all_and_exit(context, *args, **kwargs):
    """终止当前进程组并退出：用于 Fail-Fast 故障传播。"""
    print("\n" + "=" * 80)
    print("[FATAL] PEDAL NODE FAILED. KILLING ALL PROCESSES IMMEDIATELY.")
    print("=" * 80 + "\n")
    try:
        os.killpg(os.getpgid(0), signal.SIGKILL)
    except (ProcessLookupError, PermissionError) as exc:
        print(f"[FATAL] Failed to kill process group: {exc!r}", file=sys.stderr)
    finally:
        pending = sys.exception()
        if pending is not None:
            print(f"[FATAL] Unexpected error during killpg: {pending!r}", file=sys.stderr)
        sys.exit(1)


def generate_launch_description():
    # =========================================================================
    # 1. 路径与常量
    # =========================================================================
    share_desc = get_package_share_directory("leonardo_description")
    share_bringup = get_package_share_directory("leonardo_bringup")

    robot_desc_file_path = os.path.join(share_desc, "urdf", "leonardo.urdf.xacro")
    rviz_config_file_path = os.path.join(share_bringup, "rviz", "leonardo_config.rviz")
    controllers_yaml_path = os.path.join(share_bringup, "config", "leonardo_controllers.yaml")

    ns = "leonardo"
    cm = f"/{ns}/controller_manager"
    robot_desc_cmd = ParameterValue(Command(["xacro ", robot_desc_file_path]), value_type=str)

    # =========================================================================
    # 2. robot模型发布与控制器管理器
    # =========================================================================

    # 2.1 Robot State Publisher：发布 URDF 与 TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[{"robot_description": robot_desc_cmd}],
        remappings=[("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")],
        output="screen",
    )

    # 2.2 Controller Manager：加载硬件接口与控制参数
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        parameters=[controllers_yaml_path, {"robot_description": robot_desc_cmd}],
        output="screen",
    )

    # =========================================================================
    # 3. 控制器生命周期：加载 → 激活
    # =========================================================================

    # 3.1 加载并配置全部控制器，保持 inactive（--stopped）
    load_all_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            "joint_state_broadcaster",
            "fd_left_ee_controller",
            "fd_right_ee_controller",
            "arm_mapping_controller",
            "aloe_controller",
            "--controller-manager",
            cm,
            "--inactive",
        ],
        output="screen",
    )

    # 3.2 激活全部控制器
    activate_all_controllers_cmd = ExecuteProcess(
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
            "arm_mapping_controller",
            "aloe_controller",
        ],
        output="screen",
    )

    # 3.3 触发：spawner 退出后执行激活
    on_controllers_loaded = RegisterEventHandler(
        OnProcessExit(
            target_action=load_all_controllers_spawner,
            on_exit=[activate_all_controllers_cmd],
        )
    )

    # =========================================================================
    # 4. 踏板生命周期节点：配置 → 激活 + 故障保护
    # =========================================================================

    # 4.1 生命周期节点定义
    foot_pedal_node = LifecycleNode(
        package="leonardo_human_interfaces",
        executable="foot_pedal_node",
        namespace=ns,
        name="foot_pedal_node",
        parameters=[controllers_yaml_path],
        output="screen",
    )

    # 4.2 配置请求：unconfigured → inactive
    start_pedal_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(foot_pedal_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # 4.3 激活请求：inactive → active
    on_pedal_configured = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=foot_pedal_node,
            goal_state="inactive",
            entities=[
                LogInfo(msg="[Pedal] Configured. Activating..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(foot_pedal_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # 4.4 故障保护：节点退出即终止整个进程组 [fail-fast]
    on_pedal_exit_fatal = RegisterEventHandler(
        OnProcessExit(
            target_action=foot_pedal_node,
            on_exit=[
                LogInfo(msg="[FATAL] pedal_teleop_node exited. Killing all..."),
                OpaqueFunction(function=kill_all_and_exit),
            ],
        )
    )

    # =========================================================================
    # 5. 可视化
    # =========================================================================

    # 5.1 RViz2：加载预设配置与命名空间重映射
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

    # =========================================================================
    # 6. 启动项组装（按运行顺序）
    # =========================================================================
    return LaunchDescription(
        [
            # 模型与控制器管理
            robot_state_publisher_node,
            controller_manager_node,
            # 控制器生命周期
            load_all_controllers_spawner,
            on_controllers_loaded,
            # 踏板生命周期
            foot_pedal_node,
            on_pedal_exit_fatal,  # 先挂故障保护
            start_pedal_configure,  # 再发起 configure
            on_pedal_configured,  # 最后挂 activate 链
            # 可视化
            rviz_node,
        ]
    )
