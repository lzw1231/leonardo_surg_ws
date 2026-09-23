"""Launch 文件通用工具函数。"""

from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def spawner(controller, param_file=None, ns="leonardo", cm_name=None):
    """生成 controller_manager spawner 节点。

    controller:  控制器名，如 "fd_left_ee_controller"
    param_file:  控制器参数 YAML 路径，None 表示不传（如 joint_state_broadcaster）
    ns:          命名空间，默认 "leonardo"
    cm_name:     controller_manager 完全限定名，默认 /<ns>/controller_manager
    """
    if cm_name is None:
        cm_name = f"/{ns}/controller_manager"

    args = [controller, "--controller-manager", cm_name]
    if param_file is not None:
        args += ["--controller-ros-args", f"--params-file {param_file}"]

    return Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=args,
    )


def run_after_all(dependencies, action):
    """等 dependencies 里所有节点都退出后，再启动 action。

    dependencies: Node 列表（通常是 spawner）
    action:       要启动的 Node

    用法:
        run_after_all([spawner_a, spawner_b], spawner_c)
        # 等 a 和 b 都完成，才启动 c
    """
    state = {"done": [False] * len(dependencies), "started": False}

    def _check(_context):
        if all(state["done"]) and not state["started"]:
            state["started"] = True
            return [action]
        return []

    def _make_callback(idx):
        def _cb(_context):
            state["done"][idx] = True
            return [OpaqueFunction(function=_check)]

        return _cb

    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=dep,
                on_exit=[OpaqueFunction(function=_make_callback(i))],
            )
        )
        for i, dep in enumerate(dependencies)
    ]
