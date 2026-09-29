"""在固定工作单元中执行带碰撞检测的抓取—搬运—放置演示。

此文件只负责任务编排：机械臂运动交给 UR7eMoveItClient，夹爪开合交给
GripperClient，桌面/方块等逻辑碰撞物体交给 SceneManager。
"""

from dataclasses import dataclass
import math
import os
import sys
import time
from typing import Any, Callable

import rclpy

from ur7e_motion import (
    GripperClient,
    GripperControlError,
    MotionExecutionError,
    MotionPlanningError,
    MoveItDependencyError,
    PlanningSceneError,
    RobotStateUnavailableError,
    SceneManager,
    UR7eMoveItClient,
)
from ur7e_motion.fixed_task import READY
from ur7e_motion.motion_command import ExitCode
from ur7e_motion.pick_place_demo import GRIPPER_TOUCH_LINKS, TCP_LINK
from ur7e_motion.validation import validate_numeric_vector
from ur7e_motion.visual_target import (
    VisualTarget,
    VisualTargetUnavailableError,
    wait_for_visual_target,
)


# PlanningScene 物体的稳定 ID：重复运行时同 ID 会覆盖旧物体，而不是不断累积。
TABLE_ID = 'workspace_table'
BACK_WALL_ID = 'workspace_back_wall'
STORAGE_BOX_ID = 'workspace_storage_box'
CENTER_DIVIDER_ID = 'workspace_center_divider'
TARGET_ID = 'workspace_target'

# 下列 RGBA 颜色只用于在 RViz 中快速区分工作单元物体。
TABLE_COLOR = (0.45, 0.25, 0.10, 1.0)
BACK_WALL_COLOR = (0.60, 0.66, 0.75, 1.0)
STORAGE_BOX_COLOR = (0.10, 0.38, 0.82, 1.0)
CENTER_DIVIDER_COLOR = (0.95, 0.48, 0.08, 1.0)
TARGET_COLOR = (0.88, 0.08, 0.08, 1.0)

# 保持客户端到进程退出：规避当前 MoveItPy 版本在 Python 析构阶段的已知退出问题。
_MOVEIT_CLIENT: UR7eMoveItClient | None = None
_GRIPPER_CLIENT: GripperClient | None = None


@dataclass(frozen=True)
class WorkspacePickPlaceConfig:
    """已经校验过的工作单元几何和任务参数。

    所有位置/尺寸都是 base_link 下的米单位三维向量；夹爪位置是主动 knuckle 的弧度。
    """

    table_size: list[float]
    table_position: list[float]
    back_wall_size: list[float]
    back_wall_position: list[float]
    storage_box_size: list[float]
    storage_box_position: list[float]
    center_divider_size: list[float]
    center_divider_position: list[float]
    target_size: list[float]
    target_position: list[float]
    place_position: list[float]
    grasp_tcp_offset: list[float]
    lift_translation: list[float]
    grasp_position: float
    gripper_duration: float
    scene_wait_seconds: float
    center_divider_padding: float = 0.0
    static_scene_before_ready: bool = False
    use_vision_target: bool = False
    vision_target_topic: str = '/color_cube_detector/detections/red/pose'
    vision_timeout_seconds: float = 10.0
    vision_max_age_seconds: float = 0.5


def execute_workspace_pick_place(
    arm: Any,
    gripper: Any,
    scene: Any,
    config: WorkspacePickPlaceConfig,
    *,
    sleep_fn: Callable[[float], None] = time.sleep,
    target_provider: Callable[[], VisualTarget] | None = None,
) -> None:
    """执行一次完整抓取流程，成功后保留场景供 RViz 观察。

    动作顺序：READY → 张开 → 建场景 → 预抓取 → 抓取 → 闭合并附着 →
    抬升/搬运/下放 → 张开并解除附着 → 退出 → READY。
    """
    # 这三个标志让 except 中的清理遵循实际进度，避免失败后误操作未创建的资源。
    attached = False
    touch_allowed = False
    target_added = False
    if config.use_vision_target and target_provider is None:
        raise VisualTargetUnavailableError('Visual target provider is missing')

    try:
        # Gazebo 中障碍物在物理世界里启动时就存在。重复运行时若机器人不在 READY，
        # 应先把它们加入 MoveIt，再规划回 READY，才能避免“回家途中穿过隔断”。
        if config.static_scene_before_ready:
            _add_static_workcell(scene, config)
        arm.move_to_joint(READY)
        gripper.open(config.gripper_duration)
        # READY 被设计为夹爪严格向下。读取其实际四元数，并在全任务中保持该方向。
        ready_pose = arm.get_current_pose().pose
        orientation = [
            ready_pose.orientation.x,
            ready_pose.orientation.y,
            ready_pose.orientation.z,
            ready_pose.orientation.w,
        ]

        if not config.static_scene_before_ready:
            _add_static_workcell(scene, config)
        # 视觉模式必须先得到新鲜且通过范围校验的方块中心；失败则停止任务。
        target = (
            target_provider() if target_provider is not None
            else VisualTarget(tuple(config.target_position), 0.0)
        )
        target_position = list(target.position)
        if target_provider is not None:
            orientation = _rotate_downward_orientation(orientation, target.yaw)
        # 目标先作为世界碰撞物体存在：此时 MoveIt 会把它当作不可穿透的障碍物。
        target_box_options = {'color': TARGET_COLOR}
        if target_provider is not None:
            target_box_options['orientation'] = (
                0.0, 0.0, math.sin(target.yaw / 2), math.cos(target.yaw / 2)
            )
        scene.add_box(
            TARGET_ID,
            config.target_size,
            target_position,
            **target_box_options,
        )
        target_added = True
        _sleep_if_positive(config.scene_wait_seconds, sleep_fn)

        # 接近/闭合时，方块必然触碰夹爪。只对夹爪链路暂时放宽 ACM，机械臂其他部位
        # 依旧不能碰方块、桌面或隔断。
        scene.set_object_touch_allowed(
            TARGET_ID, GRIPPER_TOUCH_LINKS, True
        )
        touch_allowed = True

        # 从物体中心和配置偏移推导四个 TCP 点；lift_translation 同时定义预抓取高度、
        # 抬升高度和放置后的退出高度。
        source_grasp_tcp = _add_vectors(
            target_position, config.grasp_tcp_offset
        )
        source_pregrasp_tcp = _add_vectors(
            source_grasp_tcp, config.lift_translation
        )
        place_grasp_tcp = _add_vectors(
            config.place_position, config.grasp_tcp_offset
        )
        place_pregrasp_tcp = _add_vectors(
            place_grasp_tcp, config.lift_translation
        )

        # MoveIt 使用 OMPL 在这些离散目标之间规划；它们不是强制笛卡尔直线段。
        arm.move_to_pose(source_pregrasp_tcp, orientation)
        arm.move_to_pose(source_grasp_tcp, orientation)
        gripper.move_to(config.grasp_position, config.gripper_duration)
        # 逻辑附着后，MoveIt 会随夹爪一起移动方块碰撞体并将其纳入搬运期碰撞检查。
        # Gazebo 中的实体方块仍由真实接触与摩擦驱动，二者不是同一份物体状态。
        scene.attach_object(
            TARGET_ID,
            link_name=TCP_LINK,
            touch_links=GRIPPER_TOUCH_LINKS,
        )
        attached = True

        arm.move_to_pose(source_pregrasp_tcp, orientation)
        arm.move_to_pose(place_pregrasp_tcp, orientation)
        arm.move_to_pose(place_grasp_tcp, orientation)
        # 松开后先解除逻辑附着，将方块作为放置位置的新世界碰撞物体保留下来。
        gripper.open(config.gripper_duration)
        scene.detach_object(TARGET_ID)
        attached = False
        arm.move_to_pose(place_pregrasp_tcp, orientation)
        scene.set_object_touch_allowed(
            TARGET_ID, GRIPPER_TOUCH_LINKS, False
        )
        touch_allowed = False
        arm.move_to_joint(READY)
    except Exception:
        # 任意一步失败后不再尝试后续运动；只进行尽力而为的反向清理。
        # 静态桌面/墙/隔断故意保留，便于在 RViz 中诊断失败原因。
        if attached:
            _ignore_cleanup_error(scene.detach_object, TARGET_ID)
        if touch_allowed:
            _ignore_cleanup_error(
                scene.set_object_touch_allowed,
                TARGET_ID,
                GRIPPER_TOUCH_LINKS,
                False,
            )
        if target_added:
            _ignore_cleanup_error(scene.remove_object, TARGET_ID)
        raise


def _add_static_workcell(scene: Any, config: WorkspacePickPlaceConfig) -> None:
    """向 MoveIt 添加或覆盖桌面、后墙、储物箱和中间隔断。"""
    # 仅扩大 MoveIt 中的隔断碰撞盒；Gazebo 物理障碍物尺寸保持原样。
    # padding 是每个面的余量，因此总长/宽/高各增加 2 * padding。
    divider_planning_size = [
        dimension + 2.0 * config.center_divider_padding
        for dimension in config.center_divider_size
    ]
    for object_id, size, position, color in (
        (TABLE_ID, config.table_size, config.table_position, TABLE_COLOR),
        (
            BACK_WALL_ID,
            config.back_wall_size,
            config.back_wall_position,
            BACK_WALL_COLOR,
        ),
        (
            STORAGE_BOX_ID,
            config.storage_box_size,
            config.storage_box_position,
            STORAGE_BOX_COLOR,
        ),
        (
            CENTER_DIVIDER_ID,
            divider_planning_size,
            config.center_divider_position,
            CENTER_DIVIDER_COLOR,
        ),
    ):
        scene.add_box(object_id, size, position, color=color)


def _rotate_downward_orientation(
    ready: list[float], yaw: float
) -> list[float]:
    """保持 READY 的向下姿态，同时对齐方块在桌面上的水平朝向。"""
    x, y, z, w = ready
    cosine, sine = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        cosine * x - sine * y,
        sine * x + cosine * y,
        cosine * z + sine * w,
        cosine * w - sine * z,
    ]


def _add_vectors(first: list[float], second: list[float]) -> list[float]:
    """相加两个三维坐标向量，并压制二进制浮点显示噪声。"""
    return [round(first[index] + second[index], 10) for index in range(3)]


def _ignore_cleanup_error(function: Callable[..., Any], *args: Any) -> None:
    """清理阶段忽略异常，保留最初导致任务中止的主异常。"""
    try:
        function(*args)
    except Exception:
        pass


def _sleep_if_positive(
    duration: float, sleep_fn: Callable[[float], None]
) -> None:
    if duration > 0.0:
        sleep_fn(duration)


def _read_config(node: Any) -> WorkspacePickPlaceConfig:
    """声明 ROS 参数、读取 YAML 覆盖值并做范围/类型校验。"""
    # 默认值对应 RViz/Mock Hardware 逻辑演示；Gazebo launch 会用自己的 YAML 覆盖
    # 方块高度、抓取角度和起点容差等物理仿真专用参数。
    defaults = {
        'table_size': [0.70, 0.45, 0.30],
        'table_position': [0.0, 0.48, 0.15],
        'back_wall_size': [0.70, 0.05, 0.55],
        'back_wall_position': [0.0, 0.75, 0.575],
        'storage_box_size': [0.14, 0.14, 0.20],
        'storage_box_position': [-0.26, 0.62, 0.40],
        'center_divider_size': [0.06, 0.18, 0.24],
        'center_divider_position': [0.02, 0.47, 0.42],
        'center_divider_padding': 0.0,
        'target_size': [0.035, 0.035, 0.060],
        'target_position': [0.16, 0.47, 0.34],
        'place_position': [0.16, 0.32, 0.34],
        'grasp_tcp_offset': [0.0, 0.0, -0.03],
        'lift_translation': [0.0, 0.0, 0.10],
        'grasp_position': 0.50,
        'gripper_duration': 1.0,
        'scene_wait_seconds': 0.5,
        'static_scene_before_ready': False,
        'use_vision_target': False,
        'vision_target_topic': '/color_cube_detector/detections/red/pose',
        'vision_timeout_seconds': 10.0,
        'vision_max_age_seconds': 0.5,
    }
    for name, default in defaults.items():
        node.declare_parameter(name, default)

    # 所有尺寸和位置都必须正好含有 x、y、z 三项。
    vector_names = (
        'table_size',
        'table_position',
        'back_wall_size',
        'back_wall_position',
        'storage_box_size',
        'storage_box_position',
        'center_divider_size',
        'center_divider_position',
        'target_size',
        'target_position',
        'place_position',
        'grasp_tcp_offset',
        'lift_translation',
    )
    values = {name: _read_vector(node, name) for name in vector_names}
    for name in (
        'table_size',
        'back_wall_size',
        'storage_box_size',
        'center_divider_size',
        'target_size',
    ):
        if any(value <= 0.0 for value in values[name]):
            raise ValueError(f'{name} values must all be greater than zero')

    grasp_position = _read_finite_number(node, 'grasp_position')
    if not 0.0 <= grasp_position <= 0.8:
        raise ValueError('grasp_position must be between 0.0 and 0.8 rad')

    gripper_duration = _read_finite_number(node, 'gripper_duration')
    if gripper_duration <= 0.0:
        raise ValueError('gripper_duration must be greater than zero')

    scene_wait_seconds = _read_finite_number(node, 'scene_wait_seconds')
    if scene_wait_seconds < 0.0:
        raise ValueError('scene_wait_seconds must not be negative')

    center_divider_padding = _read_finite_number(
        node, 'center_divider_padding'
    )
    if center_divider_padding < 0.0:
        raise ValueError('center_divider_padding must not be negative')

    static_scene_before_ready = node.get_parameter(
        'static_scene_before_ready'
    ).value
    if not isinstance(static_scene_before_ready, bool):
        raise ValueError('static_scene_before_ready must be a bool')

    use_vision_target = node.get_parameter('use_vision_target').value
    if not isinstance(use_vision_target, bool):
        raise ValueError('use_vision_target must be a bool')
    vision_target_topic = node.get_parameter('vision_target_topic').value
    if (
        not isinstance(vision_target_topic, str)
        or not vision_target_topic.startswith('/')
    ):
        raise ValueError('vision_target_topic must be an absolute topic name')
    vision_timeout_seconds = _read_finite_number(node, 'vision_timeout_seconds')
    vision_max_age_seconds = _read_finite_number(node, 'vision_max_age_seconds')
    if vision_timeout_seconds <= 0 or vision_max_age_seconds <= 0:
        raise ValueError('vision timeout and max age must be positive')

    lift_translation = values['lift_translation']
    if lift_translation[2] <= 0.0:
        raise ValueError('lift_translation must have a positive Z component')

    return WorkspacePickPlaceConfig(
        table_size=values['table_size'],
        table_position=values['table_position'],
        back_wall_size=values['back_wall_size'],
        back_wall_position=values['back_wall_position'],
        storage_box_size=values['storage_box_size'],
        storage_box_position=values['storage_box_position'],
        center_divider_size=values['center_divider_size'],
        center_divider_position=values['center_divider_position'],
        target_size=values['target_size'],
        target_position=values['target_position'],
        place_position=values['place_position'],
        grasp_tcp_offset=values['grasp_tcp_offset'],
        lift_translation=lift_translation,
        grasp_position=grasp_position,
        gripper_duration=gripper_duration,
        scene_wait_seconds=scene_wait_seconds,
        center_divider_padding=center_divider_padding,
        static_scene_before_ready=static_scene_before_ready,
        use_vision_target=use_vision_target,
        vision_target_topic=vision_target_topic,
        vision_timeout_seconds=vision_timeout_seconds,
        vision_max_age_seconds=vision_max_age_seconds,
    )


def _read_vector(node: Any, name: str) -> list[float]:
    return validate_numeric_vector(name, node.get_parameter(name).value, 3)


def _read_finite_number(node: Any, name: str) -> float:
    value = node.get_parameter(name).value
    if isinstance(value, bool):
        raise ValueError(f'{name} must be a finite number')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite number') from exc
    if not math.isfinite(number):
        raise ValueError(f'{name} must be a finite number')
    return number


def _run(node: Any, config: WorkspacePickPlaceConfig) -> ExitCode:
    """构建三个核心客户端，运行任务，并转换为可供 launch 识别的退出码。"""
    global _GRIPPER_CLIENT, _MOVEIT_CLIENT

    try:
        _MOVEIT_CLIENT = UR7eMoveItClient(node)
        _GRIPPER_CLIENT = GripperClient(node)
        # 三个对象分别封装规划执行、夹爪 Action 和 PlanningScene 操作。
        scene = SceneManager(node, _MOVEIT_CLIENT.planning_scene_monitor)
        node.get_logger().info(
            'Starting workspace pick-place demo: READY -> avoid divider '
            '-> PICK -> PLACE -> READY'
        )
        target_provider = None
        if config.use_vision_target:
            def get_visual_target():
                return wait_for_visual_target(
                    node,
                    config.vision_target_topic,
                    timeout_seconds=config.vision_timeout_seconds,
                    max_age_seconds=config.vision_max_age_seconds,
                    table_position=config.table_position,
                    table_size=config.table_size,
                    object_size=config.target_size,
                )
            target_provider = get_visual_target
        execute_workspace_pick_place(
            _MOVEIT_CLIENT, _GRIPPER_CLIENT, scene, config,
            target_provider=target_provider,
        )
        node.get_logger().info(
            'Workspace pick-place demo completed; scene remains in RViz'
        )
        return ExitCode.SUCCESS
    except MoveItDependencyError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.DEPENDENCY_ERROR
    except RobotStateUnavailableError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.ROBOT_STATE_UNAVAILABLE
    except MotionPlanningError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.PLANNING_FAILED
    except MotionExecutionError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.EXECUTION_FAILED
    except PlanningSceneError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.SCENE_FAILED
    except GripperControlError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.GRIPPER_FAILED
    except VisualTargetUnavailableError as exc:
        node.get_logger().error(f'Workspace demo stopped: {exc}')
        return ExitCode.VISION_FAILED
    except Exception as exc:
        node.get_logger().error(
            f'Unexpected workspace demo error: {type(exc).__name__}: {exc}'
        )
        return ExitCode.INTERNAL_ERROR


def main(args: list[str] | None = None) -> int:
    """ROS 控制台入口：读取参数、执行一次任务、返回明确退出码。"""
    rclpy.init(args=args)
    node = rclpy.create_node('ur7e_workspace_pick_place_parameters')
    exit_code = ExitCode.INTERNAL_ERROR
    try:
        try:
            config = _read_config(node)
        except (TypeError, ValueError) as exc:
            node.get_logger().error(f'Invalid workspace demo parameter: {exc}')
            exit_code = ExitCode.INVALID_PARAMETERS
        else:
            exit_code = _run(node, config)
    finally:
        if _GRIPPER_CLIENT is not None:
            _GRIPPER_CLIENT.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if _MOVEIT_CLIENT is not None:
        # MoveItPy 2.12.4 在解释器正常析构时可能崩溃。完成节点/ROS 清理后让 OS 回收
        # MoveItPy，同时仍把正确的成功或失败退出码交给 ros2 launch。
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(int(exit_code))
    return int(exit_code)


if __name__ == '__main__':
    raise SystemExit(main())
