"""执行一条 UR7e 单次运动命令后退出。

这是最小的命令行入口：通过 ROS 参数选择“到关节目标”或“从当前 TCP 相对平移”。
更复杂的固定任务和抓取任务会复用同一个 UR7eMoveItClient。
"""

from enum import IntEnum
import os
import sys
from typing import Any

import rclpy

from ur7e_motion.moveit_client import (
    MotionExecutionError,
    MotionPlanningError,
    MoveItDependencyError,
    RobotStateUnavailableError,
    UR7eMoveItClient,
)
from ur7e_motion.validation import validate_mode, validate_numeric_vector


# 默认 READY：夹爪严格向下，适合作为工作单元任务的安全起始/结束姿态。
DEFAULT_JOINT_POSITIONS = [
    1.54,
    -1.62,
    1.4,
    -1.350796327,
    -1.570796326589793,
    -0.030796327,
]
DEFAULT_TRANSLATION = [0.0, 0.0, 0.05]

# MoveIt 2.12.4 在 Python 解释器析构 MoveItPy 时可能崩溃。保持引用到进程结束，
# 再由 os._exit() 让操作系统回收它，避免“任务实际成功但退出阶段报错”。
_MOVEIT_CLIENT: UR7eMoveItClient | None = None


class ExitCode(IntEnum):
    """供 ros2 launch、脚本和测试判断失败类型的进程退出码。"""

    SUCCESS = 0
    INVALID_PARAMETERS = 2
    DEPENDENCY_ERROR = 3
    ROBOT_STATE_UNAVAILABLE = 4
    PLANNING_FAILED = 5
    EXECUTION_FAILED = 6
    SCENE_FAILED = 7
    GRIPPER_FAILED = 8
    INTERNAL_ERROR = 10


def _declare_parameters(node: Any) -> tuple[str, list[float], list[float]]:
    """声明三个外部 ROS 参数，并验证数量、类型和模式。"""
    node.declare_parameter("mode", "joint")
    node.declare_parameter("joint_positions", DEFAULT_JOINT_POSITIONS)
    node.declare_parameter("translation", DEFAULT_TRANSLATION)

    mode = validate_mode(node.get_parameter("mode").value)
    joint_positions = validate_numeric_vector(
        "joint_positions", node.get_parameter("joint_positions").value, 6
    )
    translation = validate_numeric_vector(
        "translation", node.get_parameter("translation").value, 3
    )
    return mode, joint_positions, translation


def _run_motion(
    node: Any,
    mode: str,
    joint_positions: list[float],
    translation: list[float],
) -> ExitCode:
    """按 mode 调用对应高层接口，并把异常转换成稳定退出码。"""
    global _MOVEIT_CLIENT

    try:
        _MOVEIT_CLIENT = UR7eMoveItClient(node)
        # validate_mode() 已确保 mode 只能是 joint 或 pose。
        if mode == "joint":
            _MOVEIT_CLIENT.move_to_joint(joint_positions)
        else:
            _MOVEIT_CLIENT.move_by_translation(translation)
        return ExitCode.SUCCESS
    except MoveItDependencyError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.DEPENDENCY_ERROR
    except RobotStateUnavailableError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.ROBOT_STATE_UNAVAILABLE
    except MotionPlanningError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.PLANNING_FAILED
    except MotionExecutionError as exc:
        node.get_logger().error(str(exc))
        return ExitCode.EXECUTION_FAILED
    # MoveIt Python 绑定偶尔会透出无法精确分类的 C++ 异常，也要避免节点无提示崩溃。
    except Exception as exc:
        node.get_logger().error(
            f"Unexpected motion error: {type(exc).__name__}: {exc}"
        )
        return ExitCode.INTERNAL_ERROR


def main(args: list[str] | None = None) -> int:
    """ROS 控制台入口：读取参数，执行一条动作，然后退出。"""
    rclpy.init(args=args)
    node = rclpy.create_node("ur7e_motion_parameters")
    exit_code = ExitCode.INTERNAL_ERROR
    try:
        try:
            mode, joint_positions, translation = _declare_parameters(node)
        except (TypeError, ValueError) as exc:
            node.get_logger().error(f"Invalid motion parameters: {exc}")
            exit_code = ExitCode.INVALID_PARAMETERS
        else:
            exit_code = _run_motion(node, mode, joint_positions, translation)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if _MOVEIT_CLIENT is not None:
        # Avoid the known MoveItPy 2.12.4 destructor crash while preserving an
        # accurate process exit code for launch and automation.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(int(exit_code))
    return int(exit_code)


if __name__ == "__main__":
    raise SystemExit(main())
