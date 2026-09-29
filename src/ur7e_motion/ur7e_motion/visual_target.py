"""从视觉节点接收新的方块位姿；不读取 Gazebo 模型真值。"""

from dataclasses import dataclass
import math
import time
from typing import Any

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.qos import qos_profile_sensor_data


class VisualTargetUnavailableError(RuntimeError):
    """未在限时内取得可用于抓取的新鲜、安全的视觉目标。"""


@dataclass(frozen=True)
class VisualTarget:
    """方块在 base_link 中的几何中心和绕竖直轴的角度。"""

    position: tuple[float, float, float]
    yaw: float


def validate_visual_pose(
    message: PoseStamped,
    *,
    now_ns: int,
    max_age_seconds: float,
    table_position: list[float],
    table_size: list[float],
    object_size: list[float],
) -> VisualTarget:
    """检查 TF、时效和桌面抓取范围，并解读视觉节点发布的姿态。"""
    if message.header.frame_id != 'base_link':
        raise ValueError('visual target must be in base_link')
    stamp_ns = message.header.stamp.sec * 1_000_000_000 + (
        message.header.stamp.nanosec
    )
    age_ns = now_ns - stamp_ns
    # Gazebo 的图像与 /clock 桥接可能相差一小段时间。
    if age_ns < -250_000_000 or age_ns > max_age_seconds * 1e9:
        raise ValueError('visual target is stale or has a future timestamp')

    point = message.pose.position
    position = (float(point.x), float(point.y), float(point.z))
    if not all(math.isfinite(value) for value in position):
        raise ValueError('visual target position must be finite')
    for axis in (0, 1):
        limit = (table_size[axis] - object_size[axis]) / 2
        if limit <= 0 or abs(position[axis] - table_position[axis]) > limit:
            raise ValueError('visual target is outside the table')
    expected_height = (
        table_position[2] + table_size[2] / 2 + object_size[2] / 2
    )
    if abs(position[2] - expected_height) > 0.035:
        raise ValueError('visual target height is inconsistent with the table')

    rotation = message.pose.orientation
    q = [float(rotation.x), float(rotation.y),
         float(rotation.z), float(rotation.w)]
    if not all(math.isfinite(value) for value in q):
        raise ValueError('visual target orientation must be finite')
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0:
        raise ValueError('visual target orientation must be nonzero')
    x, y, z, w = [value / norm for value in q]
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return VisualTarget(position, yaw)


def wait_for_visual_target(
    node: Any,
    topic: str,
    *,
    timeout_seconds: float,
    max_age_seconds: float,
    table_position: list[float],
    table_size: list[float],
    object_size: list[float],
) -> VisualTarget:
    """等待一个新鲜的视觉结果；失败时不返回预设目标坐标。"""
    if timeout_seconds <= 0 or max_age_seconds <= 0:
        raise ValueError('visual timeout and max age must be positive')
    latest: list[PoseStamped] = []
    subscription = node.create_subscription(
        PoseStamped, topic, latest.append, qos_profile_sensor_data
    )
    deadline = time.monotonic() + timeout_seconds
    last_error = 'no detection received'
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(node, timeout_sec=max(0.0, min(0.1, remaining)))
            while latest:
                message = latest.pop()
                latest.clear()
                try:
                    return validate_visual_pose(
                        message,
                        now_ns=node.get_clock().now().nanoseconds,
                        max_age_seconds=max_age_seconds,
                        table_position=table_position,
                        table_size=table_size,
                        object_size=object_size,
                    )
                except ValueError as exc:
                    last_error = str(exc)
    finally:
        node.destroy_subscription(subscription)
    raise VisualTargetUnavailableError(
        f'No usable visual target on {topic} within '
        f'{timeout_seconds:.1f} s: {last_error}'
    )
