"""Validate the perception handoff before planning any grasp motion."""

from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from ur7e_motion import visual_target
from ur7e_motion.visual_target import validate_visual_pose


def _pose(x=0.16, y=0.47, z=0.33, stamp=10):
    message = PoseStamped()
    message.header.frame_id = 'base_link'
    message.header.stamp.sec = stamp
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.position.z = z
    message.pose.orientation.w = 1.0
    return message


def _validate(message, now_ns=10_100_000_000):
    return validate_visual_pose(
        message,
        now_ns=now_ns,
        max_age_seconds=0.5,
        table_position=[0.0, 0.48, 0.15],
        table_size=[0.70, 0.45, 0.30],
        object_size=[0.035, 0.035, 0.060],
    )


def test_fresh_visual_pose_on_table_is_accepted():
    target = _validate(_pose())
    assert target.position == (0.16, 0.47, 0.33)
    assert target.yaw == 0.0


@pytest.mark.parametrize(
    ('message', 'now_ns', 'reason'),
    [
        (_pose(stamp=9), 10_100_000_000, 'stale'),
        (_pose(x=0.60), 10_100_000_000, 'outside the table'),
        (_pose(z=0.41), 10_100_000_000, 'height'),
    ],
)
def test_invalid_visual_pose_is_rejected(message, now_ns, reason):
    with pytest.raises(ValueError, match=reason):
        _validate(message, now_ns)


def test_missing_detection_times_out_and_cleans_subscription(monkeypatch):
    clock = [0.0]
    calls = []
    monkeypatch.setattr(visual_target.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(visual_target.rclpy, 'ok', lambda: True)

    def spin_once(node, timeout_sec):
        assert timeout_sec >= 0.0
        clock[0] += max(timeout_sec, 0.001)

    monkeypatch.setattr(visual_target.rclpy, 'spin_once', spin_once)
    subscription = object()
    node = SimpleNamespace(
        create_subscription=lambda *args: subscription,
        destroy_subscription=lambda sub: calls.append(sub),
    )

    with pytest.raises(visual_target.VisualTargetUnavailableError,
                       match='no detection received'):
        visual_target.wait_for_visual_target(
            node, '/missing_pose', timeout_seconds=0.3, max_age_seconds=0.5,
            table_position=[0.0, 0.48, 0.15],
            table_size=[0.70, 0.45, 0.30],
            object_size=[0.035, 0.035, 0.060],
        )
    assert calls == [subscription]
