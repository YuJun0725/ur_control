#!/usr/bin/env python3
"""Exit successfully once all Gazebo controllers are active."""

import sys
import time

from controller_manager_msgs.srv import ListControllers
import rclpy


REQUIRED_CONTROLLERS = {
    'joint_state_broadcaster',
    'scaled_joint_trajectory_controller',
    'gripper_controller',
}


def main(args=None) -> int:
    """Wait on controller_manager using wall time so paused simulation works."""
    rclpy.init(args=args)
    node = rclpy.create_node('ur7e_gazebo_controller_ready')
    node.declare_parameter('timeout_seconds', 90.0)
    timeout = float(node.get_parameter('timeout_seconds').value)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    deadline = time.monotonic() + timeout
    next_log_time = 0.0

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if not future.done() or future.result() is None:
                continue
            active = {
                controller.name
                for controller in future.result().controller
                if controller.state == 'active'
            }
            missing = REQUIRED_CONTROLLERS - active
            if not missing:
                node.get_logger().info('All Gazebo controllers are active')
                return 0
            if time.monotonic() >= next_log_time:
                node.get_logger().info(
                    'Waiting for active controllers: '
                    + ', '.join(sorted(missing))
                )
                next_log_time = time.monotonic() + 2.0
            time.sleep(0.25)

        node.get_logger().error(
            f'Controllers did not become active within {timeout:.1f} seconds'
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
