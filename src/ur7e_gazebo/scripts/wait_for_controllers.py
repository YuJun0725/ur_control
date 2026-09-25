#!/usr/bin/env python3
"""等待 Gazebo 所需控制器全部激活；满足条件时以退出码 0 结束。"""

import sys
import time

from controller_manager_msgs.srv import ListControllers
import rclpy


# 一键启动只有在这三个控制器都可用时才允许启动 MoveIt 和抓取任务。
REQUIRED_CONTROLLERS = {
    'joint_state_broadcaster',
    'scaled_joint_trajectory_controller',
    'gripper_controller',
}


def main(args=None) -> int:
    """用墙钟时间轮询 controller_manager，故 Gazebo 暂停时仍能正确超时。"""
    rclpy.init(args=args)
    node = rclpy.create_node('ur7e_gazebo_controller_ready')
    node.declare_parameter('timeout_seconds', 90.0)
    timeout = float(node.get_parameter('timeout_seconds').value)
    # controller_manager 提供该服务，用于查询已加载控制器及其 inactive/active 状态。
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    deadline = time.monotonic() + timeout
    next_log_time = 0.0

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            # 服务调用本身是异步的，spin_until_future_complete 让本小节点等待响应。
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if not future.done() or future.result() is None:
                continue
            # 只接受真正 active 的控制器：已加载但 inactive 的控制器不能接收轨迹。
            active = {
                controller.name
                for controller in future.result().controller
                if controller.state == 'active'
            }
            missing = REQUIRED_CONTROLLERS - active
            if not missing:
                node.get_logger().info('All Gazebo controllers are active')
                return 0
            # 每两秒报告一次缺失项，避免每轮轮询都刷屏。
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
        # 无论成功或超时都释放节点；返回值由外层 launch 的 OnProcessExit 判断。
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
