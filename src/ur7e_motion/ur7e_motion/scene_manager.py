"""MoveIt PlanningScene 中碰撞物体的增删、附加和接触许可管理。

这里管理的是 MoveIt 的“逻辑规划场景”，用于碰撞检查和 RViz 显示；它不会直接创建或
移动 Gazebo 物理物体。Gazebo 抓取任务会在两个世界中分别维护对应物体。
"""

from collections.abc import Sequence
import time
from typing import Any

from ur7e_motion.validation import normalize_quaternion, validate_numeric_vector


# PlanningScene 的标准增量更新话题；move_group、MoveItPy 和 RViz 都会接收它。
PLANNING_SCENE_TOPIC = "/planning_scene"


class PlanningSceneError(RuntimeError):
    """Raised when a collision object cannot be applied to the scene."""


class SceneManager:
    """管理 MoveIt 规划所需的世界碰撞物体和附着物体。"""

    def __init__(
        self,
        node: Any,
        planning_scene_monitor: Any,
        *,
        planning_scene_topic: str = PLANNING_SCENE_TOPIC,
        scene_update_wait_seconds: float = 0.5,
    ) -> None:
        """创建场景管理器，并建立向 /planning_scene 发布增量的发布者。"""
        from moveit_msgs.msg import PlanningScene
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self._logger = node.get_logger()
        self._monitor = planning_scene_monitor
        # 两个集合只记录“由本管理器操作过”的物体，供任务失败时安全清理使用。
        self._known_object_ids: set[str] = set()       # 世界中的物体
        self._attached_object_ids: set[str] = set()    # 已附着在机器人上的物体
        self._scene_update_wait_seconds = float(scene_update_wait_seconds)
        if self._scene_update_wait_seconds < 0.0:
            raise ValueError("scene_update_wait_seconds must not be negative")
        # TRANSIENT_LOCAL 让晚启动的 RViz 也能获得最近一次完整的场景更新。
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = node.create_publisher(
            PlanningScene, planning_scene_topic, qos
        )

    def add_box(
        self,
        object_id: str,
        size: Sequence[float],
        position: Sequence[float],
        *,
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
        frame_id: str = "base_link",
        color: Sequence[float] | None = None,
    ) -> None:
        """添加或覆盖一个盒状碰撞物体。

        ``color`` 是可选 RGBA ``[r, g, b, a]``，范围均为 ``[0.0, 1.0]``。
        它只改变 RViz 颜色，不会改变碰撞检测或规划结果。
        """
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject, ObjectColor
        from shape_msgs.msg import SolidPrimitive

        clean_id = self._validate_id(object_id)
        dimensions = validate_numeric_vector("size", size, 3)
        if any(value <= 0.0 for value in dimensions):
            raise ValueError("size values must all be greater than zero")
        coordinates = validate_numeric_vector("position", position, 3)
        quaternion = normalize_quaternion(orientation)
        if not frame_id.strip():
            raise ValueError("frame_id must not be empty")
        object_color = None
        if color is not None:
            rgba = validate_numeric_vector("color", color, 4)
            if any(value < 0.0 or value > 1.0 for value in rgba):
                raise ValueError("color values must be between 0.0 and 1.0")
            object_color = ObjectColor()
            object_color.id = clean_id
            (
                object_color.color.r,
                object_color.color.g,
                object_color.color.b,
                object_color.color.a,
            ) = rgba

        # MoveIt 用 SolidPrimitive + Pose 描述简单几何体；本项目的桌面、墙、隔断
        # 和方块都可以用 BOX 表达，无需单独的碰撞网格文件。
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = coordinates
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = quaternion

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id.strip()
        collision_object.id = clean_id
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        # _apply() 同时更新本地 PlanningSceneMonitor 和 ROS 话题，确保当前任务的
        # MoveItPy 与其他节点/RViz 都能尽快看到同一份场景。
        self._apply(collision_object, object_color=object_color)
        self._known_object_ids.add(clean_id)
        self._logger.info(
            f"Added collision box '{clean_id}': size={dimensions}, "
            f"position={coordinates}, frame={frame_id.strip()}"
        )

    def remove_object(self, object_id: str) -> None:
        """从场景移除一个世界物体；若它已附着，会先解除附着。"""
        from moveit_msgs.msg import CollisionObject

        clean_id = self._validate_id(object_id)
        if clean_id in self._attached_object_ids:
            self.detach_object(clean_id)

        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = clean_id
        collision_object.operation = CollisionObject.REMOVE
        self._apply(collision_object)
        self._known_object_ids.discard(clean_id)
        self._logger.info(f"Removed collision object '{clean_id}'")

    def attach_object(
        self,
        object_id: str,
        *,
        link_name: str = "robotiq_tcp",
        touch_links: Sequence[str],
    ) -> None:
        """将已有世界物体逻辑附着到机器人 link。

        这会使 MoveIt 在后续搬运规划中把该物体视为机器人一部分；不会把 Gazebo 物体
        固定到夹爪，Gazebo 中的移动仍必须依赖真实接触和摩擦。
        """
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        clean_id = self._validate_id(object_id)
        clean_link = self._validate_link_name("link_name", link_name)
        clean_touch_links = self._validate_link_names(touch_links)

        attached_object = AttachedCollisionObject()
        attached_object.link_name = clean_link
        attached_object.object.id = clean_id
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = clean_touch_links

        # 附着前先确认方块确实存在于世界场景，防止拼错 ID 时产生不完整场景。
        with self._monitor.read_write() as scene:
            message = scene.planning_scene_message
            world_ids = {
                item.id for item in message.world.collision_objects
            }
            if clean_id not in world_ids:
                raise PlanningSceneError(
                    f"Cannot attach unknown world object '{clean_id}'"
                )
        # 先发布消息让其他 MoveIt/RViz 实例同步；若本地监视器未在等待时间内收到回环
        # 更新，则直接调用底层 API 应用，以保证同一任务的下一步能够继续规划。
        self._publish_attached(attached_object)
        if not self._wait_for_attachment_state(clean_id, attached=True):
            with self._monitor.read_write() as scene:
                applied = scene.process_attached_collision_object(
                    attached_object
                )
                scene.current_state.update()
            if applied is False:
                raise PlanningSceneError(
                    f"MoveIt rejected attachment of object '{clean_id}'"
                )

        self._known_object_ids.discard(clean_id)
        self._attached_object_ids.add(clean_id)
        self._logger.info(
            f"Attached collision object '{clean_id}' to '{clean_link}'"
        )

    def detach_object(self, object_id: str) -> None:
        """解除逻辑附着，并把物体保留在其当前世界位姿。"""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        clean_id = self._validate_id(object_id)
        attached_object = AttachedCollisionObject()
        attached_object.object.id = clean_id
        attached_object.object.operation = CollisionObject.REMOVE

        # 从当前附着列表读出原 link_name；移除操作需要使用同一个附着关系。
        with self._monitor.read_write() as scene:
            message = scene.planning_scene_message
            matching = [
                item
                for item in message.robot_state.attached_collision_objects
                if item.object.id == clean_id
            ]
            if not matching:
                raise PlanningSceneError(
                    f"Cannot detach object '{clean_id}': it is not attached"
                )
            attached_object.link_name = matching[0].link_name
        self._publish_attached(attached_object)
        if not self._wait_for_attachment_state(clean_id, attached=False):
            with self._monitor.read_write() as scene:
                applied = scene.process_attached_collision_object(
                    attached_object
                )
                scene.current_state.update()
            if applied is False:
                raise PlanningSceneError(
                    f"MoveIt rejected detachment of object '{clean_id}'"
                )

        self._attached_object_ids.discard(clean_id)
        self._known_object_ids.add(clean_id)
        self._logger.info(
            f"Detached collision object '{clean_id}' into the world"
        )

    def set_object_touch_allowed(
        self,
        object_id: str,
        link_names: Sequence[str],
        allowed: bool,
    ) -> None:
        """允许或撤销指定物体与指定 link 之间的碰撞。

        抓取时仅允许方块碰触 8 个夹爪 link，避免 MoveIt 因“夹住方块”而判定失败；
        机械臂其他 link 与桌面等物体仍保留正常碰撞检查。
        """
        from moveit_msgs.msg import PlanningScene

        clean_id = self._validate_id(object_id)
        clean_links = self._validate_link_names(link_names)
        if not isinstance(allowed, bool):
            raise ValueError("allowed must be a bool")

        # Allowed Collision Matrix（ACM）是 MoveIt 的“这两个实体可以接触吗”表。
        with self._monitor.read_write() as scene:
            matrix = scene.allowed_collision_matrix
            if allowed:
                for link_name in clean_links:
                    matrix.set_entry(clean_id, link_name, True)
            else:
                for link_name in clean_links:
                    matrix.remove_entry(clean_id, link_name)
            scene_message = scene.planning_scene_message
            matrix_message = scene_message.allowed_collision_matrix

        scene_update = PlanningScene()
        scene_update.is_diff = True
        scene_update.allowed_collision_matrix = matrix_message
        self._publisher.publish(scene_update)
        action = "Allowed" if allowed else "Cleared"
        self._logger.info(
            f"{action} touch collisions for object '{clean_id}'"
        )

    def clear(self) -> None:
        """清理由本管理器创建或附着的所有物体，主要用于异常恢复。"""
        for object_id in tuple(self._attached_object_ids):
            self.detach_object(object_id)
        for object_id in tuple(self._known_object_ids):
            self.remove_object(object_id)

    @staticmethod
    def _validate_id(object_id: str) -> str:
        if not isinstance(object_id, str) or not object_id.strip():
            raise ValueError("object_id must not be empty")
        return object_id.strip()

    @staticmethod
    def _validate_link_name(name: str, value: Any) -> str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must not be empty")
        return value.strip()

    @classmethod
    def _validate_link_names(cls, link_names: Sequence[str]) -> list[str]:
        if isinstance(link_names, (str, bytes)):
            raise ValueError("link_names must be a sequence of link names")
        clean_names = [
            cls._validate_link_name("link name", name) for name in link_names
        ]
        if not clean_names:
            raise ValueError("link_names must not be empty")
        return clean_names

    def _apply(
        self, collision_object: Any, *, object_color: Any = None
    ) -> None:
        from moveit_msgs.msg import CollisionObject
        from moveit_msgs.msg import PlanningScene

        # 先立即更新当前进程的场景；否则接下来的同进程规划可能早于 ROS 话题回环。
        with self._monitor.read_write() as scene:
            known_ids = {
                item.id
                for item in scene.planning_scene_message.world.collision_objects
            }
            removing_missing_object = (
                collision_object.operation == CollisionObject.REMOVE
                and collision_object.id not in known_ids
            )
            applied = (
                True
                if removing_missing_object
                else scene.apply_collision_object(collision_object)
            )
            if object_color is not None:
                scene_message = scene.planning_scene_message
                colors = scene_message.object_colors
                colors[:] = [
                    item for item in colors if item.id != object_color.id
                ]
                colors.append(object_color)
            scene.current_state.update()
        if applied is False:
            raise PlanningSceneError(
                f"MoveIt rejected collision object '{collision_object.id}'"
            )

        # 再发布 is_diff 增量，让 move_group 和 RViz 同步世界物体/颜色。
        scene_update = PlanningScene()
        scene_update.is_diff = True
        scene_update.world.collision_objects.append(collision_object)
        if object_color is not None:
            scene_update.object_colors.append(object_color)
        self._publisher.publish(scene_update)

    def _publish_attached(self, attached_object: Any) -> None:
        """向 /planning_scene 发布“附着物体”增量消息。"""
        from moveit_msgs.msg import PlanningScene

        scene_update = PlanningScene()
        scene_update.is_diff = True
        scene_update.robot_state.is_diff = True
        scene_update.robot_state.attached_collision_objects.append(
            attached_object
        )
        self._publisher.publish(scene_update)

    def _wait_for_attachment_state(
        self, object_id: str, *, attached: bool
    ) -> bool:
        """短暂轮询本地场景，确认附着/解除附着状态已真正生效。"""
        deadline = time.monotonic() + self._scene_update_wait_seconds
        while True:
            with self._monitor.read_only() as scene:
                message = scene.planning_scene_message
                attached_ids = {
                    item.object.id
                    for item in message.robot_state.attached_collision_objects
                }
                world_ids = {
                    item.id for item in message.world.collision_objects
                }
            state_matches = (
                object_id in attached_ids and object_id not in world_ids
                if attached
                else object_id not in attached_ids and object_id in world_ids
            )
            if state_matches:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
