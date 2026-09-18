"""Collision-object management for a MoveItPy planning scene."""

from collections.abc import Sequence
import time
from typing import Any

from ur7e_motion.validation import normalize_quaternion, validate_numeric_vector


PLANNING_SCENE_TOPIC = "/planning_scene"


class PlanningSceneError(RuntimeError):
    """Raised when a collision object cannot be applied to the scene."""


class SceneManager:
    """Manage world and attached collision objects used by MoveIt planning."""

    def __init__(
        self,
        node: Any,
        planning_scene_monitor: Any,
        *,
        planning_scene_topic: str = PLANNING_SCENE_TOPIC,
        scene_update_wait_seconds: float = 0.5,
    ) -> None:
        """Create a manager for a MoveItPy planning scene monitor."""
        from moveit_msgs.msg import PlanningScene
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self._logger = node.get_logger()
        self._monitor = planning_scene_monitor
        self._known_object_ids: set[str] = set()
        self._attached_object_ids: set[str] = set()
        self._scene_update_wait_seconds = float(scene_update_wait_seconds)
        if self._scene_update_wait_seconds < 0.0:
            raise ValueError("scene_update_wait_seconds must not be negative")
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
        """Add or replace an axis-aligned box collision object.

        ``color`` is an optional RGBA vector with components in ``[0.0, 1.0]``.
        It affects RViz rendering only, never collision checking or planning.
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

        self._apply(collision_object, object_color=object_color)
        self._known_object_ids.add(clean_id)
        self._logger.info(
            f"Added collision box '{clean_id}': size={dimensions}, "
            f"position={coordinates}, frame={frame_id.strip()}"
        )

    def remove_object(self, object_id: str) -> None:
        """Remove one world or attached collision object from the scene."""
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
        """Move an existing world object onto a robot link."""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        clean_id = self._validate_id(object_id)
        clean_link = self._validate_link_name("link_name", link_name)
        clean_touch_links = self._validate_link_names(touch_links)

        attached_object = AttachedCollisionObject()
        attached_object.link_name = clean_link
        attached_object.object.id = clean_id
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = clean_touch_links

        with self._monitor.read_write() as scene:
            message = scene.planning_scene_message
            world_ids = {
                item.id for item in message.world.collision_objects
            }
            if clean_id not in world_ids:
                raise PlanningSceneError(
                    f"Cannot attach unknown world object '{clean_id}'"
                )
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
        """Detach an object and preserve it at its current world pose."""
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

        clean_id = self._validate_id(object_id)
        attached_object = AttachedCollisionObject()
        attached_object.object.id = clean_id
        attached_object.object.operation = CollisionObject.REMOVE

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
        """Allow or clear collisions between an object and selected links."""
        from moveit_msgs.msg import PlanningScene

        clean_id = self._validate_id(object_id)
        clean_links = self._validate_link_names(link_names)
        if not isinstance(allowed, bool):
            raise ValueError("allowed must be a bool")

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
        """Remove all objects that were added through this manager."""
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

        scene_update = PlanningScene()
        scene_update.is_diff = True
        scene_update.world.collision_objects.append(collision_object)
        if object_color is not None:
            scene_update.object_colors.append(object_color)
        self._publisher.publish(scene_update)

    def _publish_attached(self, attached_object: Any) -> None:
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
