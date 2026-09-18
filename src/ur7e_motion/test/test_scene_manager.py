"""Tests for world and attached collision-object management."""

from contextlib import contextmanager
from types import SimpleNamespace

from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from moveit_msgs.msg import PlanningScene
import pytest

from ur7e_motion import PlanningSceneError, SceneManager


class FakeAllowedCollisionMatrix:
    """Record changes to object/link collision permissions."""

    def __init__(self):
        self.entries = []
        self.removed = []

    def set_entry(self, first, second, allowed):
        self.entries.append((first, second, allowed))

    def remove_entry(self, first, second):
        self.removed.append((first, second))


class FakePlanningScene:
    """Apply enough MoveIt scene behavior for deterministic unit tests."""

    def __init__(self):
        self.message = PlanningScene()
        self.allowed_collision_matrix = FakeAllowedCollisionMatrix()
        self.current_state = SimpleNamespace(update=lambda: None)

    @property
    def planning_scene_message(self):
        return self.message

    def apply_collision_object(self, collision_object):
        objects = self.message.world.collision_objects
        objects[:] = [item for item in objects if item.id != collision_object.id]
        if collision_object.operation == CollisionObject.ADD:
            objects.append(collision_object)
        return True

    def process_attached_collision_object(self, request):
        if request.object.operation == CollisionObject.ADD:
            matching = [
                item
                for item in self.message.world.collision_objects
                if item.id == request.object.id
            ]
            if not matching:
                return False
            self.message.world.collision_objects[:] = [
                item
                for item in self.message.world.collision_objects
                if item.id != request.object.id
            ]
            attached = AttachedCollisionObject()
            attached.link_name = request.link_name
            attached.object = matching[0]
            attached.touch_links = request.touch_links
            self.message.robot_state.attached_collision_objects.append(attached)
            return True

        matching = [
            item
            for item in self.message.robot_state.attached_collision_objects
            if item.object.id == request.object.id
        ]
        if not matching:
            return False
        self.message.robot_state.attached_collision_objects[:] = [
            item
            for item in self.message.robot_state.attached_collision_objects
            if item.object.id != request.object.id
        ]
        world_object = matching[0].object
        world_object.operation = CollisionObject.ADD
        self.message.world.collision_objects.append(world_object)
        return True


class FakeMonitor:
    """Expose a write context around one fake scene."""

    def __init__(self, scene):
        self.scene = scene

    @contextmanager
    def read_write(self):
        yield self.scene

    @contextmanager
    def read_only(self):
        yield self.scene


class FakePublisher:
    """Record PlanningScene diffs."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    """Accept informational messages from SceneManager."""

    def info(self, message):
        pass


def _manager():
    fake_scene = FakePlanningScene()
    manager = SceneManager.__new__(SceneManager)
    manager._logger = FakeLogger()
    manager._monitor = FakeMonitor(fake_scene)
    manager._known_object_ids = set()
    manager._attached_object_ids = set()
    manager._scene_update_wait_seconds = 0.0
    manager._publisher = FakePublisher()
    return manager, fake_scene


def test_object_moves_from_world_to_robot_and_back():
    """Attach and detach preserve one object while changing ownership."""
    manager, fake_scene = _manager()
    manager.add_box("box", [0.03, 0.03, 0.06], [0.4, 0.2, 0.3])

    manager.attach_object(
        "box", link_name="robotiq_tcp", touch_links=["finger"]
    )

    assert fake_scene.message.world.collision_objects == []
    attached = fake_scene.message.robot_state.attached_collision_objects
    assert len(attached) == 1
    assert attached[0].object.id == "box"
    assert attached[0].link_name == "robotiq_tcp"
    assert list(attached[0].touch_links) == ["finger"]
    assert manager._known_object_ids == set()
    assert manager._attached_object_ids == {"box"}

    manager.detach_object("box")

    assert fake_scene.message.robot_state.attached_collision_objects == []
    assert [item.id for item in fake_scene.message.world.collision_objects] == [
        "box"
    ]
    assert manager._known_object_ids == {"box"}
    assert manager._attached_object_ids == set()


def test_box_color_is_published_for_rviz():
    """An optional color is sent with the collision-object scene diff."""
    manager, fake_scene = _manager()

    manager.add_box(
        "box",
        [0.03, 0.03, 0.06],
        [0.4, 0.2, 0.3],
        color=[0.1, 0.2, 0.3, 0.8],
    )

    color = manager._publisher.messages[-1].object_colors[0]
    assert color.id == "box"
    assert [color.color.r, color.color.g, color.color.b, color.color.a] == [
        0.1,
        0.2,
        0.3,
        0.8,
    ]
    assert fake_scene.message.object_colors[0].id == "box"


def test_invalid_box_color_is_rejected():
    """RGBA colors must have four normalized components."""
    manager, _ = _manager()

    with pytest.raises(ValueError, match="color values"):
        manager.add_box(
            "box",
            [0.03, 0.03, 0.06],
            [0.4, 0.2, 0.3],
            color=[1.1, 0.2, 0.3, 1.0],
        )


def test_touch_permissions_are_set_and_removed():
    """Only requested object/link pairs are temporarily allowed."""
    manager, fake_scene = _manager()

    manager.set_object_touch_allowed("box", ["left", "right"], True)
    manager.set_object_touch_allowed("box", ["left", "right"], False)

    assert fake_scene.allowed_collision_matrix.entries == [
        ("box", "left", True),
        ("box", "right", True),
    ]
    assert fake_scene.allowed_collision_matrix.removed == [
        ("box", "left"),
        ("box", "right"),
    ]
    assert len(manager._publisher.messages) == 2


def test_clear_detaches_before_removing_object():
    """Cleanup leaves neither world nor attached copies behind."""
    manager, fake_scene = _manager()
    manager.add_box("box", [0.03, 0.03, 0.06], [0.4, 0.2, 0.3])
    manager.attach_object("box", touch_links=["finger"])

    manager.clear()

    assert fake_scene.message.world.collision_objects == []
    assert fake_scene.message.robot_state.attached_collision_objects == []
    assert manager._known_object_ids == set()
    assert manager._attached_object_ids == set()


def test_unknown_object_cannot_be_attached():
    """Attaching a missing world object reports a scene error."""
    manager, _ = _manager()

    with pytest.raises(PlanningSceneError, match="unknown world object"):
        manager.attach_object("missing", touch_links=["finger"])
