"""Reusable MoveItPy motion client for the UR7e manipulator."""

from collections.abc import Sequence
import math
import time
from typing import Any

from ur7e_motion.validation import normalize_quaternion, validate_numeric_vector


PLANNING_GROUP = "ur_manipulator"
END_EFFECTOR_LINK = "robotiq_tcp"
REFERENCE_FRAME = "base_link"
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
PLANNING_CANDIDATES = 3


class UR7eMotionError(RuntimeError):
    """Base exception for UR7e planning and execution failures."""


class MoveItDependencyError(UR7eMotionError):
    """Raised when the MoveItPy Python binding is unavailable."""


class RobotStateUnavailableError(UR7eMotionError):
    """Raised when no recent complete robot state can be obtained."""


class MotionPlanningError(UR7eMotionError):
    """Raised when MoveIt rejects a goal or cannot plan a path."""


class MotionExecutionError(UR7eMotionError):
    """Raised when a planned trajectory cannot be executed successfully."""


class UR7eMoveItClient:
    """Plan and execute repeatable UR7e motions with one MoveItPy instance.

    The caller owns the supplied rclpy node and is responsible for initializing
    and shutting down rclpy. MoveIt parameters must be supplied by the launch
    file that starts the process.
    """

    def __init__(
        self,
        node: Any,
        *,
        moveit_node_name: str = "ur7e_motion_moveit",
        state_wait_seconds: float = 5.0,
        controller_discovery_seconds: float = 3.0,
    ) -> None:
        """Initialize MoveIt, the arm planning component and state monitor."""
        try:
            from moveit.planning import MoveItPy, PlanRequestParameters
        except ImportError as exc:
            raise MoveItDependencyError(
                "moveit_py is not installed. Run: "
                "sudo apt-get install ros-jazzy-moveit-py"
            ) from exc

        self._node = node
        self._logger = node.get_logger()
        self._state_wait_seconds = float(state_wait_seconds)
        self._moveit = MoveItPy(node_name=moveit_node_name)
        self._arm = self._moveit.get_planning_component(PLANNING_GROUP)
        self._scene_monitor = self._moveit.get_planning_scene_monitor()
        self._plan_parameters = PlanRequestParameters(self._moveit, "")

        # A fresh MoveItPy instance also creates a fresh rclcpp action client.
        # Allow DDS discovery to connect it to the trajectory controller once,
        # rather than delaying every motion in a multi-step task.
        if controller_discovery_seconds > 0.0:
            time.sleep(float(controller_discovery_seconds))

    @property
    def moveit_instance(self) -> Any:
        """Return the owned MoveItPy instance for advanced integrations."""
        return self._moveit

    @property
    def planning_scene_monitor(self) -> Any:
        """Return the planning scene monitor used by this motion client."""
        return self._scene_monitor

    def get_current_pose(self) -> Any:
        """Return the current gripper TCP pose as a PoseStamped in base_link."""
        current_state = self._prepare_current_start_state()
        current_pose = current_state.get_pose(END_EFFECTOR_LINK)
        return self._make_pose_goal(
            [
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z,
            ],
            [
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z,
                current_pose.orientation.w,
            ],
            REFERENCE_FRAME,
        )

    def move_to_joint(self, positions: Sequence[float]) -> None:
        """Move to six absolute joint positions, expressed in radians."""
        from moveit.core.robot_state import RobotState

        joint_positions = validate_numeric_vector("positions", positions, 6)
        self._prepare_current_start_state()

        goal_state = RobotState(self._moveit.get_robot_model())
        goal_state.joint_positions = dict(
            zip(JOINT_NAMES, joint_positions, strict=True)
        )
        goal_state.update()
        if not self._arm.set_goal_state(robot_state=goal_state):
            raise MotionPlanningError("MoveIt rejected the requested joint goal")

        self._logger.info(f"Planning joint goal (rad): {joint_positions}")
        self._plan_and_execute()

    def move_to_pose(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        *,
        frame_id: str = REFERENCE_FRAME,
    ) -> None:
        """Move the gripper TCP to an absolute pose.

        Position is [x, y, z] in metres. Orientation is the quaternion
        [x, y, z, w]. The default reference frame is base_link.
        """
        target_position = validate_numeric_vector("position", position, 3)
        target_orientation = normalize_quaternion(orientation)
        if not frame_id.strip():
            raise ValueError("frame_id must not be empty")

        self._prepare_current_start_state()
        goal = self._make_pose_goal(
            target_position, target_orientation, frame_id.strip()
        )
        self._set_pose_goal(goal)

        self._logger.info(
            f"Planning absolute {END_EFFECTOR_LINK} pose in {frame_id}: "
            f"position={target_position}, orientation={target_orientation}"
        )
        self._plan_and_execute()

    def move_by_translation(self, translation: Sequence[float]) -> None:
        """Translate the gripper TCP relative to its current pose in base_link.

        Translation is [dx, dy, dz] in metres. The current tool orientation is
        preserved as the goal orientation.
        """
        offset = validate_numeric_vector("translation", translation, 3)
        current_state = self._prepare_current_start_state()
        current_pose = current_state.get_pose(END_EFFECTOR_LINK)

        position = [
            current_pose.position.x + offset[0],
            current_pose.position.y + offset[1],
            current_pose.position.z + offset[2],
        ]
        orientation = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ]
        goal = self._make_pose_goal(position, orientation, REFERENCE_FRAME)
        self._set_pose_goal(goal)

        self._logger.info(
            f"Planning {END_EFFECTOR_LINK} translation in {REFERENCE_FRAME} "
            f"(m): {offset}"
        )
        self._plan_and_execute()

    def _prepare_current_start_state(self) -> Any:
        self._logger.info("Waiting for a current UR7e joint state")
        state_received = self._scene_monitor.wait_for_current_robot_state(
            self._node.get_clock().now(), self._state_wait_seconds
        )
        if not state_received:
            raise RobotStateUnavailableError(
                "No current robot state received within "
                f"{self._state_wait_seconds:.1f} seconds"
            )

        self._arm.set_start_state_to_current_state()
        current_state = self._arm.get_start_state()
        if current_state is None:
            raise RobotStateUnavailableError(
                "MoveIt did not provide a current start state"
            )
        return current_state

    def _make_pose_goal(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
        frame_id: str,
    ) -> Any:
        from geometry_msgs.msg import PoseStamped

        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.position.x = position[0]
        goal.pose.position.y = position[1]
        goal.pose.position.z = position[2]
        goal.pose.orientation.x = orientation[0]
        goal.pose.orientation.y = orientation[1]
        goal.pose.orientation.z = orientation[2]
        goal.pose.orientation.w = orientation[3]
        return goal

    def _set_pose_goal(self, goal: Any) -> None:
        if not self._arm.set_goal_state(
            pose_stamped_msg=goal, pose_link=END_EFFECTOR_LINK
        ):
            raise MotionPlanningError("MoveIt rejected the requested pose goal")

    def _plan_and_execute(self) -> None:
        candidates: list[tuple[float, Any]] = []
        last_error = None
        for candidate_number in range(1, PLANNING_CANDIDATES + 1):
            candidate = self._arm.plan(
                single_plan_parameters=self._plan_parameters
            )
            if not candidate:
                last_error = candidate.error_code
                self._logger.warning(
                    f"Planning candidate {candidate_number}/"
                    f"{PLANNING_CANDIDATES} failed: {last_error}; "
                    "excluding it from trajectory selection"
                )
                continue

            path_length = _joint_path_length(candidate.trajectory)
            candidates.append((path_length, candidate))
            self._logger.info(
                f"Planning candidate {candidate_number}/"
                f"{PLANNING_CANDIDATES} succeeded: "
                f"joint-space length={path_length:.4f} rad"
            )

        if not candidates:
            raise MotionPlanningError(
                "Motion planning produced no valid trajectory after "
                f"{PLANNING_CANDIDATES} candidates: {last_error}"
            )

        path_length, plan_result = min(candidates, key=lambda item: item[0])
        self._logger.info(
            f"Selected shortest of {len(candidates)} valid candidates: "
            f"joint-space length={path_length:.4f} rad; executing trajectory"
        )
        execution_status = self._moveit.execute(
            plan_result.trajectory, controllers=[]
        )
        if not execution_status:
            raise MotionExecutionError(
                f"Trajectory execution failed: {execution_status.status}"
            )
        self._logger.info("Trajectory execution succeeded")


def _joint_path_length(trajectory: Any) -> float:
    """Return cumulative Euclidean joint travel for a robot trajectory."""
    trajectory_message = trajectory.get_robot_trajectory_msg()
    points = trajectory_message.joint_trajectory.points
    if not points:
        raise MotionPlanningError("Planned trajectory contains no joint points")

    path_length = 0.0
    for previous, current in zip(points, points[1:]):
        if len(previous.positions) != len(current.positions):
            raise MotionPlanningError(
                "Planned trajectory has inconsistent joint dimensions"
            )
        path_length += math.dist(previous.positions, current.positions)
    return path_length
