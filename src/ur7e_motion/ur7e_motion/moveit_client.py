"""UR7e 的可复用 MoveItPy 运动客户端。

这个文件把 MoveItPy 的底层调用封装为“到关节角”“到绝对位姿”和“相对平移”三种
高层动作。固定任务、避障、抓取放置等节点都通过本类控制机械臂。
"""

from collections.abc import Sequence
import math
import time
from typing import Any

from ur7e_motion.validation import normalize_quaternion, validate_numeric_vector


# ============================== 固定约定 ==============================
# 这些名称来自 URDF/SRDF；任务代码不应在每个节点里重复手写它们。
#
# PLANNING_GROUP：MoveIt 的机械臂规划组，链路为 base_link → robotiq_tcp。
# END_EFFECTOR_LINK：位姿目标实际作用的末端坐标系，即两指之间的 TCP。
# REFERENCE_FRAME：所有相对平移默认解释所在的坐标系。
PLANNING_GROUP = "ur_manipulator"
END_EFFECTOR_LINK = "robotiq_tcp"
REFERENCE_FRAME = "base_link"
# 发送关节目标时的固定顺序；输入的 6 个角度必须严格对应这个顺序。
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
# 每个目标独立规划 3 次，再从可行解中选择累计关节运动量最短的一条。
PLANNING_CANDIDATES = 3
# Gazebo 中 /clock 与 /joint_states 可能相差一个调度周期，因此允许状态最多早 0.5 秒。
STATE_FRESHNESS_TOLERANCE_SECONDS = 0.5


class UR7eMotionError(RuntimeError):
    """本模块所有机械臂规划和执行异常的父类。"""


class MoveItDependencyError(UR7eMotionError):
    """未安装 MoveItPy Python 绑定时抛出。"""


class RobotStateUnavailableError(UR7eMotionError):
    """在规定时间内无法获得完整且足够新的机器人状态时抛出。"""


class MotionPlanningError(UR7eMotionError):
    """目标被 MoveIt 拒绝，或无法找到无碰撞轨迹时抛出。"""


class MotionExecutionError(UR7eMotionError):
    """轨迹已规划成功，但控制器执行失败时抛出。"""


class UR7eMoveItClient:
    """用一个 MoveItPy 实例连续规划并执行 UR7e 动作。

    调用方负责创建/销毁传入的 rclpy 节点，以及初始化/关闭 rclpy。
    启动该任务节点的 launch 文件还必须传入 MoveIt 所需的机器人模型、SRDF、
    运动学、规划器和控制器参数。
    """

    def __init__(
        self,
        node: Any,
        *,
        moveit_node_name: str = "ur7e_motion_moveit",
        state_wait_seconds: float = 5.0,
        controller_discovery_seconds: float = 3.0,
    ) -> None:
        """初始化 MoveItPy、机械臂规划组件和 PlanningScene 状态监视器。"""
        # 延迟导入：单元测试无需安装 moveit_py；真正运行任务时才要求该依赖存在。
        try:
            from moveit.planning import MoveItPy, PlanRequestParameters
        except ImportError as exc:
            raise MoveItDependencyError(
                "moveit_py is not installed. Run: "
                "sudo apt-get install ros-jazzy-moveit-py"
            ) from exc

        # 外部任务节点：用于日志、ROS 时钟和接收 /clock。
        self._node = node
        self._logger = node.get_logger()
        self._state_wait_seconds = float(state_wait_seconds)
        # MoveItPy 内部会创建自己的 rclcpp MoveIt 节点和 MoveItCpp 实例。
        self._moveit = MoveItPy(node_name=moveit_node_name)
        # 取得 SRDF 中名为 ur_manipulator 的规划组件。
        self._arm = self._moveit.get_planning_component(PLANNING_GROUP)
        # 监视 /joint_states、PlanningScene 等信息，用于确定规划起点和碰撞环境。
        self._scene_monitor = self._moveit.get_planning_scene_monitor()
        # 空字符串表示使用 launch 文件加载的默认规划参数（例如 OMPL/RRTConnect）。
        self._plan_parameters = PlanRequestParameters(self._moveit, "")

        # 新建 MoveItPy 时也会新建 ROS 2 Action 客户端。这里预留一次 DDS 发现时间，
        # 让它先连接轨迹控制器，避免多步骤任务的第一段运动才发生连接延迟。
        if controller_discovery_seconds > 0.0:
            time.sleep(float(controller_discovery_seconds))

    @property
    def moveit_instance(self) -> Any:
        """返回内部 MoveItPy；仅供 SceneManager 等高级集成使用。"""
        return self._moveit

    @property
    def planning_scene_monitor(self) -> Any:
        """返回内部 PlanningSceneMonitor；用于读取最新机器人状态和场景。"""
        return self._scene_monitor

    def get_current_pose(self) -> Any:
        """返回当前 robotiq_tcp 在 base_link 下的 PoseStamped 位姿。"""
        # 先取得与 /joint_states 同步的 RobotState，再通过正运动学计算 TCP 位姿。
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
        """移动到 6 个绝对关节角，单位为弧度。"""
        from moveit.core.robot_state import RobotState

        # 输入检查会确保恰好 6 个有限数值，防止错误数据进入规划器。
        joint_positions = validate_numeric_vector("positions", positions, 6)
        # 每一步都从最新实际状态规划，而不是假定上一步一定精确到位。
        self._prepare_current_start_state()

        # RobotState 是 MoveIt 的“某一时刻完整机器人关节状态”。此处只设置 6 个臂关节。
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
        """移动 robotiq_tcp 到绝对位姿。

        ``position`` 为米单位的 ``[x, y, z]``，``orientation`` 为四元数
        ``[x, y, z, w]``；默认都相对于 ``base_link``。
        """
        target_position = validate_numeric_vector("position", position, 3)
        target_orientation = normalize_quaternion(orientation)
        if not frame_id.strip():
            raise ValueError("frame_id must not be empty")

        # 四元数会归一化，避免非单位四元数导致无效姿态。
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
        """在 base_link 下将当前 TCP 相对平移 ``[dx, dy, dz]`` 米。

        此函数只改变位置，目标姿态保持调用瞬间的夹爪姿态不变。它不是笛卡尔直线控制：
        MoveIt/OMPL 仍会自行寻找连接起点和终点的无碰撞关节轨迹。
        """
        offset = validate_numeric_vector("translation", translation, 3)
        # 从最新 TCP 位姿加上偏移量，构造一个新的绝对位姿目标。
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
        """等待最新状态，并把它设置为下一次规划的起点。"""
        self._logger.info("Waiting for a current UR7e joint state")
        # 请求“当前时间稍早”的状态。通常 100 Hz 的 /joint_states 仅晚几毫秒；但在
        # Gazebo 仿真时间下，/clock 与 /joint_states 由不同执行器处理，可能相差一次
        # 调度周期。若强制要求时间戳恰好等于 now，会错误拒绝本来足够新的完整状态。
        import rclpy
        from rclpy.duration import Duration

        current_time = self._node.get_clock().now()
        # use_sim_time 节点至少 spin 一次后才会接收 /clock。任务节点常常刚启动就规划，
        # 还未因 Action 客户端而 spin，因此这里主动“预热”ROS 时钟。
        for _ in range(10):
            if current_time.nanoseconds > 0:
                break
            rclpy.spin_once(self._node, timeout_sec=0.1)
            current_time = self._node.get_clock().now()

        freshness = Duration(seconds=STATE_FRESHNESS_TOLERANCE_SECONDS)
        if current_time.nanoseconds > freshness.nanoseconds:
            minimum_state_time = current_time - freshness
        else:
            minimum_state_time = current_time
        state_received = self._scene_monitor.wait_for_current_robot_state(
            minimum_state_time, self._state_wait_seconds
        )
        if not state_received:
            raise RobotStateUnavailableError(
                "No current robot state received within "
                f"{self._state_wait_seconds:.1f} seconds"
            )

        # 将 PlanningComponent 的起点明确设为当前状态；这是连续任务能从上一步实际
        # 终点继续规划、而不是从某个旧默认姿态规划的关键。
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
        """将列表形式的位置与四元数转换为 ROS PoseStamped 消息。"""
        from geometry_msgs.msg import PoseStamped

        goal = PoseStamped()
        goal.header.frame_id = frame_id
        # 带上当前 ROS 时间；Gazebo 模式下这里是 /clock 提供的仿真时间。
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
        """把 PoseStamped 写入 MoveIt，并指定目标作用在 robotiq_tcp。"""
        if not self._arm.set_goal_state(
            pose_stamped_msg=goal, pose_link=END_EFFECTOR_LINK
        ):
            raise MotionPlanningError("MoveIt rejected the requested pose goal")

    def _plan_and_execute(self) -> None:
        """多次规划，选择关节路径较短的可行轨迹，然后交给控制器执行。"""
        candidates: list[tuple[float, Any]] = []
        last_error = None
        for candidate_number in range(1, PLANNING_CANDIDATES + 1):
            # 每次 plan() 都会让 OMPL 重新随机采样，因此可能得到不同的无碰撞路径。
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

            # 以六关节在相邻轨迹点间的欧氏距离累计值衡量“关节空间路径长度”。
            # 它不是严格的时间最短或能耗最小指标，但能剔除明显绕远的候选路径。
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

        # 只在至少存在一条规划成功且经 MoveIt 验证的轨迹时才发送给控制器。
        path_length, plan_result = min(candidates, key=lambda item: item[0])
        self._logger.info(
            f"Selected shortest of {len(candidates)} valid candidates: "
            f"joint-space length={path_length:.4f} rad; executing trajectory"
        )
        # controllers=[] 表示由 MoveIt 根据 moveit_controllers.yaml 自动选择能够控制
        # 这些关节的控制器，而不是在代码中写死控制器名称。
        execution_status = self._moveit.execute(
            plan_result.trajectory, controllers=[]
        )
        if not execution_status:
            raise MotionExecutionError(
                f"Trajectory execution failed: {execution_status.status}"
            )
        self._logger.info("Trajectory execution succeeded")


def _joint_path_length(trajectory: Any) -> float:
    """计算轨迹的累计关节空间长度，用于比较多个候选路径。"""
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
        # math.dist() 计算两个六维关节角向量的欧氏距离，再对所有相邻点累加。
        path_length += math.dist(previous.positions, current.positions)
    return path_length
