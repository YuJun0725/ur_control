"""Detect red, green and blue cubes and publish their 3D base coordinates."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped
import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .color_detection import (
    create_hsv_mask,
    depth_to_meters,
    estimate_cuboid_pose,
    find_color_candidates,
    quaternion_rotation_matrix,
)


DISPLAY_COLORS = {
    'red': (0, 0, 255),
    'green': (0, 220, 0),
    'blue': (255, 100, 0),
}


class ColorCubeDetector(Node):
    """Synchronize RGB-D frames and localize one cube of each configured color."""

    def __init__(self) -> None:
        super().__init__('color_cube_detector')
        self._declare_parameters()
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._min_area = float(self.get_parameter('min_contour_area').value)
        self._max_area = float(self.get_parameter('max_contour_area').value)
        self._min_aspect = float(self.get_parameter('min_aspect_ratio').value)
        self._max_aspect = float(self.get_parameter('max_aspect_ratio').value)
        self._min_fill = float(self.get_parameter('min_fill_ratio').value)
        self._min_depth = float(self.get_parameter('min_depth').value)
        self._max_depth = float(self.get_parameter('max_depth').value)
        self._object_size = np.asarray(
            self.get_parameter('object_size').value, dtype=np.float64
        )
        if (self._object_size.shape != (3,)
                or not np.all(np.isfinite(self._object_size))
                or np.any(self._object_size <= 0)):
            raise ValueError('object_size must contain 3 positive values')
        self._top_band = float(self.get_parameter('top_band').value)
        self._min_top_points = int(self.get_parameter('min_top_points').value)
        if (not math.isfinite(self._top_band)
                or self._top_band <= 0 or self._min_top_points < 3):
            raise ValueError('top_band and min_top_points must be positive')
        self._workspace_min = np.asarray(
            self.get_parameter('workspace_min').value, dtype=np.float64
        )
        self._workspace_max = np.asarray(
            self.get_parameter('workspace_max').value, dtype=np.float64
        )
        if self._workspace_min.shape != (3,) or self._workspace_max.shape != (3,):
            raise ValueError('workspace_min and workspace_max must contain 3 values')
        self._hsv_ranges = self._load_hsv_ranges()

        self._point_publishers = {
            color: self.create_publisher(
                PointStamped, f'~/detections/{color}/center', 10
            )
            for color in self._hsv_ranges
        }
        self._pose_publishers = {
            color: self.create_publisher(
                PoseStamped, f'~/detections/{color}/pose', 10
            )
            for color in self._hsv_ranges
        }
        self._debug_publisher = self.create_publisher(Image, '~/debug_image', 10)
        self._marker_publisher = self.create_publisher(
            MarkerArray, '~/markers', 10
        )

        color_topic = str(self.get_parameter('color_topic').value)
        depth_topic = str(self.get_parameter('depth_topic').value)
        info_topic = str(self.get_parameter('camera_info_topic').value)
        self._color_subscriber = message_filters.Subscriber(
            self, Image, color_topic, qos_profile=qos_profile_sensor_data
        )
        self._depth_subscriber = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        self._info_subscriber = message_filters.Subscriber(
            self, CameraInfo, info_topic, qos_profile=qos_profile_sensor_data
        )
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [
                self._color_subscriber,
                self._depth_subscriber,
                self._info_subscriber,
            ],
            queue_size=10,
            slop=0.05,
        )
        self._synchronizer.registerCallback(self._synchronized_callback)
        self._last_log_ns = 0
        self.get_logger().info(
            'Detecting red, green and blue cubes; publishing coordinates in '
            f'{self._base_frame}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/color/camera_info'
        )
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_contour_area', 80.0)
        self.declare_parameter('max_contour_area', 3500.0)
        self.declare_parameter('min_aspect_ratio', 0.45)
        self.declare_parameter('max_aspect_ratio', 1.80)
        self.declare_parameter('min_fill_ratio', 0.35)
        self.declare_parameter('min_depth', 0.10)
        self.declare_parameter('max_depth', 3.0)
        self.declare_parameter('object_size', [0.035, 0.035, 0.060])
        self.declare_parameter('top_band', 0.006)
        self.declare_parameter('min_top_points', 20)
        self.declare_parameter('workspace_min', [-0.32, 0.25, 0.29])
        self.declare_parameter('workspace_max', [0.32, 0.70, 0.40])
        self.declare_parameter('red_hsv_lower_1', [0, 100, 60])
        self.declare_parameter('red_hsv_upper_1', [12, 255, 255])
        self.declare_parameter('red_hsv_lower_2', [168, 100, 60])
        self.declare_parameter('red_hsv_upper_2', [179, 255, 255])
        self.declare_parameter('green_hsv_lower', [38, 80, 45])
        self.declare_parameter('green_hsv_upper', [88, 255, 255])
        self.declare_parameter('blue_hsv_lower', [92, 100, 45])
        self.declare_parameter('blue_hsv_upper', [132, 255, 255])

    def _parameter_triplet(self, name: str) -> tuple[int, int, int]:
        values = self.get_parameter(name).value
        if len(values) != 3:
            raise ValueError(f'{name} must contain exactly 3 integers')
        return tuple(int(value) for value in values)

    def _load_hsv_ranges(self):
        return {
            'red': [
                (
                    self._parameter_triplet('red_hsv_lower_1'),
                    self._parameter_triplet('red_hsv_upper_1'),
                ),
                (
                    self._parameter_triplet('red_hsv_lower_2'),
                    self._parameter_triplet('red_hsv_upper_2'),
                ),
            ],
            'green': [
                (
                    self._parameter_triplet('green_hsv_lower'),
                    self._parameter_triplet('green_hsv_upper'),
                )
            ],
            'blue': [
                (
                    self._parameter_triplet('blue_hsv_lower'),
                    self._parameter_triplet('blue_hsv_upper'),
                )
            ],
        }

    def _synchronized_callback(
        self, color_msg: Image, depth_msg: Image, camera_info: CameraInfo
    ) -> None:
        try:
            bgr_image = self._bridge.imgmsg_to_cv2(
                color_msg, desired_encoding='bgr8'
            )
            raw_depth = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough'
            )
            depth_meters = depth_to_meters(raw_depth, depth_msg.encoding)
        except (CvBridgeError, ValueError) as error:
            self.get_logger().error(f'Unable to convert RGB-D images: {error}')
            return
        if bgr_image.shape[:2] != depth_meters.shape[:2]:
            self.get_logger().error('Color and depth image dimensions differ')
            return
        fx, fy = float(camera_info.k[0]), float(camera_info.k[4])
        cx, cy = float(camera_info.k[2]), float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning('CameraInfo contains invalid focal lengths')
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                color_msg.header.frame_id,
                Time.from_msg(color_msg.header.stamp),
                timeout=Duration(seconds=0.10),
            )
        except TransformException as error:
            self.get_logger().warning(f'Camera transform unavailable: {error}')
            return

        rotation = transform.transform.rotation
        translation = transform.transform.translation
        camera_to_base = np.eye(4)
        camera_to_base[:3, :3] = quaternion_rotation_matrix(
            (rotation.x, rotation.y, rotation.z, rotation.w)
        )
        camera_to_base[:3, 3] = (
            translation.x, translation.y, translation.z
        )

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        debug_image = bgr_image.copy()
        detections: dict[str, PointStamped] = {}
        poses: dict[str, PoseStamped] = {}
        for color, ranges in self._hsv_ranges.items():
            mask = create_hsv_mask(hsv_image, ranges)
            candidates = find_color_candidates(
                mask,
                self._min_area,
                self._max_area,
                self._min_aspect,
                self._max_aspect,
                self._min_fill,
            )
            for candidate in candidates:
                estimate = estimate_cuboid_pose(
                    depth_meters,
                    candidate.contour,
                    (fx, fy, cx, cy),
                    camera_to_base,
                    self._object_size,
                    min_depth=self._min_depth,
                    max_depth=self._max_depth,
                    top_band=self._top_band,
                    min_top_points=self._min_top_points,
                )
                if estimate is None:
                    continue
                point_array = np.asarray(estimate.center)
                if not np.all(point_array >= self._workspace_min) or not np.all(
                    point_array <= self._workspace_max
                ):
                    continue
                base_point = PointStamped()
                base_point.header.stamp = color_msg.header.stamp
                base_point.header.frame_id = self._base_frame
                (base_point.point.x, base_point.point.y,
                 base_point.point.z) = estimate.center
                pose = PoseStamped()
                pose.header = base_point.header
                pose.pose.position = base_point.point
                pose.pose.orientation.z = math.sin(estimate.yaw / 2)
                pose.pose.orientation.w = math.cos(estimate.yaw / 2)
                detections[color] = base_point
                poses[color] = pose
                self._point_publishers[color].publish(base_point)
                self._pose_publishers[color].publish(pose)
                self._draw_detection(debug_image, color, candidate, base_point)
                break

        self._publish_markers(poses, color_msg)
        debug_msg = self._bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header = color_msg.header
        self._debug_publisher.publish(debug_msg)
        self._log_detections(detections)

    @staticmethod
    def _draw_detection(image, color, candidate, point) -> None:
        x, y, width, height = candidate.bounding_box
        display_color = DISPLAY_COLORS[color]
        cv2.rectangle(image, (x, y), (x + width, y + height), display_color, 2)
        label = (
            f'{color}: ({point.point.x:.3f}, {point.point.y:.3f}, '
            f'{point.point.z:.3f}) m'
        )
        cv2.putText(
            image,
            label,
            (max(0, x - 5), max(18, y - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            display_color,
            1,
            cv2.LINE_AA,
        )

    def _publish_markers(self, detections, source_msg) -> None:
        marker_array = MarkerArray()
        for marker_id, color in enumerate(self._hsv_ranges):
            marker = Marker()
            marker.header.stamp = source_msg.header.stamp
            marker.header.frame_id = self._base_frame
            marker.ns = 'color_cubes'
            marker.id = marker_id
            if color not in detections:
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
                continue
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = detections[color].pose
            (marker.scale.x, marker.scale.y,
             marker.scale.z) = self._object_size.tolist()
            b, g, r = DISPLAY_COLORS[color]
            marker.color.r = r / 255.0
            marker.color.g = g / 255.0
            marker.color.b = b / 255.0
            marker.color.a = 0.95
            marker.lifetime = Duration(nanoseconds=250_000_000).to_msg()
            marker_array.markers.append(marker)
        self._marker_publisher.publish(marker_array)

    def _log_detections(self, detections) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < 1_000_000_000:
            return
        self._last_log_ns = now_ns
        if not detections:
            self.get_logger().info('No colored cubes detected')
            return
        summary = '; '.join(
            f'{color}=({point.point.x:.3f}, {point.point.y:.3f}, '
            f'{point.point.z:.3f})'
            for color, point in detections.items()
        )
        self.get_logger().info(f'Cube centers in {self._base_frame}: {summary}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ColorCubeDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
