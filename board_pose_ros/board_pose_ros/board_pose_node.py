#!/usr/bin/env python3
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3Stamped, TransformStamped, Point
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Int32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


DICT_MAP = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
}


def rvec_to_quaternion(rvec: np.ndarray) -> np.ndarray:
    rot_mtx, _ = cv2.Rodrigues(rvec)
    trace = np.trace(rot_mtx)

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot_mtx[2, 1] - rot_mtx[1, 2]) / s
        qy = (rot_mtx[0, 2] - rot_mtx[2, 0]) / s
        qz = (rot_mtx[1, 0] - rot_mtx[0, 1]) / s
    elif rot_mtx[0, 0] > rot_mtx[1, 1] and rot_mtx[0, 0] > rot_mtx[2, 2]:
        s = math.sqrt(1.0 + rot_mtx[0, 0] - rot_mtx[1, 1] - rot_mtx[2, 2]) * 2.0
        qw = (rot_mtx[2, 1] - rot_mtx[1, 2]) / s
        qx = 0.25 * s
        qy = (rot_mtx[0, 1] + rot_mtx[1, 0]) / s
        qz = (rot_mtx[0, 2] + rot_mtx[2, 0]) / s
    elif rot_mtx[1, 1] > rot_mtx[2, 2]:
        s = math.sqrt(1.0 + rot_mtx[1, 1] - rot_mtx[0, 0] - rot_mtx[2, 2]) * 2.0
        qw = (rot_mtx[0, 2] - rot_mtx[2, 0]) / s
        qx = (rot_mtx[0, 1] + rot_mtx[1, 0]) / s
        qy = 0.25 * s
        qz = (rot_mtx[1, 2] + rot_mtx[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot_mtx[2, 2] - rot_mtx[0, 0] - rot_mtx[1, 1]) * 2.0
        qw = (rot_mtx[1, 0] - rot_mtx[0, 1]) / s
        qx = (rot_mtx[0, 2] + rot_mtx[2, 0]) / s
        qy = (rot_mtx[1, 2] + rot_mtx[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    return q


def rotation_matrix_to_rpy(rot_mtx: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(rot_mtx[0, 0] ** 2 + rot_mtx[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rot_mtx[2, 1], rot_mtx[2, 2])
        pitch = math.atan2(-rot_mtx[2, 0], sy)
        yaw = math.atan2(rot_mtx[1, 0], rot_mtx[0, 0])
    else:
        roll = math.atan2(-rot_mtx[1, 2], rot_mtx[1, 1])
        pitch = math.atan2(-rot_mtx[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


class BoardPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("board_pose_node")

        pkg_share = Path(get_package_share_directory("board_pose_ros"))

        self.declare_parameter("image_topic", "/robot_09/oakd/rgb/image_raw/compressed")
        self.declare_parameter("image_type", "compressed")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("use_camera_info", False)
        self.declare_parameter("camera_frame", "oak_camera_frame")
        self.declare_parameter("board_frame", "board_frame")
        self.declare_parameter("calibration_file", str(pkg_share / "config" / "camera_calib_oak.npz"))
        self.declare_parameter("board_config_file", str(pkg_share / "config" / "board_config.json"))
        self.declare_parameter("publish_static_camera_tf", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("log_pose", True)
        self.declare_parameter("log_every_n", 10)
        self.declare_parameter("log_rpy_degrees", False)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.image_type = self.get_parameter("image_type").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.use_camera_info = self.get_parameter("use_camera_info").get_parameter_value().bool_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.board_frame = self.get_parameter("board_frame").get_parameter_value().string_value
        self.calibration_file = self.get_parameter("calibration_file").get_parameter_value().string_value
        self.board_config_file = self.get_parameter("board_config_file").get_parameter_value().string_value
        self.publish_static_camera_tf = (
            self.get_parameter("publish_static_camera_tf").get_parameter_value().bool_value
        )
        self.publish_debug_image = self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        self.process_every_n = max(
            1,
            self.get_parameter("process_every_n").get_parameter_value().integer_value,
        )
        self.log_pose = self.get_parameter("log_pose").get_parameter_value().bool_value
        self.log_every_n = self.get_parameter("log_every_n").get_parameter_value().integer_value
        self.log_rpy_degrees = self.get_parameter("log_rpy_degrees").get_parameter_value().bool_value

        calib = np.load(self.calibration_file)
        self.camera_matrix = calib["camera_matrix"]
        self.dist_coeffs = calib["dist_coeffs"]

        with open(self.board_config_file, "r", encoding="utf-8") as f:
            self.board_cfg = json.load(f)

        dict_name = self.board_cfg["dictionary"]
        self.marker_size_m = float(self.board_cfg["marker_size_m"])
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_MAP[dict_name])

        # RC2 FIX: OpenCV 4.6 (Jazzy apt default) does not have ArucoDetector.
        # DetectorParameters() also segfaults when passed to the legacy detectMarkers()
        # in 4.6 — DetectorParameters_create() returns a properly initialised Ptr<>.
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector_params = cv2.aruco.DetectorParameters()
            self._new_aruco_api = True
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()
            self._new_aruco_api = False

        self.board_object_points = self._build_board_object_points()
        if self._new_aruco_api:
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        else:
            self.aruco_detector = None

        # ── Static TF: map → oak_camera_frame, used by the original hardware demo.
        # Simulation disables this because robot_state_publisher already provides the
        # camera frame in the live robot TF tree.
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        if self.publish_static_camera_tf:
            static_msg = TransformStamped()
            static_msg.header.stamp = self.get_clock().now().to_msg()
            static_msg.header.frame_id = 'map'
            static_msg.child_frame_id = self.camera_frame
            static_msg.transform.rotation.w = 1.0
            self.static_tf_broadcaster.sendTransform(static_msg)

        # ── Dynamic TF broadcaster (board_frame when detected) ───────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Original publishers ──────────────────────────────────────────
        self.pose_pub    = self.create_publisher(PoseStamped,      "/robot_09/board_pose",       10)
        self.rpy_pub     = self.create_publisher(Vector3Stamped,   "/robot_09/board_rpy",        10)
        self.visible_pub = self.create_publisher(Bool,             "/robot_09/board_visible",    10)
        self.ids_pub     = self.create_publisher(Int32MultiArray,  "/robot_09/board_used_ids",   10)
        self.ids_pub2    = self.create_publisher(Int32MultiArray,  "/board_used_ids",            10)

        # ── New publishers ───────────────────────────────────────────────
        self.debug_image_pub = self.create_publisher(
            Image, "/robot_09/board_debug_image", 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, "/robot_09/board_markers", 10)

        if self.use_camera_info and self.camera_info_topic:
            self.info_sub = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                qos_profile_sensor_data,
            )
        else:
            self.info_sub = None

        # ── Subscriber ───────────────────────────────────────────────────
        if self.image_type.strip().lower() == "raw":
            self.sub = self.create_subscription(
                Image,
                self.image_topic,
                self.raw_image_callback,
                qos_profile_sensor_data,
            )
        else:
            self.image_type = "compressed"
            self.sub = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.compressed_image_callback,
                qos_profile_sensor_data,
            )

        self.frame_count = 0
        self.get_logger().info(f"Subscribed to {self.image_topic} ({self.image_type})")
        if self.info_sub is not None:
            self.get_logger().info(f"Using camera info: {self.camera_info_topic}")
        self.get_logger().info(f"Calibration file: {self.calibration_file}")
        self.get_logger().info(f"Board config file: {self.board_config_file}")
        if self.publish_static_camera_tf:
            self.get_logger().info(f"Static TF published: map → {self.camera_frame}")
        if not self.publish_debug_image:
            self.get_logger().info("Debug image publishing disabled")

    # ──────────────────────────────────────────────────────────────────────
    #  RViz2 marker helpers
    # ──────────────────────────────────────────────────────────────────────
    def _make_sphere(self, mid, x, y, z, r, g, b, a=1.0, size=0.06):
        m = Marker()
        m.header.frame_id    = self.camera_frame
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'board_pose'
        m.id                 = mid
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = float(z)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = size
        m.color              = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        m.lifetime           = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m

    def _make_text(self, mid, x, y, z, text, r, g, b):
        m = Marker()
        m.header.frame_id    = self.camera_frame
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = 'board_text'
        m.id                 = mid
        m.type               = Marker.TEXT_VIEW_FACING
        m.action             = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = float(z)
        m.pose.orientation.w = 1.0
        m.scale.z            = 0.05
        m.color              = ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
        m.text               = text
        m.lifetime           = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m

    def _make_line(self, mid, x, y, z, r, g, b):
        """Line from camera origin to board center."""
        m = Marker()
        m.header.frame_id = self.camera_frame
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns              = 'board_line'
        m.id              = mid
        m.type            = Marker.LINE_STRIP
        m.action          = Marker.ADD
        p0 = Point(); p0.x = 0.0; p0.y = 0.0; p0.z = 0.0
        p1 = Point(); p1.x = float(x); p1.y = float(y); p1.z = float(z)
        m.points          = [p0, p1]
        m.scale.x         = 0.01
        m.color           = ColorRGBA(r=float(r), g=float(g), b=float(b), a=0.5)
        m.lifetime        = rclpy.duration.Duration(seconds=0.3).to_msg()
        return m

    # ──────────────────────────────────────────────────────────────────────
    #  Board object points
    # ──────────────────────────────────────────────────────────────────────
    def _build_board_object_points(self) -> dict[int, np.ndarray]:
        result = {}
        size = self.marker_size_m

        for marker_id_str, marker_info in self.board_cfg["markers"].items():
            marker_id = int(marker_id_str)
            x_tl, y_tl = marker_info["top_left_xy_m"]
            rotation_deg = float(marker_info.get("rotation_deg", 0.0))

            corners = np.array(
                [
                    [x_tl, y_tl, 0.0],
                    [x_tl + size, y_tl, 0.0],
                    [x_tl + size, y_tl - size, 0.0],
                    [x_tl, y_tl - size, 0.0],
                ],
                dtype=np.float32,
            )

            if abs(rotation_deg) > 1e-6:
                center = np.array([x_tl + size / 2.0, y_tl - size / 2.0, 0.0], dtype=np.float32)
                theta = math.radians(rotation_deg)
                rot = np.array(
                    [
                        [math.cos(theta), -math.sin(theta), 0.0],
                        [math.sin(theta), math.cos(theta), 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    dtype=np.float32,
                )
                corners = ((corners - center) @ rot.T) + center

            result[marker_id] = corners

        return result

    # ──────────────────────────────────────────────────────────────────────
    #  Main image callbacks
    # ──────────────────────────────────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) == 9 and abs(msg.k[0]) > 1e-9:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)

        if msg.d:
            self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        else:
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def compressed_image_callback(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode compressed image")
            return
        self.process_frame(frame, msg.header)

    def raw_image_callback(self, msg: Image) -> None:
        try:
            frame = self.image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode raw image: {exc}")
            return
        self.process_frame(frame, msg.header)

    @staticmethod
    def image_msg_to_bgr(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "bgr8": 3,
            "rgb8": 3,
            "8uc3": 3,
            "bgra8": 4,
            "rgba8": 4,
            "8uc4": 4,
            "mono8": 1,
            "8uc1": 1,
        }
        if encoding not in channels_by_encoding:
            raise ValueError(f"unsupported encoding '{msg.encoding}'")

        channels = channels_by_encoding[encoding]
        itemsize = np.dtype(np.uint8).itemsize
        expected_step = msg.width * channels * itemsize
        if msg.step < expected_step:
            raise ValueError(
                f"invalid image step {msg.step}; expected at least {expected_step}"
            )

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        rows = arr.reshape((msg.height, msg.step))[:, :expected_step]

        if channels == 1:
            mono = rows.reshape((msg.height, msg.width))
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        frame = rows.reshape((msg.height, msg.width, channels))
        if encoding in ("rgb8", "8uc3"):
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif encoding in ("bgra8", "8uc4"):
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        return np.ascontiguousarray(frame)

    def publish_debug_frame(self, frame: np.ndarray, header) -> None:
        if not self.publish_debug_image:
            return

        frame = np.ascontiguousarray(frame)
        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height = frame.shape[0]
        debug_msg.width = frame.shape[1]
        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = False
        debug_msg.step = frame.shape[1] * 3
        debug_msg.data = frame.tobytes()
        self.debug_image_pub.publish(debug_msg)

    def process_frame(self, frame, header) -> None:
        self.frame_count += 1
        if self.frame_count % self.process_every_n != 0:
            return

        frame = np.array(frame, dtype=np.uint8)  # always writable contiguous copy
        if frame.ndim < 2 or frame.shape[0] == 0 or frame.shape[1] == 0:
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._new_aruco_api:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params)

        visible_msg = Bool()
        ids_msg     = Int32MultiArray()
        marker_array = MarkerArray()

        # ── No markers detected ──────────────────────────────────────────
        if ids is None or len(ids) == 0:
            visible_msg.data = False
            self.visible_pub.publish(visible_msg)
            self.ids_pub.publish(ids_msg)
            self.ids_pub2.publish(ids_msg)
            self.markers_pub.publish(marker_array)

            if self.publish_debug_image:
                cv2.putText(frame, 'No board detected',
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                self.publish_debug_frame(frame, header)
            return

        # ── Build correspondences ────────────────────────────────────────
        object_points = []
        image_points  = []
        used_ids      = []

        ids_flat = ids.flatten().tolist()
        for marker_corners, marker_id in zip(corners, ids_flat):
            if marker_id not in self.board_object_points:
                continue

            img_pts = marker_corners.reshape(4, 2).astype(np.float32)
            obj_pts = self.board_object_points[marker_id].astype(np.float32)

            image_points.append(img_pts)
            object_points.append(obj_pts)
            used_ids.append(marker_id)

        if len(used_ids) == 0:
            visible_msg.data = False
            self.visible_pub.publish(visible_msg)
            self.ids_pub.publish(ids_msg)
            self.ids_pub2.publish(ids_msg)
            self.markers_pub.publish(marker_array)

            if self.publish_debug_image:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                cv2.putText(frame, 'Markers detected - none match config',
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                self.publish_debug_frame(frame, header)
            return

        object_points = np.vstack(object_points)
        image_points  = np.vstack(image_points)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not success:
            visible_msg.data = False
            self.visible_pub.publish(visible_msg)
            self.ids_pub.publish(ids_msg)
            self.ids_pub2.publish(ids_msg)
            self.markers_pub.publish(marker_array)

            if self.publish_debug_image:
                cv2.putText(frame, 'solvePnP failed',
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                self.publish_debug_frame(frame, header)
            return

        # ── Pose computation ─────────────────────────────────────────────
        rot_mtx, _ = cv2.Rodrigues(rvec)
        roll, pitch, yaw = rotation_matrix_to_rpy(rot_mtx)
        quat = rvec_to_quaternion(rvec)

        x_m = float(tvec[0][0])
        y_m = float(tvec[1][0])
        z_m = float(tvec[2][0])

        header = header
        if not header.frame_id:
            header.frame_id = self.camera_frame

        # ── Publish pose ─────────────────────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header              = header
        pose_msg.header.frame_id     = self.camera_frame
        pose_msg.pose.position.x     = x_m
        pose_msg.pose.position.y     = y_m
        pose_msg.pose.position.z     = z_m
        pose_msg.pose.orientation.x  = float(quat[0])
        pose_msg.pose.orientation.y  = float(quat[1])
        pose_msg.pose.orientation.z  = float(quat[2])
        pose_msg.pose.orientation.w  = float(quat[3])
        self.pose_pub.publish(pose_msg)

        # ── Publish RPY ──────────────────────────────────────────────────
        rpy_msg = Vector3Stamped()
        rpy_msg.header   = pose_msg.header
        rpy_msg.vector.x = float(roll)
        rpy_msg.vector.y = float(pitch)
        rpy_msg.vector.z = float(yaw)
        self.rpy_pub.publish(rpy_msg)

        # ── Publish visibility and IDs ───────────────────────────────────
        visible_msg.data = True
        self.visible_pub.publish(visible_msg)

        ids_msg.data = used_ids
        self.ids_pub.publish(ids_msg)
        self.ids_pub2.publish(ids_msg)

        # ── Publish dynamic TF (camera → board) ──────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header                    = pose_msg.header
        tf_msg.child_frame_id            = self.board_frame
        tf_msg.transform.translation.x   = x_m
        tf_msg.transform.translation.y   = y_m
        tf_msg.transform.translation.z   = z_m
        tf_msg.transform.rotation        = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        # ── RViz2 markers ────────────────────────────────────────────────
        # Green sphere at board center
        marker_array.markers.append(
            self._make_sphere(0, x_m, y_m, z_m, 0.0, 1.0, 0.0, size=0.07))

        # White line from camera origin to board
        marker_array.markers.append(
            self._make_line(1, x_m, y_m, z_m, 1.0, 1.0, 1.0))

        # Text labels (white)
        marker_array.markers.append(
            self._make_text(2, x_m, y_m + 0.08, z_m,
                f'X: {x_m*100:.1f} cm', 1.0, 1.0, 1.0))
        marker_array.markers.append(
            self._make_text(3, x_m, y_m + 0.13, z_m,
                f'Y: {y_m*100:.1f} cm', 1.0, 1.0, 1.0))
        marker_array.markers.append(
            self._make_text(4, x_m, y_m + 0.18, z_m,
                f'Z: {z_m*100:.1f} cm', 1.0, 1.0, 1.0))
        marker_array.markers.append(
            self._make_text(5, x_m, y_m + 0.23, z_m,
                f'IDs: {sorted(used_ids)}', 0.0, 1.0, 0.0))

        self.markers_pub.publish(marker_array)

        # ── OpenCV debug image overlay ───────────────────────────────────
        if self.publish_debug_image:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(
                frame, self.camera_matrix, self.dist_coeffs,
                rvec, tvec, 0.05)

            # Info panel background
            overlay = frame.copy()
            cv2.rectangle(overlay, (10, 10), (300, 160), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

            cv2.putText(frame, f'IDs: {sorted(used_ids)}',
                (18, 35),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f'X:  {x_m*100:+7.1f} cm',
                (18, 63),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f'Y:  {y_m*100:+7.1f} cm',
                (18, 91),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f'Z:  {z_m*100:+7.1f} cm',
                (18, 119), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f'Yaw:{math.degrees(yaw):+7.1f} deg',
                (18, 147), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            self.publish_debug_frame(frame, header)

        # ── Logging ──────────────────────────────────────────────────────
        if self.log_pose and self.frame_count % max(1, self.log_every_n) == 0:
            rx, ry, rz = roll, pitch, yaw
            if self.log_rpy_degrees:
                rx, ry, rz = map(math.degrees, (roll, pitch, yaw))
            self.get_logger().info(
                f"visible=True ids={used_ids} "
                f"x={x_m:.4f} y={y_m:.4f} z={z_m:.4f} "
                f"rx={rx:.4f} ry={ry:.4f} rz={rz:.4f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoardPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
