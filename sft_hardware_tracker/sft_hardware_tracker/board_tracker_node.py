#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String


class KFBackend:
    """
    Linear constant-velocity Kalman filter.

    Hardware version:
      state = [x, z, vx, vz]

    x = board lateral position in camera frame
    z = board forward distance in camera frame
    """

    def __init__(self, process_noise: float, measurement_noise: float) -> None:
        self._H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )

        self._R = measurement_noise * np.eye(2)
        self._qn = process_noise

        self._x = np.zeros(4, dtype=float)
        self._P = np.eye(4, dtype=float) * 10.0

    def initialize(self, x: float, z: float) -> None:
        self._x = np.array([x, z, 0.0, 0.0], dtype=float)
        self._P = np.eye(4, dtype=float) * 10.0

    def _make_FQ(self, dt: float):
        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

        dt2 = dt ** 2
        dt3 = dt ** 3
        dt4 = dt ** 4

        q1 = dt4 / 4.0
        q2 = dt3 / 2.0
        q3 = dt2

        Q = np.array(
            [
                [q1, 0.0, q2, 0.0],
                [0.0, q1, 0.0, q2],
                [q2, 0.0, q3, 0.0],
                [0.0, q2, 0.0, q3],
            ],
            dtype=float,
        )

        return F, self._qn * Q

    def update(self, meas_x: float, meas_z: float, dt: float) -> None:
        dt = float(np.clip(dt, 0.0, 5.0))

        if dt > 1e-6:
            F, Q = self._make_FQ(dt)
            self._x = F @ self._x
            self._P = F @ self._P @ F.T + Q

        z_meas = np.array([meas_x, meas_z], dtype=float)

        S = self._H @ self._P @ self._H.T + self._R
        K = self._P @ self._H.T @ np.linalg.inv(S)

        innovation = z_meas - self._H @ self._x
        self._x = self._x + K @ innovation
        self._P = (np.eye(4) - K @ self._H) @ self._P

    def peek(self, dt: float) -> tuple[float, float, float, float, np.ndarray]:
        """
        Read-only prediction.

        Returns:
          x, z, vx, vz, P_xz
        """
        dt = float(np.clip(dt, 0.0, 5.0))

        if dt > 1e-6:
            F, Q = self._make_FQ(dt)
            x_pred = F @ self._x
            P_pred = F @ self._P @ F.T + Q
        else:
            x_pred = self._x.copy()
            P_pred = self._P.copy()

        return (
            float(x_pred[0]),
            float(x_pred[1]),
            float(x_pred[2]),
            float(x_pred[3]),
            P_pred[:2, :2],
        )

    def rollout(self, dt: float, n_steps: int) -> list[tuple[float, float]]:
        x_tmp = self._x.copy()
        F, _ = self._make_FQ(dt)

        result = []
        for _ in range(n_steps):
            x_tmp = F @ x_tmp
            result.append((float(x_tmp[0]), float(x_tmp[1])))

        return result


class PFBackend:
    """
    Bootstrap particle filter.

    Hardware version:
      particle state = [x, z, vx, vz]

    x = board lateral position in camera frame
    z = board forward distance in camera frame
    """

    def __init__(
        self,
        process_noise_scale: float,
        measurement_noise_scale: float,
        n_particles: int,
    ) -> None:
        self.N = int(n_particles)
        self._pns = float(process_noise_scale)
        self._mns = float(measurement_noise_scale)

        self._particles = np.zeros((self.N, 4), dtype=float)
        self._weights = np.ones(self.N, dtype=float) / self.N

        self._sig_pos = 0.08
        self._sig_vel = 0.20
        self._sig_meas = 0.06

    def initialize(self, x: float, z: float) -> None:
        self._particles[:, 0] = x + np.random.randn(self.N) * 0.10
        self._particles[:, 1] = z + np.random.randn(self.N) * 0.10
        self._particles[:, 2] = 0.0
        self._particles[:, 3] = 0.0
        self._weights[:] = 1.0 / self.N

    def _motion_update(self, dt: float) -> None:
        dt = float(np.clip(dt, 0.0, 5.0))
        if dt <= 1e-6:
            return

        sdt = math.sqrt(dt)

        self._particles[:, 0] += (
            self._particles[:, 2] * dt
            + np.random.randn(self.N) * self._sig_pos * sdt * self._pns
        )
        self._particles[:, 1] += (
            self._particles[:, 3] * dt
            + np.random.randn(self.N) * self._sig_pos * sdt * self._pns
        )
        self._particles[:, 2] += (
            np.random.randn(self.N) * self._sig_vel * sdt * self._pns
        )
        self._particles[:, 3] += (
            np.random.randn(self.N) * self._sig_vel * sdt * self._pns
        )

        # Hardware safety clamp.
        self._particles[:, 0] = np.clip(self._particles[:, 0], -1.50, 1.50)
        self._particles[:, 1] = np.clip(self._particles[:, 1], 0.20, 3.00)

    def update(self, meas_x: float, meas_z: float, dt: float) -> None:
        self._motion_update(dt)

        sigma = self._sig_meas * self._mns

        dx = self._particles[:, 0] - meas_x
        dz = self._particles[:, 1] - meas_z

        log_w = -0.5 * (dx * dx + dz * dz) / (sigma * sigma)
        log_w -= np.max(log_w)

        w = np.exp(log_w)
        total = np.sum(w)

        if total < 1e-300:
            self.initialize(meas_x, meas_z)
            return

        self._weights = w / total

        n_eff = 1.0 / np.sum(self._weights ** 2)
        if n_eff < self.N / 2.0:
            self._systematic_resample()

    def _systematic_resample(self) -> None:
        cumulative = np.cumsum(self._weights)
        start = np.random.uniform(0.0, 1.0 / self.N)
        positions = start + np.arange(self.N) / self.N

        indexes = np.zeros(self.N, dtype=int)

        i = 0
        j = 0
        while i < self.N:
            if positions[i] < cumulative[j]:
                indexes[i] = j
                i += 1
            else:
                j = min(j + 1, self.N - 1)

        self._particles = self._particles[indexes]
        self._weights[:] = 1.0 / self.N

    def _mean(self) -> tuple[float, float, float, float]:
        w = self._weights

        x = float(np.dot(w, self._particles[:, 0]))
        z = float(np.dot(w, self._particles[:, 1]))
        vx = float(np.dot(w, self._particles[:, 2]))
        vz = float(np.dot(w, self._particles[:, 3]))

        return x, z, vx, vz

    def peek(self, dt: float) -> tuple[float, float, float, float, np.ndarray]:
        dt = float(np.clip(dt, 0.0, 5.0))

        x, z, vx, vz = self._mean()

        x_pred = x + vx * dt
        z_pred = z + vz * dt

        x_pred = float(np.clip(x_pred, -1.50, 1.50))
        z_pred = float(np.clip(z_pred, 0.20, 3.00))

        mean_pos = np.array([x, z], dtype=float)
        diff = self._particles[:, 0:2] - mean_pos
        P_xz = (self._weights[:, None] * diff).T @ diff

        return x_pred, z_pred, vx, vz, P_xz

    def rollout(self, dt: float, n_steps: int) -> list[tuple[float, float]]:
        x, z, vx, vz = self._mean()

        result = []
        for _ in range(n_steps):
            x += vx * dt
            z += vz * dt

            x = float(np.clip(x, -1.50, 1.50))
            z = float(np.clip(z, 0.20, 3.00))

            result.append((x, z))

        return result

    def get_particles(self) -> np.ndarray:
        return self._particles.copy()

    def get_weights(self) -> np.ndarray:
        return self._weights.copy()


class BoardTrackerNode(Node):
    """
    Hardware board tracker with selectable KF/PF backend.

    Input:
      /robot_09/board_pose
      /robot_09/board_visible

    Output:
      /robot_09/tracked_board_pose
      /robot_09/tracker_status
      /robot_09/predicted_board_path

    Status:
      measured  = fresh board pose is being received
      predicted = board disappeared recently, estimator predicts pose
      lost      = board has been missing too long
    """

    def __init__(self):
        super().__init__('board_tracker_node')

        self.declare_parameter('board_pose_topic', '/robot_09/board_pose')
        self.declare_parameter('board_visible_topic', '/robot_09/board_visible')
        self.declare_parameter('tracked_pose_topic', '/robot_09/tracked_board_pose')
        self.declare_parameter('status_topic', '/robot_09/tracker_status')
        self.declare_parameter('predicted_path_topic', '/robot_09/predicted_board_path')

        self.declare_parameter('tracker_backend', 'kf')  # kf or pf
        self.declare_parameter('pf_num_particles', 200)
        self.declare_parameter('process_noise', 0.5)
        self.declare_parameter('measurement_noise', 0.1)

        self.declare_parameter('fresh_threshold_s', 0.5)
        self.declare_parameter('prediction_timeout_s', 3.0)
        self.declare_parameter('prediction_horizon_s', 1.5)
        self.declare_parameter('prediction_dt_s', 0.1)
        self.declare_parameter('publish_rate_hz', 20.0)

        board_pose_topic = self.get_parameter('board_pose_topic').value
        board_visible_topic = self.get_parameter('board_visible_topic').value
        tracked_pose_topic = self.get_parameter('tracked_pose_topic').value
        status_topic = self.get_parameter('status_topic').value
        predicted_path_topic = self.get_parameter('predicted_path_topic').value

        self.tracker_backend = str(
            self.get_parameter('tracker_backend').value
        ).strip().lower()

        pf_num_particles = int(self.get_parameter('pf_num_particles').value)
        process_noise = float(self.get_parameter('process_noise').value)
        measurement_noise = float(self.get_parameter('measurement_noise').value)

        self.fresh_threshold_s = float(
            self.get_parameter('fresh_threshold_s').value
        )
        self.prediction_timeout_s = float(
            self.get_parameter('prediction_timeout_s').value
        )
        self.prediction_horizon_s = float(
            self.get_parameter('prediction_horizon_s').value
        )
        self.prediction_dt_s = float(
            self.get_parameter('prediction_dt_s').value
        )

        self.kf = KFBackend(process_noise, measurement_noise)

        if self.tracker_backend == 'pf':
            self.backend = PFBackend(
                process_noise_scale=process_noise,
                measurement_noise_scale=measurement_noise,
                n_particles=pf_num_particles,
            )
        else:
            self.tracker_backend = 'kf'
            self.backend = self.kf

        self.initialized = False
        self.last_update_time = None
        self.last_meas_time = None
        self.last_pose: Optional[PoseStamped] = None
        self.board_visible = False

        self.sub_pose = self.create_subscription(
            PoseStamped,
            board_pose_topic,
            self.pose_callback,
            10,
        )

        self.sub_visible = self.create_subscription(
            Bool,
            board_visible_topic,
            self.visible_callback,
            10,
        )

        self.pub_pose = self.create_publisher(
            PoseStamped,
            tracked_pose_topic,
            10,
        )
        self.pub_status = self.create_publisher(
            String,
            status_topic,
            10,
        )
        self.pub_path = self.create_publisher(
            Path,
            predicted_path_topic,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info('Board tracker node started')
        self.get_logger().info(f'Backend: {self.tracker_backend}')
        self.get_logger().info(f'Subscribing pose: {board_pose_topic}')
        self.get_logger().info(f'Subscribing visible: {board_visible_topic}')
        self.get_logger().info(f'Publishing tracked pose: {tracked_pose_topic}')
        self.get_logger().info(f'Publishing status: {status_topic}')
        self.get_logger().info(f'Publishing predicted path: {predicted_path_topic}')

    def visible_callback(self, msg: Bool):
        self.board_visible = bool(msg.data)

    def pose_callback(self, msg: PoseStamped):
        meas_x = float(msg.pose.position.x)
        meas_z = float(msg.pose.position.z)

        if not math.isfinite(meas_x) or not math.isfinite(meas_z):
            return

        now = self.get_clock().now()

        if not self.initialized:
            self.backend.initialize(meas_x, meas_z)

            # Keep KF warm even when PF is selected, useful if you later add comparison.
            if self.tracker_backend == 'pf':
                self.kf.initialize(meas_x, meas_z)

            self.initialized = True
            self.last_update_time = now
            self.last_meas_time = now
            self.last_pose = msg

            self.get_logger().info(
                f'Tracker initialized with {self.tracker_backend}: '
                f'x={meas_x:.3f}, z={meas_z:.3f}'
            )
            return

        dt = (now - self.last_update_time).nanoseconds * 1e-9
        dt = float(np.clip(dt, 0.0, 5.0))

        self.backend.update(meas_x, meas_z, dt)

        if self.tracker_backend == 'pf':
            self.kf.update(meas_x, meas_z, dt)

        self.last_update_time = now
        self.last_meas_time = now
        self.last_pose = msg

    def timer_callback(self):
        if not self.initialized or self.last_meas_time is None or self.last_pose is None:
            status_msg = String()
            status_msg.data = 'lost'
            self.pub_status.publish(status_msg)
            return

        now = self.get_clock().now()
        now_msg = now.to_msg()

        age_s = (now - self.last_meas_time).nanoseconds * 1e-9

        if self.board_visible and age_s <= self.fresh_threshold_s:
            status = 'measured'
        elif age_s <= self.prediction_timeout_s:
            status = 'predicted'
        else:
            status = 'lost'

        status_msg = String()
        status_msg.data = status
        self.pub_status.publish(status_msg)

        if status == 'lost':
            self.get_logger().info(
                'status=lost',
                throttle_duration_sec=0.5,
            )
            return

        dt_now = (now - self.last_update_time).nanoseconds * 1e-9
        dt_now = float(np.clip(dt_now, 0.0, 5.0))

        est_x, est_z, est_vx, est_vz, _ = self.backend.peek(dt_now)

        tracked = PoseStamped()
        tracked.header.stamp = now_msg
        tracked.header.frame_id = self.last_pose.header.frame_id
        tracked.pose.position.x = est_x
        tracked.pose.position.y = self.last_pose.pose.position.y
        tracked.pose.position.z = est_z
        tracked.pose.orientation = self.last_pose.pose.orientation

        self.pub_pose.publish(tracked)

        self.publish_predicted_path(now_msg, tracked.header.frame_id)

        self.get_logger().info(
            f"backend={self.tracker_backend} "
            f"status={status} "
            f"x={est_x:.3f} z={est_z:.3f} "
            f"vx={est_vx:.3f} vz={est_vz:.3f}",
            throttle_duration_sec=0.5,
        )

    def publish_predicted_path(self, stamp, frame_id: str):
        n_steps = int(self.prediction_horizon_s / self.prediction_dt_s) + 1
        waypoints = self.backend.rollout(self.prediction_dt_s, n_steps)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = frame_id

        for x, z in waypoints:
            wp = PoseStamped()
            wp.header = path.header
            wp.pose.position.x = float(x)
            wp.pose.position.y = 0.0
            wp.pose.position.z = float(z)
            wp.pose.orientation.w = 1.0
            path.poses.append(wp)

        self.pub_path.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = BoardTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()