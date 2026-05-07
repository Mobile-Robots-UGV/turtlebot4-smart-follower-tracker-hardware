#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class RecoveryFollowerNode(Node):
    """
    Hardware follower with short prediction-based recovery.

    measured:
      normal visual following

    predicted:
      cautious predicted tracking with LiDAR safety guard

    lost:
      stop
    """

    def __init__(self):
        super().__init__('recovery_follower_node')

        self.declare_parameter('tracked_pose_topic', '/robot_09/tracked_board_pose')
        self.declare_parameter('status_topic', '/robot_09/tracker_status')
        self.declare_parameter('cmd_vel_topic', '/robot_09/cmd_vel')

        self.declare_parameter('desired_distance_m', 0.75)
        self.declare_parameter('min_distance_m', 0.45)

        self.declare_parameter('kp_linear', 0.35)
        self.declare_parameter('kp_angular', 0.90)

        self.declare_parameter('max_linear_measured', 0.15)
        self.declare_parameter('max_angular_measured', 0.45)

        self.declare_parameter('max_linear_predicted', 0.06)
        self.declare_parameter('max_angular_predicted', 0.20)

        self.declare_parameter('pose_timeout_s', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.declare_parameter('scan_topic', '/robot_09/scan')
        self.declare_parameter('front_stop_distance_m', 0.45)
        self.declare_parameter('front_slow_distance_m', 0.80)

        self.desired_distance_m = float(self.get_parameter('desired_distance_m').value)
        self.min_distance_m = float(self.get_parameter('min_distance_m').value)

        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)

        self.max_linear_measured = float(self.get_parameter('max_linear_measured').value)
        self.max_angular_measured = float(self.get_parameter('max_angular_measured').value)

        self.max_linear_predicted = float(self.get_parameter('max_linear_predicted').value)
        self.max_angular_predicted = float(self.get_parameter('max_angular_predicted').value)

        self.pose_timeout_s = float(self.get_parameter('pose_timeout_s').value)

        self.front_stop_distance_m = float(
            self.get_parameter('front_stop_distance_m').value
        )
        self.front_slow_distance_m = float(
            self.get_parameter('front_slow_distance_m').value
        )

        tracked_pose_topic = self.get_parameter('tracked_pose_topic').value
        status_topic = self.get_parameter('status_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.latest_pose = None
        self.latest_pose_time = None
        self.status = 'lost'
        self.front_min_range = float('inf')

        self.sub_pose = self.create_subscription(
            PoseStamped,
            tracked_pose_topic,
            self.pose_callback,
            10
        )

        self.sub_status = self.create_subscription(
            String,
            status_topic,
            self.status_callback,
            10
        )

        self.sub_scan = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_cmd = self.create_publisher(TwistStamped, cmd_vel_topic, cmd_qos)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info('Recovery follower node started')
        self.get_logger().info(f'Subscribing tracked pose: {tracked_pose_topic}')
        self.get_logger().info(f'Subscribing status: {status_topic}')
        self.get_logger().info(f'Subscribing scan: {scan_topic}')
        self.get_logger().info(f'Publishing cmd_vel: {cmd_vel_topic}')

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg
        self.latest_pose_time = self.get_clock().now()

    def status_callback(self, msg: String):
        self.status = msg.data

    def scan_callback(self, msg: LaserScan):
        """
        Estimate nearest obstacle in front of the robot.

        Uses approximately +/- 25 degrees in front of base_link.
        """
        front_angle_rad = math.radians(25.0)

        ranges = []
        angle = msg.angle_min

        for r in msg.ranges:
            if -front_angle_rad <= angle <= front_angle_rad:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    ranges.append(r)
            angle += msg.angle_increment

        if len(ranges) == 0:
            self.front_min_range = float('inf')
        else:
            self.front_min_range = min(ranges)

    def timer_callback(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'robot_09/base_link'

        if self.latest_pose is None or self.latest_pose_time is None:
            self.pub_cmd.publish(cmd)
            return

        now = self.get_clock().now()
        age_s = (now - self.latest_pose_time).nanoseconds * 1e-9

        if age_s > self.pose_timeout_s:
            self.pub_cmd.publish(cmd)
            return

        if self.status == 'lost':
            self.pub_cmd.publish(cmd)
            return

        x = float(self.latest_pose.pose.position.x)
        z = float(self.latest_pose.pose.position.z)

        distance_error = z - self.desired_distance_m
        linear = self.kp_linear * distance_error

        # If the board is too close, only allow slow reverse motion.
        # Do not allow forward motion when inside min_distance_m.
        if z < self.min_distance_m:
            linear = min(linear, 0.0)

        # Camera frame: +x means board appears to robot's right.
        # If the robot turns the wrong way, flip this sign.
        angular = -self.kp_angular * x

        if self.status == 'measured':
            linear = self.clamp(
                linear,
                -self.max_linear_measured,
                self.max_linear_measured
            )
            angular = self.clamp(
                angular,
                -self.max_angular_measured,
                self.max_angular_measured
            )

        elif self.status == 'predicted':
            # In predicted mode, do not blindly drive through obstacles.
            # Allow cautious rotation for reacquisition, but forward motion is heavily limited.
            linear = self.clamp(
                linear,
                0.0,
                self.max_linear_predicted
            )
            angular = self.clamp(
                angular,
                -self.max_angular_predicted,
                self.max_angular_predicted
            )

            # Extra conservative behavior during prediction.
            # If anything is near the front, stop forward motion.
            if self.front_min_range < self.front_slow_distance_m:
                linear = 0.0

        else:
            linear = 0.0
            angular = 0.0

        # LiDAR safety guard for all forward motion.
        # This does not plan around obstacles yet. It prevents forward collision.
        if linear > 0.0:
            if self.front_min_range < self.front_stop_distance_m:
                linear = 0.0
                angular = 0.0
            elif self.front_min_range < self.front_slow_distance_m:
                linear = min(linear, 0.03)

        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular

        self.get_logger().info(
            f"status={self.status} "
            f"x={x:.3f} z={z:.3f} "
            f"front={self.front_min_range:.3f} "
            f"linear={linear:.3f} angular={angular:.3f}",
            throttle_duration_sec=0.5
        )

        self.pub_cmd.publish(cmd)

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(value, high))


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
