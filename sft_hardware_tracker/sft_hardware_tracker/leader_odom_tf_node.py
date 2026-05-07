#!/usr/bin/env python3
"""Publish a leader robot TF from bridged Gazebo odometry."""

from geometry_msgs.msg import TransformStamped
import math
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class LeaderOdomTfNode(Node):
    def __init__(self):
        super().__init__('leader_odom_tf_node')

        self.declare_parameter('odom_topic', '/leader/odom')
        self.declare_parameter('parent_frame', 'robot_09/odom')
        self.declare_parameter('child_frame', 'leader/base_link')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.spawn_x = float(self.get_parameter('spawn_x').value)
        self.spawn_y = float(self.get_parameter('spawn_y').value)
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        self.spawn_yaw = float(self.get_parameter('spawn_yaw').value)
        self.spawn_cos = math.cos(self.spawn_yaw)
        self.spawn_sin = math.sin(self.spawn_yaw)
        odom_topic = self.get_parameter('odom_topic').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            f'Leader TF: {odom_topic} -> {self.parent_frame} to {self.child_frame} '
            f'with spawn offset ({self.spawn_x:.2f}, {self.spawn_y:.2f}, '
            f'{self.spawn_z:.2f}, yaw={self.spawn_yaw:.2f})'
        )

    def odom_callback(self, msg: Odometry):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame

        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        transform.transform.translation.x = (
            self.spawn_x + self.spawn_cos * odom_x - self.spawn_sin * odom_y
        )
        transform.transform.translation.y = (
            self.spawn_y + self.spawn_sin * odom_x + self.spawn_cos * odom_y
        )
        transform.transform.translation.z = self.spawn_z + msg.pose.pose.position.z
        transform.transform.rotation = self.compose_yaw_with_quaternion(
            self.spawn_yaw,
            msg.pose.pose.orientation,
        )

        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def compose_yaw_with_quaternion(yaw, quaternion):
        half_yaw = 0.5 * yaw
        yaw_z = math.sin(half_yaw)
        yaw_w = math.cos(half_yaw)

        result = type(quaternion)()
        result.x = -yaw_z * quaternion.y + yaw_w * quaternion.x
        result.y = yaw_z * quaternion.x + yaw_w * quaternion.y
        result.z = yaw_z * quaternion.w + yaw_w * quaternion.z
        result.w = -yaw_z * quaternion.z + yaw_w * quaternion.w

        norm = math.sqrt(
            result.x * result.x
            + result.y * result.y
            + result.z * result.z
            + result.w * result.w
        )
        if norm > 0.0:
            result.x /= norm
            result.y /= norm
            result.z /= norm
            result.w /= norm

        return result


def main(args=None):
    rclpy.init(args=args)
    node = LeaderOdomTfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
