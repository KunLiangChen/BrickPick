#!/usr/bin/env python3
"""Strips Z, roll, and pitch from odometry and republishes as a 2D transform.

Projects 6-DoF odometry down to a 2D plane by extracting only the yaw
component from the orientation quaternion, then broadcasts an odom_2d →
base_link transform using the original timestamp.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class Odom2DFilter(Node):
    def __init__(self):
        super().__init__('odom_2d_filter')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('Odom 2D Filter Started. Stripping Z, Roll, Pitch.')

    def euler_from_quaternion(self, x, y, z, w):
        """Extract yaw from a quaternion (intrinsic ZYX convention)."""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def quaternion_from_euler(self, yaw):
        """Build a quaternion representing a pure yaw rotation."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return 0.0, 0.0, sy, cy

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = 0.0

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        yaw = self.euler_from_quaternion(qx, qy, qz, qw)
        new_qx, new_qy, new_qz, new_qw = self.quaternion_from_euler(yaw)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom_2d'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        t.transform.rotation.x = new_qx
        t.transform.rotation.y = new_qy
        t.transform.rotation.z = new_qz
        t.transform.rotation.w = new_qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Odom2DFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
