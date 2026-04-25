#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray


class RegionGenerator(Node):

    def __init__(self):
        super().__init__('region_generator')

        # ===== 参数 =====
        self.radius = 0.8          # 子区域半径（米）
        self.cost_threshold = 50   # costmap阈值

        # ===== 状态 =====
        self.centers = []
        self.initialized = False

        # ===== Sub =====
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.map_callback,
            10
        )

        # ===== Pub =====
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/region_centers',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/region_markers',
            10
        )

        # 定时发布（防止RViz不刷新）
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Region Generator Started")

    # =========================
    # 只初始化一次
    # =========================
    def map_callback(self, msg):

        if self.initialized:
            return

        self.get_logger().info("Generating fixed region centers...")

        self.centers = self.generate_grid_centers(msg)

        self.initialized = True

        self.get_logger().info(f"Generated {len(self.centers)} centers")

    # =========================
    # 网格划分（稳定可靠版）
    # =========================
    def generate_grid_centers(self, map_msg):

        w = map_msg.info.width
        h = map_msg.info.height
        res = map_msg.info.resolution

        data = np.array(map_msg.data).reshape((h, w))

        free = data < self.cost_threshold

        # stride = 半径（推荐）
        stride = int(self.radius / res)

        centers = []

        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        for y in range(0, h, stride):
            for x in range(0, w, stride):

                if not free[y, x]:
                    continue

                wx = origin_x + x * res
                wy = origin_y + y * res

                centers.append((wx, wy))

        return centers

    # =========================
    # 定时发布
    # =========================
    def timer_callback(self):

        if not self.initialized:
            return

        self.publish_centers()
        self.publish_markers()

    # =========================
    # 发布 PoseArray
    # =========================
    def publish_centers(self):

        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in self.centers:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.orientation.w = 1.0
            msg.poses.append(p)

        self.pose_pub.publish(msg)

    # =========================
    # 可视化（正方形区域）
    # =========================
    def publish_markers(self):

        marker_array = MarkerArray()
        size = self.radius * 2

        for i, (x, y) in enumerate(self.centers):

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()

            m.ns = "regions"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.orientation.w = 1.0

            m.scale.x = size
            m.scale.y = size
            m.scale.z = 0.01

            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.3

            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = RegionGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()