#!/usr/bin/env python3
"""
brickpick: 接近物体节点
- 订阅 vision/detections
- 控制机器人旋转以对准物体中心
- 对准后缓慢前进，直到物体到达画面底部或消失
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import time

class ApproachNode(Node):
    def __init__(self):
        super().__init__('approach_node')
        
        # 1. 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('img_width', 640),
                ('img_height', 360),
                ('target_x_offset', 317.28), # 根据 camera_info 的 p[2] (cx)
                ('yaw_kp', 0.002),           # 略微调高增益以提高对准灵敏度
                ('forward_speed', 0.1),      # 前进速度 (m/s)
                ('stop_y_threshold', 340.0), # 画面高度 360，设为 340 接近底部
                ('align_threshold', 15.0),   # 缩小允许偏差，对准更精确
                ('timeout_lost', 1.0)        # 目标丢失超时停止 (秒)
            ]
        )
        
        self.img_width = self.get_parameter('img_width').value
        self.img_height = self.get_parameter('img_height').value
        self.target_x = self.get_parameter('target_x_offset').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.stop_y_threshold = self.get_parameter('stop_y_threshold').value
        self.align_threshold = self.get_parameter('align_threshold').value
        self.timeout_lost = self.get_parameter('timeout_lost').value

        # 2. 状态变量
        self.last_detection_time = 0.0
        self.current_state = "IDLE" # IDLE, ALIGN, APPROACH, DONE
        
        # 3. 订阅与发布
        self.subscription = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detection_callback,
            10)
            
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 4. 定时器用于处理超时或停止
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Approach Node 已启动，等待目标...")

    def detection_callback(self, msg):
        if len(msg.detections) > 0:
            # 取第一个检测到的物体 (假设是我们要找的目标)
            detection = msg.detections[0]
            self.target_x_current = detection.bbox.center.position.x
            self.target_y_current = detection.bbox.center.position.y
            self.last_detection_time = time.time()
            
            # 更新状态
            if self.current_state == "IDLE":
                self.current_state = "ALIGN"
                self.get_logger().info("发现目标，开始对准...")
        else:
            # 如果没检测到，且之前在运行，会由 control_loop 处理超时
            pass

    def control_loop(self):
        twist = Twist()
        now = time.time()
        
        # 检查是否超时丢失目标
        if self.current_state != "IDLE" and (now - self.last_detection_time > self.timeout_lost):
            if self.current_state == "APPROACH":
                self.get_logger().info("目标消失在画面中，认为已到达，停止。")
                self.current_state = "DONE"
            else:
                self.get_logger().warn("目标丢失，停止运动。")
                self.current_state = "IDLE"
            
            self.stop_robot()
            return

        if self.current_state == "ALIGN":
            error_x = self.target_x - self.target_x_current
            if abs(error_x) < self.align_threshold:
                self.get_logger().info("对准完成，开始前进...")
                self.current_state = "APPROACH"
            else:
                # 简单的比例控制旋转
                twist.angular.z = error_x * self.yaw_kp
                self.cmd_pub.publish(twist)

        elif self.current_state == "APPROACH":
            # 检查是否到达画面最下方
            if self.target_y_current > self.stop_y_threshold:
                self.get_logger().info(f"物体到达底部 (Y={self.target_y_current:.1f})，停止。")
                self.current_state = "DONE"
                self.stop_robot()
            else:
                # 保持对准的同时缓慢前进
                error_x = self.target_x - self.target_x_current
                twist.linear.x = self.forward_speed
                twist.angular.z = error_x * self.yaw_kp
                self.cmd_pub.publish(twist)
                
        elif self.current_state == "DONE":
            self.stop_robot()
            # 可以在这里做一些后续处理，或者回到 IDLE
            # self.current_state = "IDLE"

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
