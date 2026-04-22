#!/usr/bin/env python3
"""
brickpick: 接近物体节点 (无ID空间锁定版)
- 订阅 vision/detections
- 首次检测时锁定画面最下方（离车最近）的目标坐标
- 后续仅追踪空间距离最近的框，防止多目标跳变
- 控制机器人旋转对准，对准后缓慢前进
- 到达画面底部或丢失超时后停止
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import time
from std_srvs.srv import Trigger          # 🔹 [BT集成]
from std_msgs.msg import String           # 🔹 [BT集成]
import math

class ApproachNode(Node):
    def __init__(self):
        super().__init__('approach_node')
        # 1. 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('img_width', 640),
                ('img_height', 360),
                ('target_x_offset', 317.28),
                ('yaw_kp', 0.002),
                ('forward_speed', 0.1),
                ('stop_y_threshold', 340.0),
                ('align_threshold', 15.0),
                ('timeout_lost', 1.0),
                ('tracking_threshold', 80.0)  # 🔑 新增：空间连续追踪容差(像素)
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
        self.tracking_threshold = self.get_parameter('tracking_threshold').value
        
        # 2. 状态与锁定变量
        self.last_detection_time = 0.0
        self.current_state = "IDLE" # IDLE, ALIGN, APPROACH, DONE
        
        self.locked_x = 0.0  # 🔑 锁定目标的中心X
        self.locked_y = 0.0  # 🔑 锁定目标的中心Y
        self.target_x_current = 0.0 # 当前控制用的X
        self.target_y_current = 0.0 # 当前控制用的Y
        self.active = False

        # 3. 订阅与发布
        self.subscription = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detection_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 4. 控制定时器
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Approach Node 已启动，等待目标...")

        self.srv = self.create_service(Trigger, '~/start', self.handle_start)  # 🔹 [BT集成]
        self.status_pub = self.create_publisher(String, '~/status', 10)        # 🔹 [BT集成]
        self.get_logger().info("Approach Node 已启动，等待 BT 触发...")

    def handle_start(self, req, res):     # 🔹 [BT集成]
        self.active = True
        self.current_state = "IDLE"
        self._reset_lock()
        self.last_detection_time = time.time()
        self.status_pub.publish(String(data="WAITING_FOR_DETECTION"))
        res.success = True
        res.message = "Approach started"
        return res
    
    def detection_callback(self, msg):
        if not msg.detections:
            return

        # 🟢 状态 IDLE：寻找并锁定最下方的目标
        if self.current_state == "IDLE":
            # Y坐标越大越靠近画面底部
            target = max(msg.detections, key=lambda d: d.bbox.center.position.y)
            
            self.locked_x = target.bbox.center.position.x
            self.locked_y = target.bbox.center.position.y
            self.target_x_current = self.locked_x
            self.target_y_current = self.locked_y
            self.last_detection_time = time.time()
            
            self.current_state = "ALIGN"
            self.get_logger().info(f"🔒 已锁定最下方目标 (X:{self.locked_x:.0f}, Y:{self.locked_y:.0f})，开始对准...")
            return

        # 🟡 状态 ALIGN / APPROACH：空间距离过滤，防止跳变
        best_match = None
        min_dist = float('inf')

        for d in msg.detections:
            dx = d.bbox.center.position.x - self.locked_x
            dy = d.bbox.center.position.y - self.locked_y
            dist = math.hypot(dx, dy) # 等价于 (dx**2 + dy**2)**0.5
            
            if dist < min_dist:
                min_dist = dist
                best_match = d

        # 仅当最近的目标在容差范围内时，才更新锁定位置
        if best_match is not None and min_dist <= self.tracking_threshold:
            self.locked_x = best_match.bbox.center.position.x
            self.locked_y = best_match.bbox.center.position.y
            self.target_x_current = self.locked_x
            self.target_y_current = self.locked_y
            self.last_detection_time = time.time()
        # 否则：忽略本帧所有检测框，交由 control_loop 的超时逻辑处理

    def control_loop(self):
        if not self.active: return
        twist = Twist()
        now = time.time()
        if self.current_state != "IDLE" and (now - self.last_detection_time > self.timeout_lost):
            if self.current_state == "APPROACH":
                self.current_state = "DONE"
                self.stop_robot()
            else:
                self.current_state = "IDLE"
                self.stop_robot()
                self._reset_lock()
                return

        if self.current_state == "ALIGN":
            error_x = self.target_x - self.target_x_current
            if abs(error_x) < self.align_threshold:
                self.current_state = "APPROACH"
                self.get_logger().info("对准完成，开始前进...")
            else:
                twist.angular.z = error_x * self.yaw_kp
                self.cmd_pub.publish(twist)
            self.status_pub.publish(String(data="ALIGNING"))
        elif self.current_state == "APPROACH":
            if self.target_y_current > self.stop_y_threshold:
                self.current_state = "DONE"
                self.stop_robot()
                self.get_logger().info("到达阈值，停止。")
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = (self.target_x - self.target_x_current) * self.yaw_kp
                self.cmd_pub.publish(twist)
            self.status_pub.publish(String(data="APPROACHING"))
        elif self.current_state == "DONE":
            self.active = False           # 🔹 [BT集成] 任务完成
            self.status_pub.publish(String(data="SUCCESS"))
            self.stop_robot()

    def _reset_lock(self):
        """重置锁定状态，准备下一次任务"""
        self.locked_x = 0.0
        self.locked_y = 0.0
        self.target_x_current = 0.0
        self.target_y_current = 0.0

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