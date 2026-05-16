#!/usr/bin/env python3
"""
brickpick: 接近物体节点 (工业级高鲁棒性版)
- 增加了置信度过滤与多帧防抖
- 增加了坐标平滑滤波 (EMA) 防止抖动
- 增加了“盲区智能接管”逻辑，防止靠近时丢失导致抓空
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import time
from std_srvs.srv import Trigger
from std_msgs.msg import String
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
                ('stop_y_threshold', 320.0), # 稍微调高一点，避免刚好卡在盲区边缘
                ('align_threshold', 15.0),
                ('timeout_lost', 1.0),
                ('tracking_threshold', 120.0), # 🔑 放宽一点容差，防止旋转时跟丢
                ('min_confidence', 0.6)        # 🔑 新增：置信度阈值，低于此概率的框直接忽略
            ]
        )
        self.declare_parameter('extend_dist', 0.20)
        self.extend_dist = self.get_parameter('extend_dist').value
        
        self.img_width = self.get_parameter('img_width').value
        self.img_height = self.get_parameter('img_height').value
        self.target_x = self.get_parameter('target_x_offset').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.stop_y_threshold = self.get_parameter('stop_y_threshold').value
        self.align_threshold = self.get_parameter('align_threshold').value
        self.timeout_lost = self.get_parameter('timeout_lost').value
        self.tracking_threshold = self.get_parameter('tracking_threshold').value
        self.min_confidence = self.get_parameter('min_confidence').value
        
        # 2. 状态与锁定变量
        self.last_detection_time = 0.0
        self.current_state = "IDLE"
        
        self.locked_x = 0.0
        self.locked_y = 0.0
        self.target_x_current = 0.0 
        self.target_y_current = 0.0 
        self.active = False
        
        self.extend_start_time = 0.0
        self.extend_duration = self.extend_dist / self.forward_speed
        
        # 🔑 新增：防抖计数器
        self.confirm_frames = 0
        self.required_confirm_frames = 3 # 必须连续3帧看到同一个位置才锁定，防止闪烁误识别

        # 3. 订阅与发布
        self.subscription = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detection_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.srv = self.create_service(Trigger, '~/start', self.handle_start)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        
        self.get_logger().info("Approach Node 高鲁棒版已启动，等待 BT 触发...")

    def handle_start(self, req, res):
        self.active = True
        self.current_state = "IDLE"
        self._reset_lock()
        self.last_detection_time = time.time()
        self.status_pub.publish(String(data="WAITING_FOR_DETECTION"))
        res.success = True
        res.message = "Approach started"
        return res
    
    def detection_callback(self, msg):
        if not self.active or not msg.detections:
            return

        # 🔑 过滤低置信度的“假积木”框
        valid_detections = []
        for d in msg.detections:
            # 假设你的 ROS YOLO 包装器将分数放在 results[0].score 中
            if len(d.results) > 0 and d.results[0].hypothesis.score >= self.min_confidence:
                valid_detections.append(d)
        
        if not valid_detections:
            return

        # 🟢 状态 IDLE：寻找并锁定最下方的目标
        if self.current_state == "IDLE":
            target = max(valid_detections, key=lambda d: d.bbox.center.position.y)
            
            # 防抖逻辑：必须连续几帧看到它
            self.confirm_frames += 1
            if self.confirm_frames >= self.required_confirm_frames:
                self.locked_x = target.bbox.center.position.x
                self.locked_y = target.bbox.center.position.y
                self.target_x_current = self.locked_x
                self.target_y_current = self.locked_y
                self.last_detection_time = time.time()
                self.current_state = "ALIGN"
                self.get_logger().info(f"🔒 稳定锁定最下方目标 (X:{self.locked_x:.0f}, Y:{self.locked_y:.0f})")
            return

        # 🟡 状态 ALIGN / APPROACH：空间距离过滤，防止跳变
        best_match = None
        min_dist = float('inf')

        for d in valid_detections:
            dx = d.bbox.center.position.x - self.locked_x
            dy = d.bbox.center.position.y - self.locked_y
            dist = math.hypot(dx, dy)
            
            if dist < min_dist:
                min_dist = dist
                best_match = d

        if best_match is not None and min_dist <= self.tracking_threshold:
            # 🔑 引入 EMA (指数移动平均) 平滑滤波，让目标的坐标变化更平滑，小车不再“抽搐”
            alpha = 0.6  # 信任新数据的比例
            self.locked_x = (alpha * best_match.bbox.center.position.x) + ((1 - alpha) * self.locked_x)
            self.locked_y = (alpha * best_match.bbox.center.position.y) + ((1 - alpha) * self.locked_y)
            
            self.target_x_current = self.locked_x
            self.target_y_current = self.locked_y
            self.last_detection_time = time.time()

    def control_loop(self):
        if not self.active: return
        twist = Twist()
        now = time.time()
        
        if self.current_state == "IDLE" and (now - self.last_detection_time > 5.0):
            self.get_logger().warn("❌ 启动后未发现有效目标 (假识别)，上报 FAILURE 给行为树...")
            self.active = False
            self.status_pub.publish(String(data="FAILURE")) # 立刻告诉 BT 我失败了
            self.stop_robot()
            return
        
        # 🌟 核心修改 2：处理视觉丢失的情况
        if self.current_state not in ["IDLE", "EXTEND"] and (now - self.last_detection_time > self.timeout_lost):
            if self.current_state == "APPROACH":
                if self.target_y_current > (self.stop_y_threshold - 60.0):
                    self.current_state = "EXTEND"
                    self.extend_start_time = time.time()
                    self.get_logger().info("⚠️ 目标进入下视盲区，判定为接近，提前接管并进入盲驶！")
                else:
                    self.get_logger().warn("❌ 距离尚远却丢失目标，上报 FAILURE...")
                    self.current_state = "IDLE"
                    self.stop_robot()
                    self._reset_lock()
                    
                    self.active = False
                    self.status_pub.publish(String(data="FAILURE")) # 丢失目标立刻上报失败！
            else:
                self.get_logger().warn("❌ 对准时丢失目标，上报 FAILURE...")
                self.current_state = "IDLE"
                self.stop_robot()
                self._reset_lock()
                
                self.active = False
                self.status_pub.publish(String(data="FAILURE")) # 对准失败也立刻上报！
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
                self.current_state = "EXTEND"
                self.extend_start_time = time.time()
                self.get_logger().info(f"到达 Y 轴阈值，开始盲驶延长 {self.extend_dist}m...")
            else:
                twist.linear.x = self.forward_speed
                twist.angular.z = (self.target_x - self.target_x_current) * self.yaw_kp
                self.cmd_pub.publish(twist)     
            self.status_pub.publish(String(data="APPROACHING"))
        
        elif self.current_state == "EXTEND":
            elapsed = time.time() - self.extend_start_time
            if elapsed >= self.extend_duration:
                self.current_state = "DONE"
                self.stop_robot()
                self.get_logger().info("✅ 盲驶完成，停止。")
            else:
                twist.linear.x = self.forward_speed
                self.cmd_pub.publish(twist)
            self.status_pub.publish(String(data="APPROACHING"))
        
        elif self.current_state == "DONE":
            self.active = False           
            self.status_pub.publish(String(data="SUCCESS"))
            self.stop_robot()

    def _reset_lock(self):
        self.locked_x = 0.0
        self.locked_y = 0.0
        self.target_x_current = 0.0
        self.target_y_current = 0.0
        self.confirm_frames = 0 # 重置防抖计数器

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