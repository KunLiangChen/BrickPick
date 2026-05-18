#!/usr/bin/env python3
"""
brickpick: 寻找物体节点 (工业防抖版)
- 控制机器人原地旋转
- 订阅 vision/detections
- 过滤低置信度目标，并且必须连续多帧确认，防止闪烁误报
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_srvs.srv import Trigger          
from std_msgs.msg import String           

class FindNode(Node):
    def __init__(self):
        super().__init__('find_node')
        
        # 1. 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rotate_speed', 0.5),            # 默认旋转速度 (rad/s)
                ('required_confirm_frames', 4),   # 🌟 必须连续 3 帧看清才算数
                ('min_confidence', 0.6)           # 🌟 置信度阈值
            ]
        )
        
        self.rotate_speed = self.get_parameter('rotate_speed').value
        self.required_confirm_frames = self.get_parameter('required_confirm_frames').value
        self.min_confidence = self.get_parameter('min_confidence').value

        # 2. 状态变量
        self.found = False
        self.active = False
        self.confirm_count = 0  # 🌟 连续确认识别帧数的计数器
        
        # 3. 订阅与发布
        self.subscription = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detection_callback,
            10)
            
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.srv = self.create_service(Trigger, '~/start', self.handle_start)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        
        self.get_logger().info("Find Node 已启动，等待 BT 触发...")
        
        # 4. 定时器用于发布控制指令
        self.timer = self.create_timer(0.1, self.control_loop)

    def handle_start(self, req, res):
        self.active = True
        self.found = False
        self.confirm_count = 0  # 🌟 每次重新寻找时，清空计数器
        self.status_pub.publish(String(data="SEARCHING"))
        
        self.get_logger().info("开始旋转寻找目标...")
        res.success = True
        res.message = "Find started"
        return res
    
    def detection_callback(self, msg):
        # 如果节点未激活，或者已经找到目标了，就不再处理视觉消息
        if not self.active or self.found: 
            return       
        
        valid_detections = False
        if msg.detections:
            for d in msg.detections:
                # 检查置信度
                if len(d.results) > 0 and d.results[0].hypothesis.score >= self.min_confidence: 
                    valid_detections = True
                    break

        # 🌟 核心防抖逻辑
        if valid_detections:
            self.confirm_count += 1
            if self.confirm_count >= self.required_confirm_frames:
                self.found = True
                self.get_logger().info(f"✅ 连续 {self.required_confirm_frames} 帧确认有效物体！停止旋转。")
                self.stop_robot()
        else:
            # 只要有一帧没看到（或者置信度太低），计数器清零，防止把断断续续的噪点当成目标
            if self.confirm_count > 0:
                self.confirm_count = 0
                # 取消下面这行注释可以查看防抖过程（调试用，日常建议注释掉防刷屏）
                # self.get_logger().debug("视野闪烁，重置防抖计数器")

    def control_loop(self):
        if not self.active: 
            return       
            
        if not self.found:
            twist = Twist()
            twist.angular.z = self.rotate_speed
            self.cmd_pub.publish(twist)
            self.status_pub.publish(String(data="SEARCHING"))
        else:
            self.stop_robot()
            self.active = False           
            self.status_pub.publish(String(data="SUCCESS"))
            self.get_logger().info("Find 任务完成，状态: SUCCESS")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = FindNode()
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