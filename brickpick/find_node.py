#!/usr/bin/env python3
"""
brickpick: 寻找物体节点
- 控制机器人原地旋转
- 订阅 vision/detections
- 一旦检测到物体，停止旋转
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray

class FindNode(Node):
    def __init__(self):
        super().__init__('find_node')
        
        # 1. 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rotate_speed', 0.6),  # 默认旋转速度 (rad/s)
            ]
        )
        
        self.rotate_speed = self.get_parameter('rotate_speed').value

        # 2. 状态变量
        self.found = False
        
        # 3. 订阅与发布
        self.subscription = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detection_callback,
            10)
            
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 4. 定时器用于发布控制指令
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Find Node 已启动，正在旋转寻找目标...")

    def detection_callback(self, msg):
        # 如果检测到数组不为空，说明看到了物体
        if len(msg.detections) > 0:
            if not self.found:
                self.get_logger().info("检测到物体！停止旋转。")
                self.found = True
                self.stop_robot()

    def control_loop(self):
        if not self.found:
            twist = Twist()
            twist.angular.z = self.rotate_speed
            self.cmd_pub.publish(twist)
        else:
            # 找到后持续发送停止指令，防止惯性滑动
            self.stop_robot()

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