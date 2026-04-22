#!/usr/bin/env python3
"""
brickpick: 图像抓拍节点
- 订阅相机图像话题
- 监听键盘输入
- 当按下指定按键时（默认为空格或's'），保存当前帧图像到指定目录
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import select
import termios
import tty
from datetime import datetime

class ImageCaptureNode(Node):
    def __init__(self):
        super().__init__('image_capture_node')
        
        # 1. 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', 'camera/image_color'),
                ('save_dir', '/home/nvidia/brickpick_ws/src/brickpick/dataset'),
                ('capture_key', ' '), # 默认为空格键
            ]
        )
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.capture_key = self.get_parameter('capture_key').value
        
        # 2. 确保保存目录存在
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"创建目录: {self.save_dir}")
            
        # 3. 初始化
        self.bridge = CvBridge()
        self.latest_image = None
        
        # 4. 键盘配置
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 5. 订阅与定时器
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)
            
        # 使用定时器定期检查键盘输入
        self.timer = self.create_timer(0.1, self.keyboard_loop)
        
        self.get_logger().info(f"图像抓拍节点已启动")
        self.get_logger().info(f"监听话题: {self.camera_topic}")
        self.get_logger().info(f"按 '{'Space' if self.capture_key == ' ' else self.capture_key}' 键拍照保存到 {self.save_dir}")
        self.get_logger().info(f"按 'q' 键退出")

    def image_callback(self, msg):
        try:
            # 将 ROS 图像转换为 OpenCV 格式
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    def get_key(self):
        try:
            tty.setraw(sys.stdin.fileno())
            # 增加 select 超时时间到 0.02s，提高检测稳定性
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        except Exception as e:
            self.get_logger().error(f"读取键盘错误: {str(e)}")
            key = ''
       
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        key = self.get_key()
        if key != '':
            # 只要有按键就打印，方便用户调试
            self.get_logger().info(f"检测到按键: '{key}'")
            
            if key == self.capture_key or key == 's':
                self.get_logger().info("正在触发拍照...")
                self.capture_image()
            elif key == 'q' or key == '\x03': # q 或 CTRL-C
                self.get_logger().info("正在退出...")
                # 在退出前恢复终端设置
                # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
                # self.destroy_node()
                # rclpy.shutdown()
                # sys.exit()
                raise SystemExit

    def capture_image(self):
        if self.latest_image is not None:
            # 使用当前时间作为文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.save_dir, f"capture_{timestamp}.jpg")
            
            try:
                cv2.imwrite(filename, self.latest_image)
                self.get_logger().info(f"成功保存图像: {filename}")
            except Exception as e:
                self.get_logger().error(f"保存图像失败: {str(e)}")
        else:
            self.get_logger().warn("尚未接收到图像，无法拍照")

def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,SystemExit):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()