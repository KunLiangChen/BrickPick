#!/usr/bin/env python3
"""Keyboard-triggered image capture node for dataset collection.

Subscribes to a camera topic and saves the latest frame to disk when the
configured capture key is pressed. Uses non-blocking terminal I/O to poll
keystrokes from a timer callback.
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

        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', 'camera/image_color'),
                ('save_dir', '/home/nvidia/brickpick_ws/src/brickpick/dataset'),
                ('capture_key', ' '),
            ]
        )

        self.camera_topic = self.get_parameter('camera_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.capture_key = self.get_parameter('capture_key').value

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f"Create a directory: {self.save_dir}")

        self.bridge = CvBridge()
        self.latest_image = None

        self.settings = termios.tcgetattr(sys.stdin)

        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10)

        self.timer = self.create_timer(0.1, self.keyboard_loop)

        self.get_logger().info("Image capture node has been started.")
        self.get_logger().info(f"Listen to the topic: {self.camera_topic}")
        key_label = 'Space' if self.capture_key == ' ' else self.capture_key
        self.get_logger().info(f"Press '{key_label}' to take a photo and save it to {self.save_dir}")
        self.get_logger().info("Press 'q' to exit")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")

    def get_key(self):
        """Read a single keystroke without blocking the event loop."""
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        except Exception as e:
            self.get_logger().error(f"Keyboard reading error: {str(e)}")
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        key = self.get_key()
        if key != '':
            self.get_logger().info(f"Key detected: '{key}'")

            if key == self.capture_key or key == 's':
                self.get_logger().info("Triggering photo capture...")
                self.capture_image()
            elif key == 'q' or key == '\x03':
                self.get_logger().info("Exiting...")
                raise SystemExit

    def capture_image(self):
        if self.latest_image is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(
                self.save_dir, f"capture_{timestamp}.jpg")

            try:
                cv2.imwrite(filename, self.latest_image)
                self.get_logger().info(f"Image saved successfully: {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {str(e)}")
        else:
            self.get_logger().warn("No image received, cannot take a photo.")


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
