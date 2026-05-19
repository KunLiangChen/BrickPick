#!/usr/bin/env python3
"""Object search node with multi-frame debounce to suppress false positives.

Controls in-place rotation while subscribing to vision/detections.
Requires consecutive-frame confirmation before reporting success,
preventing transient false detections from triggering the behavior tree.
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

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rotate_speed', 0.5),
                ('required_confirm_frames', 4),
                ('min_confidence', 0.6),
            ]
        )

        self.rotate_speed = self.get_parameter('rotate_speed').value
        self.required_confirm_frames = self.get_parameter('required_confirm_frames').value
        self.min_confidence = self.get_parameter('min_confidence').value

        self.found = False
        self.active = False
        self.confirm_count = 0

        self.subscription = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detection_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.srv = self.create_service(Trigger, '~/start', self.handle_start)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.get_logger().info("Find Node has started, awaiting BT trigger...")

        self.timer = self.create_timer(0.1, self.control_loop)

    def handle_start(self, req, res):
        self.active = True
        self.found = False
        self.confirm_count = 0
        self.status_pub.publish(String(data="SEARCHING"))

        self.get_logger().info("Start rotating to search for the target...")
        res.success = True
        res.message = "Find started"
        return res

    def detection_callback(self, msg):
        """Filter detections by confidence and debounce across consecutive frames."""
        if not self.active or self.found:
            return

        valid_detections = False
        if msg.detections:
            for d in msg.detections:
                if len(d.results) > 0 and d.results[0].hypothesis.score >= self.min_confidence:
                    valid_detections = True
                    break

        if valid_detections:
            self.confirm_count += 1
            if self.confirm_count >= self.required_confirm_frames:
                self.found = True
                self.get_logger().info(f" Confirm valid objects for {self.required_confirm_frames} consecutive frames! Stop rotating.")
                self.stop_robot()
        else:
            if self.confirm_count > 0:
                self.confirm_count = 0

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
            self.get_logger().info("Find task completed, status: SUCCESS")

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