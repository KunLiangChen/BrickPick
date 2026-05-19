#!/usr/bin/env python3
"""Object-approach controller with multi-frame debounce and blind-zone takeover.

After a target is locked, this node aligns the robot (yaw correction), drives
forward until the object is too close for the camera, then extends blindly for
a configurable distance before reporting success. EMA filtering suppresses
detection jitter, and a lost-target timeout provides early failure reporting
to the behavior tree.
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

        self.declare_parameters(
            namespace='',
            parameters=[
                ('img_width', 640),
                ('img_height', 360),
                ('target_x_offset', 317.28),
                ('yaw_kp', 0.002),
                ('forward_speed', 0.1),
                ('stop_y_threshold', 320.0),
                ('align_threshold', 15.0),
                ('timeout_lost', 1.0),
                ('tracking_threshold', 120.0),
                ('min_confidence', 0.6),
            ]
        )
        self.declare_parameter('extend_dist', 0.30)
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

        self.last_detection_time = 0.0
        self.current_state = "IDLE"

        self.locked_x = 0.0
        self.locked_y = 0.0
        self.target_x_current = 0.0
        self.target_y_current = 0.0
        self.active = False

        self.extend_start_time = 0.0
        self.extend_duration = self.extend_dist / self.forward_speed

        self.confirm_frames = 0
        self.required_confirm_frames = 3

        self.subscription = self.create_subscription(
            Detection2DArray,
            'vision/detections',
            self.detection_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.srv = self.create_service(Trigger, '~/start', self.handle_start)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.get_logger().info("The Approach Node High Robustness version has been launched, waiting for BT trigger...")

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
        """Lock onto the lowest object in frame, then track via EMA + spatial gating."""
        if not self.active or not msg.detections:
            return

        valid_detections = []
        for d in msg.detections:
            if len(d.results) > 0 and d.results[0].hypothesis.score >= self.min_confidence:
                valid_detections.append(d)

        if not valid_detections:
            return

        if self.current_state == "IDLE":
            target = max(valid_detections, key=lambda d: d.bbox.center.position.y)

            self.confirm_frames += 1
            if self.confirm_frames >= self.required_confirm_frames:
                self.locked_x = target.bbox.center.position.x
                self.locked_y = target.bbox.center.position.y
                self.target_x_current = self.locked_x
                self.target_y_current = self.locked_y
                self.last_detection_time = time.time()
                self.current_state = "ALIGN"
                self.get_logger().info(
                    f" Stably lock onto the bottommost target (X:{self.locked_x:.0f}, Y:{self.locked_y:.0f})")
            return

        # ALIGN / APPROACH: associate via nearest-neighbor within tracking radius.
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
            alpha = 0.6
            self.locked_x = (alpha * best_match.bbox.center.position.x) + ((1 - alpha) * self.locked_x)
            self.locked_y = (alpha * best_match.bbox.center.position.y) + ((1 - alpha) * self.locked_y)

            self.target_x_current = self.locked_x
            self.target_y_current = self.locked_y
            self.last_detection_time = time.time()

    def control_loop(self):
        """State machine: IDLE -> ALIGN -> APPROACH -> EXTEND -> DONE."""
        if not self.active:
            return
        twist = Twist()
        now = time.time()

        if self.current_state == "IDLE" and (now - self.last_detection_time > 5.0):
            self.get_logger().warn("After startup, no valid target was found (false identification), and a FAILURE was reported to the behavior tree...")
            self.active = False
            self.status_pub.publish(String(data="FAILURE"))
            self.stop_robot()
            return

        if self.current_state not in ["IDLE", "EXTEND", "DONE"] and (
                now - self.last_detection_time > self.timeout_lost):
            if self.current_state == "APPROACH":
                if self.target_y_current > (self.stop_y_threshold - 60.0):
                    self.current_state = "EXTEND"
                    self.extend_start_time = time.time()
                    self.get_logger().info("Target enters the downward blind spot, judged as approaching, take over in advance and enter blind driving!")
                else:
                    self.get_logger().warn(" The target is lost despite the long distance; report FAILURE...")
                    self.current_state = "IDLE"
                    self.stop_robot()
                    self._reset_lock()
                    self.active = False
                    self.status_pub.publish(String(data="FAILURE"))
            else:
                self.get_logger().warn(" Failed to lose the target on time, reporting FAILURE...")
                self.current_state = "IDLE"
                self.stop_robot()
                self._reset_lock()
                self.active = False
                self.status_pub.publish(String(data="FAILURE"))
            return

        if self.current_state == "ALIGN":
            error_x = self.target_x - self.target_x_current
            if abs(error_x) < self.align_threshold:
                self.current_state = "APPROACH"
                self.get_logger().info("Aligned, complete, starting forward...")
            else:
                twist.angular.z = error_x * self.yaw_kp
                self.cmd_pub.publish(twist)
            self.status_pub.publish(String(data="ALIGNING"))

        elif self.current_state == "APPROACH":
            if self.target_y_current > self.stop_y_threshold:
                self.current_state = "EXTEND"
                self.extend_start_time = time.time()
                self.get_logger().info(f"Reach the Y-axis threshold, start blind driving extension {self.extend_dist}m...")
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
                self.get_logger().info(" Blind driving completed. Stop.")
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
        self.confirm_frames = 0

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
