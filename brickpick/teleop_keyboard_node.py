#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import time

msg = """
Control Your Robomaster EP!
---------------------------
Moving around:
   q    w    e
   a    s    d
        x

w/x : increase/decrease linear x speed (forward/backward)
a/d : increase/decrease linear y speed (strafe left/right)
q/e : increase/decrease angular z speed (rotate left/right)

s : stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),
    'x': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}

speedBindings = {
    'i': (1.1, 1.1),
    'k': (.9, .9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.speed = 0.2
        self.turn = 0.5
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.status = 0
        self.last_twist = (0.0, 0.0, 0.0)
        self.last_key_time = time.time()
        self.key_timeout = 0.1  # 0.1 seconds timeout

        self.timer = self.create_timer(0.1, self.run_loop)
        self.get_logger().info(msg)

    def run_loop(self):
        key = getKey(self.settings)
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.th = moveBindings[key][2]
            self.last_key_time = time.time()
        elif key == 's':
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0
            self.last_key_time = 0.0 # Force immediate stop
        elif key == '\x03':  # CTRL-C
            self.destroy_node()
            rclpy.shutdown()
            sys.exit()
        else:
            # Check if we should stop due to timeout
            if time.time() - self.last_key_time > self.key_timeout:
                self.x = 0.0
                self.y = 0.0
                self.th = 0.0

        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th * self.turn
        
        current_twist = (twist.linear.x, twist.linear.y, twist.angular.z)
        if current_twist != self.last_twist:
            if all(v == 0 for v in current_twist):
                self.get_logger().info('Robot stopped.')
            else:
                self.get_logger().info(f'Moving: x={twist.linear.x:.2f}, y={twist.linear.y:.2f}, yaw={twist.angular.z:.2f}')
            self.last_twist = current_twist
        
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
