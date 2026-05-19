#!/usr/bin/env python3
"""Incremental keyboard teleoperation node (TurtleBot3-style).

Keys add/subtract from a persistent velocity state rather than setting
absolute values. A 10 Hz timer publishes the current velocity continuously
to prevent the motor controller from timing out.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
Control Your Robomaster EP (TurtleBot3 Style)!
---------------------------
Moving around:
   w    x
   a    d
   q    e

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity (yaw)
q/e : increase/decrease angular velocity (strafe, mecanum only)

s : force stop

CTRL-C to quit
"""

moveBindings = {
    'w': (0.1, 0.0, 0.0),
    'x': (-0.1, 0.0, 0.0),
    'a': (0.0, 0.0, 0.1),
    'd': (0.0, 0.0, -0.1),
    'q': (0.0, 0.1, 0.0),
    'e': (0.0, -0.1, 0.0),
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
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_th = 0.0

        self.max_speed = 1.0
        self.max_turn = 2.0

        self.timer = self.create_timer(0.1, self.run_loop)
        self.get_logger().info(msg)

    def run_loop(self):
        key = getKey(self.settings)

        if key in moveBindings.keys():
            self.target_x += moveBindings[key][0]
            self.target_y += moveBindings[key][1]
            self.target_th += moveBindings[key][2]

            self.target_x = max(-self.max_speed,
                                min(self.max_speed, self.target_x))
            self.target_y = max(-self.max_speed,
                                min(self.max_speed, self.target_y))
            self.target_th = max(-self.max_turn,
                                 min(self.max_turn, self.target_th))

            self.get_logger().info(
                f'Current Speed -> X:{self.target_x:.1f}, '
                f'Y:{self.target_y:.1f}, Th:{self.target_th:.1f}')

        elif key == 's':
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_th = 0.0
            self.get_logger().info('Robot forced stop.')

        elif key == '\x03':
            self.destroy_node()
            rclpy.shutdown()
            sys.exit()

        twist = Twist()
        twist.linear.x = self.target_x
        twist.linear.y = self.target_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.target_th

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
