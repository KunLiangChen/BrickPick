#!/usr/bin/env python3
"""Mechanical arm preset-sequence controller.

Exposes a Trigger service so the behavior tree can request execution of
a configurable sequence of preset positions. Uses a multi-threaded executor
with reentrant callback groups so blocking action-client waits don't stall
the ROS event loop.
"""
import rclpy
import time
import os
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from robomaster_msgs.action import MoveArm
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class EPPresetArmController(Node):
    def __init__(self):
        super().__init__('brickpick_arm_preset')

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameters(namespace='', parameters=[
            ('presets.home.x', 0.0), ('presets.home.z', 0.6),
            ('presets.forward.x', 0.14), ('presets.forward.z', 0.6),
            ('presets.down.x', 0.0), ('presets.down.z', -0.08),
            ('presets.backward.x', -0.05), ('presets.backward.z', 0.0),
            ('use_relative', False), ('goal_timeout', 5.0),
            ('position_limits.x.min', -0.06), ('position_limits.x.max', 0.18),
            ('position_limits.z.min', -0.12), ('position_limits.z.max', 0.06),
            ('emergency_stop_on_error', True),
            ('default_sequence', ['home', 'forward', 'home']),
        ])

        self.presets = {
            k: {
                'x': self.get_parameter(f'presets.{k}.x').value,
                'z': self.get_parameter(f'presets.{k}.z').value,
            }
            for k in ['home', 'forward', 'down', 'backward']
        }
        self.use_relative = self.get_parameter('use_relative').value
        self.limits = {
            ax: {
                'min': self.get_parameter(f'position_limits.{ax}.min').value,
                'max': self.get_parameter(f'position_limits.{ax}.max').value,
            }
            for ax in ['x', 'z']
        }
        self.sequence = self.get_parameter('default_sequence').value

        self._action_client = ActionClient(
            self, MoveArm, 'move_arm', callback_group=self.callback_group)
        self._validate_presets()

        self.active = False
        self._lock = threading.Lock()

        self.srv = self.create_service(
            Trigger, '~/start', self.handle_start,
            callback_group=self.callback_group)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.get_logger().info("Arm Preset Node Already started, waiting for BT trigger...")

    def handle_start(self, req, res):
        with self._lock:
            if self.active:
                res.success, res.message = False, "Sequence already running"
                return res
            self.active = True

        self.status_pub.publish(String(data="EXECUTING"))
        threading.Thread(target=self._run_sequence_thread, daemon=True).start()
        res.success, res.message = True, "Arm sequence triggered"
        return res

    def _run_sequence_thread(self):
        try:
            if not self.wait_for_server(5.0):
                self.status_pub.publish(String(data="FAILURE_NO_SERVER"))
                return

            success = self.execute_sequence()
            self.status_pub.publish(
                String(data="SUCCESS" if success else "FAILURE"))
        except Exception as e:
            self.get_logger().error(f"Sequence error: {e}")
            self.status_pub.publish(String(data="FAILURE"))
        finally:
            self.active = False

    def _validate_presets(self):
        for name, pos in self.presets.items():
            x, z = pos['x'], pos['z']
            if not (self.limits['x']['min'] <= x <= self.limits['x']['max']
                    and self.limits['z']['min'] <= z <= self.limits['z']['max']):
                self.get_logger().warn(f" Preset '{name}' exceeds the limit: [{x},{z}]")

    def wait_for_server(self, timeout_sec=10.0):
        return self._action_client.wait_for_server(timeout_sec)

    def execute_preset(self, preset_name: str):
        """Send a single preset goal and block until result or failure."""
        if preset_name not in self.presets:
            return False

        pos = Point(
            x=float(self.presets[preset_name]['x']),
            y=0.0,
            z=float(self.presets[preset_name]['z']))

        goal = MoveArm.Goal()
        goal.x, goal.z = pos.x, pos.z
        goal.relative = self.use_relative

        # Block the calling thread (not the executor) until the goal resolves.
        future = self._action_client.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.01)

        handle = future.result()
        if not handle or not handle.accepted:
            return False

        result_future = handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.01)

        return result_future.result().status == 4

    def execute_sequence(self, sequence=None):
        seq = sequence or self.sequence
        for name in seq:
            if not self.execute_preset(name):
                if self.get_parameter('emergency_stop_on_error').value:
                    self.execute_preset('home')
                return False
            time.sleep(0.1)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = EPPresetArmController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
