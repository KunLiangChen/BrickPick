#!/usr/bin/env python3
import rclpy, time, os, threading
from rclpy.node import Node
from rclpy.action import ActionClient
from robomaster_msgs.action import MoveArm
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger          # 🔹 [BT集成]
from std_msgs.msg import String           # 🔹 [BT集成]

class EPPresetArmController(Node):
    def __init__(self):
        super().__init__('brickpick_arm_preset')
        self.declare_parameters(namespace='', parameters=[
            ('presets.home.x', 0.0), ('presets.home.z', 0.0),
            ('presets.forward.x', 0.15), ('presets.forward.z', 0.0),
            ('presets.down.x', 0.0), ('presets.down.z', -0.08),
            ('presets.backward.x', -0.05), ('presets.backward.z', 0.0),
            ('use_relative', False), ('goal_timeout', 5.0),
            ('position_limits.x.min', -0.06), ('position_limits.x.max', 0.18),
            ('position_limits.z.min', -0.12), ('position_limits.z.max', 0.05),
            ('emergency_stop_on_error', True),
            ('default_sequence', ['home', 'forward', 'down', 'backward', 'home'])
        ])
        self.presets = {k: {'x': self.get_parameter(f'presets.{k}.x').value, 
                            'z': self.get_parameter(f'presets.{k}.z').value} 
                        for k in ['home','forward','down','backward']}
        self.use_relative = self.get_parameter('use_relative').value
        self.limits = {ax: {'min': self.get_parameter(f'position_limits.{ax}.min').value,
                            'max': self.get_parameter(f'position_limits.{ax}.max').value} for ax in ['x','z']}
        self.sequence = self.get_parameter('default_sequence').value
        self._action_client = ActionClient(self, MoveArm, 'move_arm')
        self._validate_presets()
        
        self.active = False               # 🔹 [BT集成]
        self._lock = threading.Lock()
        self.srv = self.create_service(Trigger, '~/start', self.handle_start)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.get_logger().info("Arm Preset Node 已启动，等待 BT 触发...")

    def handle_start(self, req, res):     # 🔹 [BT集成]
        with self._lock:
            if self.active:
                res.success, res.message = False, "Sequence already running"
                return res
            self.active = True
        self.status_pub.publish(String(data="EXECUTING"))
        threading.Thread(target=self._run_sequence_thread, daemon=True).start()
        res.success, res.message = True, "Arm sequence triggered"
        return res

    def _run_sequence_thread(self):       # 🔹 [BT集成] 后台执行
        try:
            if not self.wait_for_server(5.0):
                self.status_pub.publish(String(data="FAILURE_NO_SERVER"))
                return
            self.execute_preset('home')
            time.sleep(0.005)
            success = self.execute_sequence()
            self.status_pub.publish(String(data="SUCCESS" if success else "FAILURE"))
        except Exception as e:
            self.get_logger().error(f"Sequence error: {e}")
            self.status_pub.publish(String(data="FAILURE"))
        finally:
            self.active = False

    def _validate_presets(self):
        for name, pos in self.presets.items():
            x, z = pos['x'], pos['z']
            if not (self.limits['x']['min'] <= x <= self.limits['x']['max'] and
                    self.limits['z']['min'] <= z <= self.limits['z']['max']):
                self.get_logger().warn(f"⚠️ 预设 '{name}' 超出限位: [{x},{z}]")

    def _point_from_dict(self, d: dict) -> Point:
        return Point(x=float(d['x']), y=0.0, z=float(d['z']))

    def wait_for_server(self, timeout_sec=10.0):
        if not self._action_client.wait_for_server(timeout_sec):
            self.get_logger().error("❌ move_arm Action 未就绪")
            return False
        return True

    def send_goal(self, position: Point, relative=None):
        goal = MoveArm.Goal()
        goal.x, goal.z = position.x, position.z
        goal.relative = relative if relative is not None else self.use_relative
        return self._action_client.send_goal_async(goal)

    def execute_preset(self, preset_name: str):
        if preset_name not in self.presets: return False
        pos = self._point_from_dict(self.presets[preset_name])
        future = self.send_goal(pos)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or not future.result().accepted: return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().status == 4

    def execute_sequence(self, sequence=None):  # 🔹 改为返回 bool
        seq = sequence or self.sequence
        for i, name in enumerate(seq):
            if not self.execute_preset(name):
                if self.get_parameter('emergency_stop_on_error').value:
                    self.execute_preset('home')
                return False
            time.sleep(0.005)
        return True

def main(args=None):
    rclpy.init(args=args)
    node = EPPresetArmController()
    try: rclpy.spin(node)                 # 🔹 移除自动执行，仅 spin
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()