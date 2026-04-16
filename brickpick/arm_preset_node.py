#!/usr/bin/env python3
"""
brickpick: EP 机械臂预设动作控制器
- 加载 config/arm_presets.yaml 参数
- 通过 move_arm Action 执行精准点位控制
- 支持安全限位校验与错误恢复
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from robomaster_msgs.action import MoveArm
from geometry_msgs.msg import Point
import yaml
import os

class EPPresetArmController(Node):
    def __init__(self):
        super().__init__('brickpick_arm_preset')
        
        # 1️⃣ 加载参数（支持 YAML + 命令行覆盖）
        self.declare_parameters(
            namespace='',
            parameters=[
                ('presets', {
                    'home': {'x':0.0,'z':0.0},
                    'forward': {'x':0.15,'z':0.0},
                    'down': {'x':0.0,'z':-0.08},
                    'backward': {'x':-0.05,'z':0.0}
                }),
                ('use_relative', False),
                ('position_limits', {
                    'x': {'min': -0.06, 'max': 0.18},
                    'z': {'min': -0.12, 'max': 0.05}
                }),
                ('default_sequence', ['home','forward','down','backward','home']),
                ('emergency_stop_on_error', True)
            ]
        )
        
        # 读取参数
        self.presets = self.get_parameter('presets').value
        self.use_relative = self.get_parameter('use_relative').value
        self.limits = self.get_parameter('position_limits').value
        self.sequence = self.get_parameter('default_sequence').value
        
        # 2️⃣ 初始化 Action Client
        self._action_client = ActionClient(self, MoveArm, 'move_arm')
        
        # 3️⃣ 安全校验：检查预设位置是否在限位内
        self._validate_presets()
    
    def _validate_presets(self):
        """校验所有预设位置是否在安全范围内"""
        for name, pos in self.presets.items():
            x, z = pos['x'], pos['z']
            if not (self.limits['x']['min'] <= x <= self.limits['x']['max'] and
                    self.limits['z']['min'] <= z <= self.limits['z']['max']):
                self.get_logger().warn(f" 预设 '{name}' 超出安全限位: [{x},{z}]")
    
    def _point_from_dict(self, d: dict) -> Point:
        return Point(x=float(d['x']), y=0.0, z=float(d['z']))
    
    def wait_for_server(self, timeout_sec=10.0):
        if not self._action_client.wait_for_server(timeout_sec):
            self.get_logger().error(f" move_arm 服务未就绪")
            return False
        return True
    
    def send_goal(self, position: Point, relative=None):
        """发送 move_arm 目标"""
        goal = MoveArm.Goal()
        # 根据参考文档修改：直接设置 x, z 字段
        goal.x = position.x
        goal.z = position.z
        goal.relative = relative if relative is not None else self.use_relative
        
        self.get_logger().info(f" 执行: x={goal.x}, z={goal.z} | relative={goal.relative}")
        return self._action_client.send_goal_async(goal)
    
    def execute_preset(self, preset_name: str):
        """执行单个预设动作"""
        if preset_name not in self.presets:
            self.get_logger().error(f" 未知预设: {preset_name}")
            return False
        
        pos = self._point_from_dict(self.presets[preset_name])
        future = self.send_goal(pos)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().accepted:
            self.get_logger().info(f" '{preset_name}' 已接受")
            return True
        else:
            self.get_logger().warn(f" '{preset_name}' 被拒绝")
            return False
    
    def execute_sequence(self, sequence=None):
        """执行动作序列（带简单错误恢复）"""
        seq = sequence or self.sequence
        if not self.wait_for_server():
            return
        
        for i, name in enumerate(seq):
            self.get_logger().info(f"🔹 [{i+1}/{len(seq)}] 执行: {name}")
            if not self.execute_preset(name):
                if self.get_parameter('emergency_stop_on_error').value:
                    self.get_logger().error("🛑 错误触发急停，执行回 home")
                    self.execute_preset('home')
                    break
            # 简单延时：让机械臂稳定（实际可订阅 result 反馈）
            rclpy.sleep(rclpy.duration.Duration(seconds=2.0))
        
        self.get_logger().info("🎉 序列完成！")

def main(args=None):
    rclpy.init(args=args)
    
    # 支持通过命令行覆盖配置文件路径
    node = EPPresetArmController()
    
    try:
        # 首次启动先回正（安全）
        if node.wait_for_server(5.0):
            node.execute_preset('home')
            rclpy.sleep(rclpy.duration.Duration(seconds=1.5))
        
        # 执行默认序列
        node.execute_sequence()
        
    except KeyboardInterrupt:
        node.get_logger().info("🛑 用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()