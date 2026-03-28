
import time
import numpy as np
from .. import config

class Chassis:
    def __init__(self, ep_chassis):
        self.ep_chassis = ep_chassis

    def move(self, x, y, z, speed):
        self.ep_chassis.move(x=x, y=y, z=z, xy_speed=speed).wait_for_completed()

    def drive_speed(self, x, y, z, timeout=None):
        self.ep_chassis.drive_speed(x=x, y=y, z=z, timeout=timeout)

    def search_for_block(self, search_rotation_speed):
        print("开始搜索积木...")
        self.drive_speed(x=0, y=0, z=search_rotation_speed, timeout=0.1)

    def align_to_block(self, center_x, frame_center_x):
        error_x = center_x - frame_center_x
        print(f"对齐误差 X: {error_x:.1f}")

        if abs(error_x) <= config.ALIGNMENT_TOLERANCE:
            self.drive_speed(x=0, y=0, z=0, timeout=0.1)
            return True
        else:
            z_speed = error_x * config.ROTATE_SPEED_P
            z_speed = np.clip(z_speed, -30, 30)
            print(f"旋转速度 Z: {z_speed:.2f}")
            self.drive_speed(x=0, y=0, z=z_speed, timeout=0.1)
            return False

    def stop(self):
        self.drive_speed(x=0, y=0, z=0, timeout=0.1)

    def backward(self, distance):
        print(f"开始后退 {distance} 米...")
        try:
            self.move(x=-distance, y=0, z=0, speed=0.3)
            print("后退完成")
            return True
        except Exception as e:
            print(f"后退时出错: {e}")
            return False
