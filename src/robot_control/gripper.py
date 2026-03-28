
import time

class Gripper:
    def __init__(self, ep_gripper):
        self.ep_gripper = ep_gripper

    def open(self):
        print("开始松开机械爪...")
        try:
            self.ep_gripper.open()
            time.sleep(2)
            print("机械爪已松开")
            return True
        except Exception as e:
            print(f"松开机械爪时出错: {e}")
            return False

    def close(self):
        print("开始闭合机械爪...")
        try:
            self.ep_gripper.close()
            time.sleep(2)
            print("机械爪已闭合")
            return True
        except Exception as e:
            print(f"闭合机械爪时出错: {e}")
            return False
