
class Arm:
    def __init__(self, ep_arm):
        self.ep_arm = ep_arm

    def move_up(self):
        print("开始抬起机械臂...")
        try:
            self.ep_arm.move(x=40).wait_for_completed()
            print("X轴移动完成")
            self.ep_arm.move(y=90).wait_for_completed()
            print("Y轴移动完成")
            print("机械臂抬起完成")
            return True
        except Exception as e:
            print(f"抬起机械臂时出错: {e}")
            return False

    def move_down(self):
        print("开始放下机械臂...")
        try:
            self.ep_arm.move(y=-90).wait_for_completed()
            print("Y轴移动完成")
            self.ep_arm.move(x=-40).wait_for_completed()
            print("X轴移动完成")
            print("机械臂放下完成")
            return True
        except Exception as e:
            print(f"放下机械臂时出错: {e}")
            return False
