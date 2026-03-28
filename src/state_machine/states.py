
import time
from .. import config
from ..vision.detector import select_best_block

class State:
    def __init__(self, robot, detector):
        self.robot = robot
        self.detector = detector

    def on_enter(self):
        pass

    def execute(self):
        raise NotImplementedError

    def on_exit(self):
        pass

class SearchState(State):
    def on_enter(self):
        print("=== 进入搜索状态 ===")
        self.robot.chassis.search_for_block(config.SEARCH_ROTATION_SPEED)
        self.search_start_time = time.time()

    def execute(self):
        if time.time() - self.search_start_time > config.SEARCH_TIMEOUT:
            print("搜索超时")
            return "failure"

        img = self.robot.camera.read_cv2_image(strategy="newest", timeout=0.5)
        blocks = self.detector.detect_blocks(img)

        if blocks:
            best_block = select_best_block(blocks, img.shape[1], img.shape[0])
            if best_block:
                print(f"检测到积木: {best_block}，切换到对齐状态")
                self.robot.chassis.stop()
                return "align"

        return "searching"

    def on_exit(self):
        self.robot.chassis.stop()
        print("=== 退出搜索状态 ===")

class AlignState(State):
    def on_enter(self):
        print("=== 进入对齐状态 ===")
        self.alignment_stable_count = 0

    def execute(self):
        img = self.robot.camera.read_cv2_image(strategy="newest", timeout=0.5)
        blocks = self.detector.detect_blocks(img)

        if not blocks:
            print("丢失目标，返回搜索状态")
            return "search"

        best_block = select_best_block(blocks, img.shape[1], img.shape[0])
        if not best_block:
            print("丢失目标，返回搜索状态")
            return "search"

        if self.robot.chassis.align_to_block(best_block.center_x, best_block.frame_center_x):
            self.alignment_stable_count += 1
            if self.alignment_stable_count >= config.ALIGNMENT_STABILITY_FRAMES:
                print("对齐完成，切换到靠近状态")
                return "approach"
        else:
            self.alignment_stable_count = 0
        
        return "aligning"

    def on_exit(self):
        self.robot.chassis.stop()
        print("=== 退出对齐状态 ===")

class ApproachState(State):
    def on_enter(self):
        print("=== 进入靠近状态 ===")
        self.robot.chassis.drive_speed(x=config.MOVE_SPEED, y=0, z=0)

    def execute(self):
        # 在这个状态下，我们只依赖测距仪
        if self.robot.get_distance() is not None and self.robot.get_distance() < config.DISTANCE_THRESHOLD:
            print("已到达抓取距离，切换到抓取状态")
            return "grab"
        return "approaching"

    def on_exit(self):
        self.robot.chassis.stop()
        print("=== 退出靠近状态 ===")

class GrabState(State):
    def on_enter(self):
        print("=== 进入抓取状态 ===")
        self.robot.gripper.open()
        self.robot.arm.move_down()
        self.robot.gripper.close()
        time.sleep(config.GRAB_CONFIRMATION_TIME)
        self.robot.arm.move_up()

    def execute(self):
        # 简单的状态，进入后就执行抓取，然后切换
        return "find_marker"

    def on_exit(self):
        print("=== 退出抓取状态 ===")

class FindMarkerState(State):
    def on_enter(self):
        print("=== 进入寻找Marker状态 ===")
        self.robot.vision.sub_detect_info(name="marker", callback=self.robot.on_detect_marker)

    def execute(self):
        target = self.robot.find_target_marker(config.TARGET_MARKER_NAME)
        if target:
            if target.width > config.MARKER_DISTANCE_THRESHOLD:
                print("已到达Marker，切换到释放状态")
                return "release"
            else:
                # 控制前进和转向
                forward_speed = (config.MARKER_DISTANCE_THRESHOLD - target.width) * 1.0
                forward_speed = max(0.1, min(0.5, forward_speed))
                turn_speed = target.offset_x * 60
                turn_speed = max(-60, min(60, turn_speed))
                self.robot.chassis.drive_speed(x=forward_speed, y=0, z=turn_speed)
        else:
            # 旋转寻找
            self.robot.chassis.drive_speed(x=0, y=0, z=15)
        
        return "finding_marker"

    def on_exit(self):
        self.robot.chassis.stop()
        self.robot.vision.unsub_detect_info(name="marker")
        print("=== 退出寻找Marker状态 ===")

class ReleaseState(State):
    def on_enter(self):
        print("=== 进入释放状态 ===")
        self.robot.arm.move_down()
        self.robot.gripper.open()
        self.robot.arm.move_up()
        self.robot.chassis.backward(config.BACKWARD_DISTANCE)

    def execute(self):
        # 释放后，任务完成
        return "success"

    def on_exit(self):
        print("=== 退出释放状态 ===")
