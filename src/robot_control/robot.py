
from .chassis import Chassis
from .arm import Arm
from .gripper import Gripper
from ..vision.marker import MarkerInfo
from .. import config

class Robot:
    def __init__(self, ep_robot):
        self.ep_robot = ep_robot
        self.chassis = Chassis(ep_robot.chassis)
        self.arm = Arm(ep_robot.robotic_arm)
        self.gripper = Gripper(ep_robot.gripper)
        self.camera = ep_robot.camera
        self.vision = ep_robot.vision
        self.sensor = ep_robot.sensor
        self.current_distance = None
        self.markers = []

    def initialize(self):
        self.ep_robot.initialize(conn_type="ap")
        self.camera.start_video_stream(display=False)
        self.sensor.sub_distance(freq=5, callback=self.distance_callback)

    def distance_callback(self, adapter_info):
        try:
            sensor_data = adapter_info[config.DISTANCE_SENSOR_INDEX]
            self.current_distance = sensor_data
            print(f"Distance: {self.current_distance} cm")
        except (IndexError, TypeError) as e:
            print(f"Error reading distance data: {e}")
            self.current_distance = None

    def on_detect_marker(self, marker_info):
        self.markers.clear()
        for info in marker_info:
            x, y, w, h, data = info
            self.markers.append(MarkerInfo(x, y, w, h, data))

    def find_target_marker(self, target_name):
        for marker in self.markers:
            if marker.text == target_name:
                return marker
        return None

    def get_distance(self):
        return self.current_distance

    def close(self):
        self.sensor.unsub_distance()
        self.camera.stop_video_stream()
        self.ep_robot.close()
