
from . import states

class StateMachine:
    def __init__(self, robot, detector):
        self.robot = robot
        self.detector = detector
        self.states = {
            "search": states.SearchState(robot, detector),
            "align": states.AlignState(robot, detector),
            "approach": states.ApproachState(robot, detector),
            "grab": states.GrabState(robot, detector),
            "find_marker": states.FindMarkerState(robot, detector),
            "release": states.ReleaseState(robot, detector),
        }
        self.current_state = None

    def set_initial_state(self, state_name):
        self.current_state = self.states[state_name]
        self.current_state.on_enter()

    def run(self):
        while True:
            next_state_name = self.current_state.execute()

            if next_state_name not in ["searching", "aligning", "approaching", "finding_marker"]:
                self.current_state.on_exit()
                if next_state_name == "success" or next_state_name == "failure":
                    print(f"任务结束，状态: {next_state_name}")
                    break
                self.set_initial_state(next_state_name)
