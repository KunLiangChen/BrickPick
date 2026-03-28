
import robomaster
from robomaster import robot
from src.robot_control.robot import Robot
from src.vision.detector import Detector
from src.state_machine.state_machine import StateMachine

def main():
    ep_robot = robot.Robot()
    robot_controller = Robot(ep_robot)
    detector = Detector()

    try:
        robot_controller.initialize()
        state_machine = StateMachine(robot_controller, detector)
        state_machine.set_initial_state("search")
        state_machine.run()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        robot_controller.close()

if __name__ == '__main__':
    main()
