import UR3E
import time


if __name___ == '__main__':

    # instanciate Robot Control Object
    robot_control = UR3E.RobotOrders()
    # move joints
    new_angles = [0.5 for _ in range(6)]
    check = robot_control.moveJoints(new_angles)
    # wait for the movement to be completed
    while not robot_control.Comunication.MODE == 0:
        time.sleep(1)
        print('time:', time.time(), 'robot joints:', robot_control.ask4RobotJoints())
    # get new coordinates
    robot_angles = robot_control.ask4RobotJoints()
    print('final robot position:', robot_angles)