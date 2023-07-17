import UR3E
import time
import math
import numpy as np
import sys



p0 = np.array([0, -math.pi/2, 0, -math.pi/2, 0, 0])
p1 = np.array([math.pi/2, -math.pi/2, 0, -math.pi/2, 0, 0])
p2 = np.array([math.pi, -math.pi, 0, -math.pi/2, 0, 0])



if __name__ == '__main__':
    

    # initialize control class
    robot_control = UR3E.RobotOrders()

    input('wait for polyscope entry point ... then press enter')


    # movement initial timestamp
    t0 = time.time()


    #TEST MOVEMENTS
    # full turn
    for i in range(3):
        for j in range(4):
            # print(i,j, '     ', np.round([j*math.pi/2, -math.pi/2 - i*math.pi/4, 0, -math.pi/2, 0, 0], 2))
            check = robot_control.moveJoints(np.array([j*math.pi/2, -math.pi/2 - i*math.pi/4, 0, -math.pi/2, 0, 0]))
    # exceed max degrees
    # for i in range(6):
    #     check = robot_control.moveJoints(np.array([i*math.pi/2, -math.pi/2, 0, -math.pi/2, 0, 0]))
    # exceed min degrees
    # for i in range(6):
    #     check = robot_control.moveJoints(np.array([-i*math.pi/2, -math.pi/2, 0, -math.pi/2, 0, 0]))



    # TEST INACTIVITY TOLERANCE:
    # for i in range(4):
    #     print(f'delay of: {10*i} seconds')
    #     time.sleep(10*i)
    #     check = robot_control.moveJoints(p1)

    #     print(f'delay of: {10*i + 5} seconds')
    #     time.sleep(10*i + 5)
    #     check = robot_control.moveJoints(p0)



    # get new coordinates
    robot_angles = robot_control.ask4RobotJoints()
    print('final robot joint angles:', np.round(robot_angles, 3), '  time:', time.time() - t0)
    robot_terminal_position = robot_control.ask4AbsPosition()
    print('final robot terminal position:', np.round(robot_terminal_position, 3))
    sys.exit(-1)

