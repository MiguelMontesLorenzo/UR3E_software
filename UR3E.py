
import threading
import sys
import os 

import time
import math
import numpy as np
# import rtde_config as rtde_config
# import rtde as rtde
from libs import rtde_config as rtde_config
from libs import rtde as rtde


# GLOBAL VARIABLES
t0 = time.time()
dir_path = os.path.dirname(os.path.abspath(__file__))
mode_bloq = threading.Semaphore(0)
finished_movement_confirmation = threading.Semaphore(0)
mutex = threading.Semaphore(1)


class UR3EConnection:

    def __init__(self):

        # execution mode: (-1: stop; 0: not communicating; 1: angle control; 2: position control)
        self.MODE = 0
        # due to the choice of continuous speed control instead of the default UR3E movement control it is necesary to keep constant comunication in parallel
        self.t = threading.Thread(target=self.monitor)
        

        # info required for connection
        self.ROBOT_HOST = '10.0.0.150' # ip in settings in the tablet
        self.ROBOT_PORT = 30004

        # communication config file path
        aux_directory_name = 'libs'
        self.config_filename = dir_path + f'/{aux_directory_name}/control_loop_configuration.xml'
        
        # communication object reference
        self.con = None
        # direct multiplier of robot movement speed
        self.gain = 0.6
        # object containing information to be send in each moment
        self.setp = None
        # 
        self.watchdog = None
        # boolean indicating full control over the robot (once it has concluded the initial recolocation)
        self.monitoring = False

        # robot actual joint angles and targeted angles to move towards
        self.angles = None
        self.target_ang = None
        # robot actual joint positions (3D vectors) and targeted positions to move towards (only for position control)
        self.position = None
        self.target_pos = None

        # precision error in reaching targeted values
        self.check = 0
        self.error = 0.01

    def monitor(self):

        # Check the connection is open
        if not self.con.send_start():
            print("connection failed")
            sys.exit()
        else:
            print("connected")

        # 
        init_time = time.time()
        while self.MODE != -1:

            state = self.con.receive()
            if state is None:
                break
            joint_angles = state.actual_q
            position = state.actual_TCP_pose

            # Check if the program is running in the Polyscope
            if not state.output_int_register_0 == 0:

                if self.MODE == 0:
                    self.angles = state.actual_q
                    self.position = state.actual_TCP_pose
                    self.list_to_setp(self.setp, [0,0,0,0,0,0,0])
                    self.con.send(self.setp)
                    finished_movement_confirmation.release()
                    mode_bloq.acquire()


                if self.MODE == 1:

                    # Get UR3 position
                    self.angles = state.actual_q
                    self.position = state.actual_TCP_pose
                    
                    # Compute control error    
                    speeds = self.compute_speed(self.target_ang, state.actual_q)
                    # Compute control effort
                    control_effort = self.compute_control_effort(speeds, self.gain)
                    # Reformat control effort list into setpoint
                    control_effort.append(self.MODE)
                    self.list_to_setp(self.setp, control_effort)
                    # Send new control effort
                    self.con.send(self.setp)
                    self.check = self.check_dif(self.target_ang,self.angles)
                    if self.check == 1:
                        self.MODE = 0
                    

                if self.MODE == 2:

                    # Get UR3 position
                    self.angles = state.actual_q
                    self.position = state.actual_TCP_pose

                    self.target_pos.append(self.MODE)
                    self.list_to_setp(self.setp,self.target_pos) #control_effort
                    # Send new control effort        
                    self.con.send(self.setp)

                    self.check = self.check_dif(self.target_pos,self.position)
                    if self.check == 1:
                        self.MODE = 0



            self.con.send(self.watchdog)
                
    def compute_speed(self, target_angles, joints):

        """Computes a 6D vector containing the error in every joint in the control loop

        Args:
            target (list): List of floats containing the target joint angles in radians
            joints (list): List of floats containing the measured joint angles in radians

        Returns:
            list: List of floats containing the angles error in radians
        """
        # return [target_angle - joints[i] for i,target_angle in enumerate(target_angles)]

        exp = 3/5

        diffs = [0 for _ in range(6)]
        speeds = [0 for _ in range(6)]
        for i,_ in enumerate(speeds):
            diffs[i] = np.round(target_angles[i] - joints[i], 2)
            speeds[i] = np.round((2*np.heaviside(diffs[i],0)-1) * ((abs(diffs[i])/(2*math.pi))**exp ) * (2*math.pi), 2)

        return speeds

    def compute_control_effort(self, error, gain):

        """Computes a 6D vector containing the control effort in every joint in the control loop

        Args:
            error (list): List of floats containing the angles error in radians
            gain (float): Gain in the control loop (each joint angle error will be multiplied times this value to compute control effort)

        Returns:
            liat: List of floats containing the control efforts in each joint
        """
        return [i*gain for i in error]

    def list_to_degrees(self, angles):

        ### Transforma una lista de valores de radianes a grados

        """Converts input list values from radians to degrees

        Args:
            angles (list): List containing angles in radians

        Returns:
            list: List containing angles in degrees
        """
        return [i*360/(2*math.pi) for i in angles]

    def list_to_radians(self, angles):
        """Converts input list values from degrees to radians

        Args:
            angles (list): List containing angles in degrees

        Returns:
            list: List containing angles in radians
        """
        return [i*(2*3.14592)/(360) for i in angles]

    # Reformat setpoint into list
    def setp_to_list(self, setp):
        return [setp.__dict__['input_double_register_%i' % i] for i in range(0,7)]

    # Reformat list into setpoint
    def list_to_setp(self, setp, lst):
        for i in range(0,7):
            setp.__dict__["input_double_register_%i" % i] = lst[i]
        return setp

    def connect2Robot(self):

        # logging.getLogger().setLevel(logging.INFO)

        # configuration files
        conf = rtde_config.ConfigFile(self.config_filename)
        state_names, state_types = conf.get_recipe('state')
        setp_names, setp_types = conf.get_recipe('setp')
        watchdog_names, watchdog_types = conf.get_recipe('watchdog')

        ### self.con.connect() se encarga de crear la conexión con el UR3E
        ### Además si la conexión ya existe, entonces se devuelve un 0, si la conexión no existe todavía no se devuelve nada (None)

        ### En el siguiente fragmento se llama 2 veces al método self.con.connect():
        ### 1 - para crear la conexión
        ### 2 - para recibir un (0) y verificar que la conexión ha sido creada

        # connection to the robot
        self.con = rtde.RTDE(self.ROBOT_HOST, self.ROBOT_PORT)
        self.con.connect()  ###?
        connection_state = self.con.connect()  ### esto devuelve 0 si la conexion está activa

        # check connection
        ### Repetición del bucle hasta el momento en el que se lleva a cabo la conexión satisfactoriamente
        while connection_state != 0:
            time.sleep(5)
            connection_state = self.con.connect()

        self.con.send_output_setup(state_names, state_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)
        self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)

        # Initialize 6 registers which will hold the target angle values
        self.setp.input_double_register_0 = 0.0
        self.setp.input_double_register_1 = 0.0
        self.setp.input_double_register_2 = 0.0
        self.setp.input_double_register_3 = 0.0
        self.setp.input_double_register_4 = 0.0
        self.setp.input_double_register_5 = 0.0
        self.setp.input_double_register_6 = 0.0
        self.setp.input_double_register_7 = 0.0         # control movimiento
        self.setp.input_double_register_8 = 0.0         # control movimiento
        self.setp.input_double_register_9 = 0.0         # control movimiento
        self.setp.input_double_register_10 = 0.0        # control movimiento
        self.setp.input_double_register_11 = 0.0        # control movimiento
          
        # The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
        self.watchdog.input_int_register_0 = 0

        mutex.acquire()
        self.t.start()
        finished_movement_confirmation.acquire()
        mutex.release()
        self.monitoring = True
        print('robot ready')
        print('monitoring ...')

        return 0


    def change_mode(self, n):
        self.MODE = n
        return 0

    def check_dif(self, targ, actual):
        ch = 0
        for i in range(len(actual)):
            if abs(targ[i]-actual[i])/(2*math.pi) < self.error :
                ch = 1
            else :
                ch = 0
                break
        return ch

    def go_pos(self, position):
        self.check  = 0
        mutex.acquire()
        self.target_pos = position
        self.MODE = 2
        mode_bloq.release()
        finished_movement_confirmation.acquire()
        mutex.release()

    def go_ang(self, angles):
        self.check  = 0
        mutex.acquire()
        self.target_ang = angles
        self.MODE = 1
        mode_bloq.release()
        finished_movement_confirmation.acquire()
        mutex.release()

    def get_ang(self):
        return self.angles

    def get_pos(self):
        return self.position

    def stop(self):
        self.MODE = -1
        self.t.join()
        self.list_to_setp(self.setp, [0,0,0,0,0,0,0])
        self.con.send(self.setp)
        self.con.disconnect()
        return 0






class RobotOrders:

    def __init__(self):

        # Initialize the message vector and the control mode
        self.control_info = [0, 0, 0, 0, 0, 0, 0]

        # Set the communication
        self.Communication = UR3EConnection()
        self.confirmation = self.Communication.connect2Robot()
        # self.confirmation = self.Communication.go_ang([0,0,0,0,0,0])
        return

    def moveCoords(self, coordinates):  
        # Coordinates in mm
        # !!! coordinates must be 6x3 array
        self.Communication.go_pos(coordinates)
        # Recive the new robot coordinates
        recived_pose = self.Communication.get_pos()
        # Check if the robot pose is within the tolerance range
        if recived_pose <= coordinates + 1 or recived_pose >= coordinates - 1:
            # Pose okey
            check = 1
        else:
            # Wrong pose
            check = 0
        return check

    def moveJoints(self, angles):
            
        # send the order
        self.Communication.go_ang(angles)
        # Recive the new orientation
        joint_values = self.Communication.get_ang()
        jv_high = [item + 0.5 for item in angles]
        jv_low = [item - 0.5 for item in angles]

        # Comprobation of joints
        if joint_values > jv_low and joint_values < jv_high:
        # Joints' orientation okey
            check = 1
        else:
            # Wrong joints' orientation
            check = 0

        return check

    def ask4RobotJoints(self):
        return self.Communication.get_ang()

    def ask4AbsPosition(self):
        return self.Communication.get_pos()
    

# NOTAS:
# se inicializa la clase cuando se pide la confirmación
# El rango de movimiento permitido por el brazo robótico es de (-2*pi, 2*pi) (al menos para la articulación de la base)