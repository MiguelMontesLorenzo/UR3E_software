#!/usr/bin/env python3
import socket
import os
import numpy as np
import shutil
import math


class CommunicationClass:
    # Class constructor
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.confirm = 1
        return

    # Establish the communication
    def connect2Robot(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Creates a socket object
        print(self.s)
        # Start connection
        self.s.connect((HOST, PORT))
        # Confirmation message
        self.s.sendall(b"0;0;0;0;0;0;1;")
        print(self.s)
        # Confirmation message from robot
        data = self.s.recv(1024)
        print("Received", repr(data))
        # Check the connection
        if data != b"Si":
            print("Connection failed")
            self.confirm = 1
        else:
            print("Connection started")
            self.confirm = 0
        print(self.s)

        return self.confirm

    # Send messages
    def sendMessage(self, message):
        # Message is a list
        # print(self.s)
        str_message = "".join([str(item) + ";" for item in message])
        # print("Sending to the robot: ", str_message)
        # print(bytes(str_message, "ascii"))
        self.s.sendall(bytes(str_message, "ascii"))

    def recvMessage(self):
        data = self.s.recv(1024)
        # print("Received", repr(data))
        data_proc = data.split(b";")
        data_proc.pop()
        # print(data_proc)
        # Check the connection
        vec = [float(item.decode("UTF-8")) for item in data_proc]
        # print("Received from the robot:", vec)
        return vec


# class YoloDetection:
#     def __init__(self):
#         # Maximum number of positionsclass YoloDetection:
#     def __init__(self):
#         # Maximum number of positions to move the piece in the x axis
#         self.x_max = 10
#         # Maximum number of positions to move the piece in the y axis
#         self.y_max = 5
#         # Distance (mm) from one potition to the adjacent in one axis
#         self.step = 15.25
#         self.x_yolo = 0.8
#         self.y_yolo = 0.5
#         self.x_mm = 300
#         self.y_mm = 200
#         # Pixeles de imagen que no representan area de trabajo en X
#         self.x0 = 100
#         # Pixeles de imagen que no representan area de trabajo en Y
#         self.y0 = 100
#         # Camera source
#         self.source = 0

#     def AskForPiecePosition_Yolo(self):
#         # Checks if exp directory already exists and eliminate it
#         exp_path = os.path.join(PATH, "./yolov5/runs/detect/exp")
#         if os.path.isdir(exp_path):
#             shutil.rmtree(exp_path)
#         # Executes detect.py
#         yolo_path = os.path.join(PATH, "yolov5")
#         os.chdir(yolo_path)
#         os.system(f"python detect.py --source {self.source} --conf-thres 0.80 --save-txt --save-conf --weights {PATH}\YOLO.pt")
#         # Reads current.txt to obtain the position vector
#         txt_path = os.path.join(PATH, "yolov5/runs/detect/exp/labels/current.txt")
#         f = open(txt_path, "r")
#         message = f.read()
#         # Process the txt
#         dataset = []
#         temp1 = message.split("\n")
#         if temp1[-1] == "":
#             temp1.pop()
#         # Vec: [object-class-ID, X center, Y center, Box width, Box height, Prob]
#         for n in range(len(temp1)):
#             temp2 = temp1[n].split(" ")
#             vec = []
#             for m in range(len(temp2)):
#                 vec.append(float(temp2[m]))
#             dataset.append(vec)
#         f.close()
#         dataset = np.array(dataset)
#         # Analyses standard deviations and mean values
#         final_pos = [0, 0, 0, 0, 0, 0, 0]
#         std_dev_list = np.std(dataset.T[1:5, :], axis=1)
#         final_pos = np.mean(dataset.T[1:3, :], axis=1)
#         width_list = np.mean(dataset.T[3:5, :], axis=1)
#         # Erase the directory that contains the txt
#         os.chdir(PATH)
#         exp_path = os.path.join(PATH, "./yolov5/runs/detect/exp")
#         shutil.rmtree(exp_path)
#         # Transform from yolo units to mm
#         final_pos[0] = (final_pos[0] - self.x0) * (self.x_mm / width_list[0])
#         final_pos[1] = (final_pos[1] - self.y0) * (self.y_mm / width_list[1])
#         # Another posible option
#         """
#         final_pos[0] = (final_pos[0] - x0) * (x_mm / x_yolo)
#         final_pos[1] = (final_pos[1] - y0) * (y_mm / y_yolo)
#         """
#         return final_pos to move the piece in the x axis
#         self.x_max = 10
#         # Maximum number of positions to move the piece in the y axis
#         self.y_max = 5
#         # Distance (mm) from one potition to the adjacent in one axis
#         self.step = 15.25
#         self.x_yolo = 0.8
#         self.y_yolo = 0.5
#         self.x_mm = 300
#         self.y_mm = 200
#         # Pixeles de imagen que no representan area de trabajo en X
#         self.x0 = 100
#         # Pixeles de imagen que no representan area de trabajo en Y
#         self.y0 = 100
#         # Camera source
#         self.source = 0

#     def AskForPiecePosition_Yolo(self):
#         # Checks if exp directory already exists and eliminate it
#         exp_path = os.path.join(PATH, "./yolov5/runs/detect/exp")
#         if os.path.isdir(exp_path):
#             shutil.rmtree(exp_path)
#         # Executes detect.py
#         yolo_path = os.path.join(PATH, "yolov5")
#         os.chdir(yolo_path)
#         os.system(f"python detect.py --source {self.source} --conf-thres 0.80 --save-txt --save-conf --weights {PATH}\YOLO.pt")
#         # Reads current.txt to obtain the position vector
#         txt_path = os.path.join(PATH, "yolov5/runs/detect/exp/labels/current.txt")
#         f = open(txt_path, "r")
#         message = f.read()
#         # Process the txt
#         dataset = []
#         temp1 = message.split("\n")
#         if temp1[-1] == "":
#             temp1.pop()
#         # Vec: [object-class-ID, X center, Y center, Box width, Box height, Prob]
#         for n in range(len(temp1)):
#             temp2 = temp1[n].split(" ")
#             vec = []
#             for m in range(len(temp2)):
#                 vec.append(float(temp2[m]))
#             dataset.append(vec)
#         f.close()
#         dataset = np.array(dataset)
#         # Analyses standard deviations and mean values
#         final_pos = [0, 0, 0, 0, 0, 0, 0]
#         std_dev_list = np.std(dataset.T[1:5, :], axis=1)
#         final_pos = np.mean(dataset.T[1:3, :], axis=1)
#         width_list = np.mean(dataset.T[3:5, :], axis=1)
#         # Erase the directory that contains the txt
#         os.chdir(PATH)
#         exp_path = os.path.join(PATH, "./yolov5/runs/detect/exp")
#         shutil.rmtree(exp_path)
#         # Transform from yolo units to mm
#         final_pos[0] = (final_pos[0] - self.x0) * (self.x_mm / width_list[0])
#         final_pos[1] = (final_pos[1] - self.y0) * (self.y_mm / width_list[1])
#         # Another posible option
#         """
#         final_pos[0] = (final_pos[0] - x0) * (x_mm / x_yolo)
#         final_pos[1] = (final_pos[1] - y0) * (y_mm / y_yolo)
#         """
#         return final_pos


class RobotOrders:
    def __init__(self):
        # Initialize the message vector and the order index
        self.message = [0, 0, 0, 0, 0, 0, 0]
        self.order = 0
        # Set the communication
        self.Communication = CommunicationClass()
        self.confirmation = self.Communication.connect2Robot()
        return

    def moveCoords(self, coordinates):
        # Coordinates in mm
        # Fill in the last position of the message with the order index
        self.order = 0
        self.message[-1] = self.order
        # Move Coords
        # print("Moving robot position according to coordinates")
        self.message[0:3] = coordinates[:]
        # Send the message
        self.Communication.sendMessage(self.message)
        # Recive the new robot coordinates
        recived_pose = self.Communication.recvMessage()
        # Check if the robot pose is within the tolerance range
        if recived_pose <= coordinates + 1 or recived_pose >= coordinates - 1:
            # Pose okey
            check = 1
        else:
            # Wrong pose
            check = 0
        return check

    # def moveFromYolo(self):
    #     Yolo = YoloDetection()
    #     self.message = Yolo.AskForPiecePosition_Yolo()
    #     # Fill in the last position of the message with the order index
    #     self.order = 0
    #     self.message[-1] = self.order
    #     # Send the message
    #     self.Communication.sendMessage(self.message)
    #     # Recive the relative robot position
    #     rel_position = self.Communication.recvMessage()
    #     return rel_position

    def moveJoints(self, angles):
        # Degrees in radians
        # Fill in the last position of the message with the order index
        self.order = 1
        self.message[-1] = self.order
        # Move joints
        # print("Moving robot joints (degrees)")
        # angles_deg = [round(math.degrees(angles[i])) for i in range(len(angles))]
        self.message[0:6] = angles
        # Send the message
        self.Communication.sendMessage(self.message)
        # Recive the new orientation
        joint_values = self.Communication.recvMessage()
        jv_high = [item + 0.5 for item in self.message]
        jv_low = [item - 0.5 for item in self.message]
        # Comprobation of joints
        if joint_values > jv_low and joint_values < jv_high:
            # Joints' orientation okey
            check = 1
        else:
            # Wrong joints' orientation
            check = 0
        return check

    def ask4RobotJoints(self):
        # Fill in the last position of the message with the order index
        self.order = 2
        self.message[-1] = self.order
        # Send the message
        self.Communication.sendMessage(self.message)
        # Recive the joints' orientation
        joint_values = self.Communication.recvMessage()
        return joint_values

    def ask4AbsPosition(self):
        # Fill in the last position of the message with the order index
        self.order = 3
        self.message[-1] = self.order
        # Send the message
        self.Communication.sendMessage(self.message)
        # Recive the joints' orientation
        abs_position = self.Communication.recvMessage()
        return abs_position

    # def ask4RelativePosition(self):
    #     # Ask for relative position to top left corner of YOLO img
    #     # Fill in the last position of the def moveFromYolo(self):
    #     Yolo = YoloDetection()
    #     self.message = Yolo.AskForPiecePosition_Yolo()
    #     # Fill in the last position of the message with the order index
    #     self.order = 4
    #     self.message[-1] = self.order
    #     # Send the message
    #     self.Communication.sendMessage(self.message)
    #     # Recive the relative robot position
    #     rel_position = self.Communication.recvMessage()
    #     return rel_position

    # def pickPlace_target(self, old_position, new_position):
    #     # Fill in the last position of the message with the order index
    #     self.order = 1
    #     self.message[-1] = self.order
    #     # Move Coords
    #     # print("Moving robot position to old target position")
    #     self.message[0:3] = old_position[:]
    #     # Send the message with the old target position
    #     self.Communication.sendMessage(self.message)
    #     # Recive the new robot coordinates
    #     _ = self.Communication.recvMessage()
    #     # Pick the piece
    #     self.order = 5
    #     self.message[-1] = self.order
    #     # print("Picking the piece")
    #     self.Communication.sendMessage(self.message)
    #     # Recive the confirmation
    #     _ = self.Communication.recvMessage()
    #     # Place the target in the new position
    #     self.order = 0
    #     self.message[-1] = self.order
    #     # Move Coords
    #     # print("Moving robot position to new target position")
    #     self.message[0:3] = new_position[:]
    #     # Send the message with the new target position
    #     self.Communication.sendMessage(self.message)
    #     # Recive the new robot coordinates
    #     _ = self.Communication.recvMessage()
    #     # Place the piece
    #     self.order = 6
    #     self.message[-1] = self.order
    #     # print("Placing the piece")
    #     self.Communication.sendMessage(self.message)
    #     # Recive the confirmation
    #     _ = self.Communication.recvMessage()
    #     return
