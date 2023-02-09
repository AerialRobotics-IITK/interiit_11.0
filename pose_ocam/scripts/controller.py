#!/usr/bin/env python3
import math
import time
import rospy
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from protocol import Protocol
from env import *
from multiprocessing import shared_memory
import os
from std_msgs.msg import Int32
from CppPythonSocket.server import Server

import socket

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)

class pidcontroller:
    def __init__(
        self,
        pos_sub_topic,
        server_port,
        client_port,
        pose_port=7000,
        kp=[4, 4, 2.7],
        kd=[5, 5, 2.15],
        ki=[0.05, 0.05, 1.1],
        eqb_thrust=1550,
        IP=IP,
        PORT=PORT,
        drone_num=1
    ):
        self.temp = np.array([0,], dtype = np.int32)
        self.escape = 0
        self.oth = 0
        self.drone_num = drone_num
        self.talker = Protocol(IP, PORT)
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.tol = 10
        self.prev_time = [0.0, 0.0, 0.0]
        self.error_tol = 0.01
        self.prev_error = [0.0, 0.0, 0.0]
        self.e_i = [0.0, 0.0, 0]
        self.e_d = [0.0, 0.0, 0.0]
        self.vel = np.array([0.0, 0.0, 0.0])
        self.curr_pos = [0.0, 0.0, 0.0]
        self.curr_attitude = [0, 0, 0, 0]
        self.equilibrium_thrust = eqb_thrust
        self.b3d = np.array([0.0, 0.0, 0.0])
        self.re3 = np.array([0.0, 0.0, 0.0])
        self.pose_sub_topic = pos_sub_topic
        self.timer_start = None

        # server/client thing
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((HOST, server_port))
        print("waiting for connections")
        server.listen()
        # sleep for 2 seconds to start server
        time.sleep(2)
        self.conn, self.addr = server.accept()
        print(f"Connected to {addr}")
        self.client = server.connect((HOST, client_port))
        self.pose_client = server.connect((HOST, pose_port))


        os.makedirs(os.path.join(LOG_FOLDER_PATH, "controller"), exist_ok=True)
        self.LOGFILE = os.path.join(
            LOG_FOLDER_PATH, f"controller/d1_{int(time.time())}.log"
        )
        self.log_file = open(self.LOGFILE, "w")
        self.start_time = time.time()
        print(
            "time,e_d_x,e_p_x,e_i_x,e_d_y,e_p_y,e_i_y,e_d_z,e_p_z,e_i_z,roll,pitch,yaw,thrust,x,y,z",
            file=self.log_file,
        )
        # self.pub = rospy.Publisher(f'esc{self.drone_num}', Int32, queue_size=1000)

    def listener(self):
        # rospy.Subscriber(self.pose_sub_topic, numpy_msg(Floats), self.callback)
        message = self.server.receive()
        message = message.split(",")
        print(message)
        message = np.array([np.float64(x) for x in message])

        if message[0] == -1000:
            if self.prev_pose[0] == -1000:
                continue
            self.curr_pos = self.prev_pose
        else:
            self.curr_pos = message
        self.curr_pos[0] = self.x_filter.predict_kalman(self.curr_pos[0])
        self.curr_pos[1] = self.y_filter.predict_kalman(self.curr_pos[1])
        self.curr_pos[2] = self.z_filter.predict_kalman(self.curr_pos[2])

    def listen(self):
        msg = client.read(1024).decode('utf-8')
        self.oth = int(msg)
        # self.cally(int(msg))
        # rospy.Subscriber('esc1', Int32, self.cally)
        
    def cally(self, msg):
        self.oth = msg

    def callback(self, msg):

        self.curr_pos[0] = msg.data[0]
        self.curr_pos[1] = msg.data[1]
        self.curr_pos[2] = msg.data[2]

    def clip(self, parameter, low, high):

        if parameter < low:
            parameter = low
        elif parameter > high:
            parameter = high
        return parameter

    def talker_pub(self, roll, pitch, yaw, thrust):

        pitch = 1500 + pitch * (1800 / pi)
        pitch = self.clip(pitch, 1400, 1600)
        roll = 1500 + roll * (1800 / pi)
        roll = self.clip(roll, 1400, 1600)
        thrust = self.clip(thrust, 1480, 2099)
        yaw = 1510
        print(
            roll,
            ",",
            pitch,
            ",",
            yaw,
            ",",
            thrust,
            ",",
            self.curr_pos[0],
            ",",
            self.curr_pos[1],
            ",",
            self.curr_pos[2],
            file=self.log_file,
        )
        self.talker.set_RPY_THR(int(roll), int(pitch), int(yaw), int(thrust))

    def calc_error(self, i, error):

        curr_time = time.time()
        dt = 0.0
        if self.prev_time[i] != 0.0:
            dt = curr_time - self.prev_time[i]
        de = error - self.prev_error[i]
        e_p = error
        if self.prev_time[i] != 0:
            self.e_i[i] += error * dt
        self.e_d[i] = 0
        if dt > 0:
            self.e_d[i] = de / dt
        self.prev_time[i] = curr_time
        self.prev_error[i] = error
        if i == 0:
            print(time.time() - self.start_time, end=",", file=self.log_file)
        print(
            np.round(self.e_d[i], 0),
            ",",
            e_p,
            ",",
            self.e_i[i],
            end=",",
            file=self.log_file,
        )
        return (
            (self.kp[i] * e_p) + (self.kd[i] * self.e_d[i]) +
            (self.ki[i] * self.e_i[i])
        )

    def pos_change(self, targ_pos=([0, 0, 0])):

        errors = (
            targ_pos[0] - self.curr_pos[0],
            targ_pos[1] - self.curr_pos[1],
            targ_pos[2] - self.curr_pos[2],
        )
        for i in range(len(errors)):
            self.vel[i] = self.calc_error(i, errors[i])
        self.vel[2] += self.equilibrium_thrust
        for i in range(3):
            self.b3d[i] = self.vel[i]
        self.b3d = self.b3d / np.linalg.norm(self.b3d)
        self.curr_attitude[0] = math.atan(self.b3d[0] / self.b3d[2])
        self.curr_attitude[1] = (-1) * math.asin(self.b3d[1])

        if self.curr_attitude[0] > 1:
            self.curr_attitude[0] = 1
        if self.curr_attitude[0] < -1:
            self.curr_attitude[0] = -1
        if self.curr_attitude[1] > 1:
            self.curr_attitude[1] = 1
        if self.curr_attitude[1] < -1:
            self.curr_attitude[1] = -1

        self.re3[0] = (
            (-1) * math.cos(self.curr_attitude[0]
                            ) * math.sin(self.curr_attitude[1])
        )
        self.re3[1] = math.sin(self.curr_attitude[0])
        self.re3[2] = math.cos(self.curr_attitude[0])
        self.curr_attitude[3] = np.linalg.norm(self.vel)

        self.talker_pub(
            self.curr_attitude[0],
            self.curr_attitude[1],
            self.curr_attitude[2],
            self.curr_attitude[3],
        )

    def autopilot(self, targ_pos, duration):

        self.start = time.time()
        self.listener()
        start = time.time()
        # r = rospy.Rate(CONTROLLER_RATE)
        while time.time() - start < duration:
            self.listener()
            if (max([abs(targ_pos[0] - self.curr_pos[0]),abs(targ_pos[1] - self.curr_pos[1]),])) < self.tol:
                print(f"Drone{self.drone_num} Position Reached!!")
            self.pos_change(targ_pos)
            r.sleep()

    def concurrent_autopilot(self, targ_pos, duration):

        self.start = time.time()
        self.listener()
        self.escape = 0
        start = time.time()
        r = rospy.Rate(CONTROLLER_RATE)
        exit_cond = False
        count = 0

        while not rospy.is_shutdown():
            self.listener()
            self.listen()
            self.pub.publish(self.escape)
            if time.time() - start > duration and count == 0:
                exit_cond = True
                count = 1
                print(f"Drone{self.drone_num} Time Reached!!")
            if (max([abs(targ_pos[0] - self.curr_pos[0]),abs(targ_pos[1] - self.curr_pos[1]),])) < self.tol and (abs(targ_pos[2] - self.curr_pos[2]) < 25) and count == 0:
                exit_cond = True
                count = 1
                print(f"Drone{self.drone_num} Position Reached!!")
            if exit_cond and count==1:
                count = 2
                if self.escape == 0:
                    self.escape = 1
            if exit_cond and self.oth == 1 and self.timer_start is None:
                print(f"Drone{self.drone_num} escape is {self.escape}")
                self.pub.publish(self.escape)
                self.timer_start = time.time()
            if (self.timer_start is not None) and (time.time() - self.timer_start > 1):
                print("Deadlock broken!!")
                self.timer_start = None 
                break
            self.pos_change(targ_pos)
            r.sleep()