#!/usr/bin/env python3
import math
import time
import os

import rospy
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from protocol import Protocol

from params import *

os.makedirs(os.path.join(LOG_FOLDER_PATH, "controller"), exist_ok=True)
LOGFILE = os.path.join(LOG_FOLDER_PATH, f"controller/{int(time.time())}.txt")
log_file = open(LOGFILE, 'w')
start_time = time.time()


class pidcontroller:
    def __init__(self, kp=[2.5, 2.5, 5], kd=[4, 4, 4.5], ki=[0.001, 0.001, 0], eqb_thrust=1500, IP="192.168.4.1", PORT=23):
        self.talker = Protocol(IP, PORT)
        self.kp = kp
        self.kd = kd
        self.ki = ki
        # self.m = 1.5
        # self.g = 9.817
        # self.breaker = True
        self.prev_time = [0.0, 0.0, 0.0]
        self.error_tol = 0.01
        self.prev_error = [0.0, 0.0, 0.0]
        self.e_i = [0.0, 0.0, 0]
        self.e_d = [0.0, 0.0, 0.0]
        self.vel = np.array([0.0, 0.0, 0.0])
        self.speed = 1
        self.repeat_pilot = False
        self.single_point_error = 0.001
        self.multiple_point_error = 0.2  # 0.1
        self.ang_error = 0.05  # 0.02
        self.length = 1
        self.curr_pos = [0.0, 0.0, 0.0]
        self.curr_attitude = [0, 0, 0, 0]
        self.equilibrium_thrust = eqb_thrust

        # need to do something about thrust
        self.equilibrium_attitude = [0, 0, 0, 15]
        self.b3d = np.array([0.0, 0.0, 0.0])
        self.re3 = np.array([0.0, 0.0, 0.0])
        self.time = 0.0

    def callback(self, msg):
        self.curr_pos[0] = msg.data[0]
        self.curr_pos[1] = msg.data[1]
        self.curr_pos[2] = msg.data[2]

    def talker_pub(self, roll, pitch, yaw, thrust):
        pitch = 1500 + pitch*(1800/pi)
        # pitch=np.clip(pitch, 1200, 1800)
        if pitch < 1200:
            pitch = 1200
        elif pitch > 1800:
            pitch = 1800
        roll = 1500 + roll*(1800/pi)
        # roll=np.clip(roll, 1200,2000)
        if roll < 1200:
            roll = 1200
        elif roll > 1800:
            roll = 1800
        # print(thrust)

        if thrust > 2099:
            thrust = 2099
        elif thrust < 1000:
            thrust = 1000
        yaw = 1500
        # thrust = np.clip(thrust, 1200,1800)
        # print(roll, pitch, yaw, thrust)
        # pub = rospy.Publisher('/firefly/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=10)
        # while not rospy.is_shutdown():
        # pub.publish(p)
        self.talker.set_RPY_THR(int(roll), int(pitch), int(yaw), int(thrust))

    def listener(self):
        rospy.Subscriber("position", numpy_msg(Floats), self.callback)

    def calc_error(self, i, error):
        # print(i) # Calculates the error, its `derivative' and `integral'
        curr_time = time.time()
        dt = 0.0
        if self.prev_time[i] != 0.0:
            dt = curr_time - self.prev_time[i]
        de = error - self.prev_error[i]
        e_p = error
        if self.prev_time[i] != 0:
            self.e_i[i] += error*dt
        self.e_d[i] = 0
        if dt > 0:
            self.e_d[i] = de/dt

        self.prev_time[i] = curr_time
        self.prev_error[i] = error
        # print("e_i", self.e_i[i])
        # print('dt', dt)
        # if i == 2:
        # print(self.e_d[i], ",", e_p, end=",")  # printing e_d
        # print("ep:",e_p,", ei:",self.e_i[i],", ed:",self.e_d[i])

        return (self.kp[i] * e_p) + (self.kd[i]*self.e_d[i]) + (self.ki[i]*self.e_i[i])

    # Corrects only position
    def pos_change(self, targ_pos=([0, 0, 0]), curr_pos=([0, 0, 0])):
        # if not self.breaker:
        # print("curr_pose: ",curr_pos[2])
        print(
            f"{time.time() - start_time}, {curr_pos[0]}, {curr_pos[1]}, {curr_pos[2]}", file=log_file)

        errors = (targ_pos[0] - curr_pos[0], targ_pos[1] -
                  curr_pos[1], targ_pos[2] - curr_pos[2])
        for i in range(len(errors)):
            self.vel[i] = self.calc_error(i, errors[i])
        self.vel[2] += self.equilibrium_thrust
        # if max(errors) > self.error_tol or min(errors) < -self.error_tol:
        for i in range(3):
            self.b3d[i] = self.vel[i]
        self.b3d = self.b3d / np.linalg.norm(self.b3d)
        self.curr_attitude[0] = math.atan(self.b3d[0] / self.b3d[2])
        self.curr_attitude[1] = (-1)*math.asin(self.b3d[1])

        if self.curr_attitude[0] > 1:
            self.curr_attitude[0] = 1
        if self.curr_attitude[0] < -1:
            self.curr_attitude[0] = -1
        if self.curr_attitude[1] > 1:
            self.curr_attitude[1] = 1
        if self.curr_attitude[1] < -1:
            self.curr_attitude[1] = -1

        self.re3[0] = (-1)*math.cos(self.curr_attitude[0]) * \
            math.sin(self.curr_attitude[1])
        self.re3[1] = math.sin(self.curr_attitude[0])
        self.re3[2] = (math.cos(self.curr_attitude[0]))

        self.curr_attitude[3] = np.linalg.norm(self.vel)
        # print("error:", errors)
        # print(self.curr_attitude[3])
        # if(curr_attitude[3]+self.speed*self.vel[2]<0):
        #     curr_attitude[3]=0
        # print("X")
        # print('cosalphacosbeta', self.re3[2])
        # print('thrust', curr_attitude[3], 'error', errors[2])
        # print()
        # print('roll', curr_attitude[0], 'error', errors[1])
        # print()
        # print('pitch', curr_attitude[1], 'error', errors[0])
        # print()

        # curr_attitude[0]=0#########3
        # curr_attitude[1]=0######3
        # self.talker.set_RPY_THR(curr_attitude[0], curr_attitude[1], curr_attitude[2], curr_attitude[3]+self.speed*self.vel[2])
        self.talker_pub(self.curr_attitude[0], self.curr_attitude[1],
                        self.curr_attitude[2], self.curr_attitude[3])
        # Thrust will be changed by self.speed*self.vel[2]
        # Roll will be changed by self.speed*self.vel[0]
        # Pitch will be changed by self.speed*self.vel[1]
        # self.reach_pose = False
        # else:
        #     self.reach_pose = True
        #     print ("reached destination coordinates")
        #     print ("error:",np.linalg.norm(errors))

    def autopilot(self, targ_pos, duration):

        self.start = time.time()
        # self.curr_pos and orientation needs to be read
        self.listener()
        if self.curr_pos != targ_pos:
            self.reach_pose = False
        start = time.time()
        # while not self.reach_pose:
        r = rospy.Rate(30)
        while not rospy.is_shutdown() and time.time() - start < duration:
            self.listener()
            self.pos_change(targ_pos, self.curr_pos)
            r.sleep()


print("e_d,e_p,thrust", file=log_file)
