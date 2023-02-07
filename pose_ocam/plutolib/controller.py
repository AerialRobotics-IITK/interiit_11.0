import math
import time
import os
import numpy as np
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from plutolib.protocol import Protocol
from plutolib.logger import Logger
from CppPythonSocket.server import Server
from plutolib.utils import Filter


class pidcontroller:
    def __init__(
        self,
        kp: list = [2.5, 2.5, 5],
        kd: list = [4, 4, 4.5],
        ki: list = [0.001, 0.001, 0.5],
        eqb_thrust: int = 1500,
        publishing_rate: int = 35,
        roll_clip=100,
        pitch_clip=100,
        thrust_clip=(1480, 2050),
        env: dict = None,
        pose_topic="position",
    ):
        """Initialise a PID Controller

        Args:
            kp (list, optional): proporitional gains in roll, pitch, thrust. Defaults to [2.5, 2.5, 5].
            kd (list, optional): derivative gains in roll, pitch, thrust. Defaults to [4, 4, 4.5].
            ki (list, optional): integral gains in roll, pitch, thrust. Defaults to [0.001, 0.001, 0.5].
            eqb_thrust (int, optional): Equilibrium thrust of the UAV. Defaults to 1500.
            publishing_rate (int, optional): Publishing to the UAV. Defaults to 35.
            env (dict, optional): environment variables containing IP, PORT, LOG_FOLDER_PATH, VERBOSE. Defaults to default_dict.
        """
        default_env = {
            "IP": "192.168.4.1",
            "PORT": 23,
            "LOG_FOLDER_PATH": "~/pluto_logs",
            "VERBOSE": False,
            "LOG_FOLDER": "controller",
            "SERVER_PORT": 5002,
        }
        if env == None:
            env = default_env
        else:
            for key in default_env.keys():
                if env.get(key, None) == None:
                    env[key] = default_env[
                        key
                    ]  # allot missing keys to env from default_env
        self.pose_topic = pose_topic
        self.tol = 5
        self.start_time = time.time()
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.pitch_clip = pitch_clip
        self.roll_clip = roll_clip
        self.thrust_clip = thrust_clip
        self.prev_time = [0.0] * 3
        self.prev_error = [0.0] * 3
        self.e_i = [0.0] * 3
        self.e_d = [0.0] * 3
        self.vel = np.array([0.0] * 3)
        self.curr_pos = [0.0] * 3
        self.curr_attitude = [0] * 4
        self.equilibrium_thrust = eqb_thrust
        self.b3d = np.array([0.0] * 3)
        self.re3 = np.array([0.0] * 3)
        self.publishing_rate = publishing_rate
        self.talker = Protocol(env["IP"], env["PORT"])
        self.verbose = env["VERBOSE"]
        self.x_filter = Filter()
        self.y_filter = Filter()
        self.z_filter = Filter()

        self.server = Server("127.0.0.1", env["SERVER_PORT"])

        if self.verbose:
            self.logger = Logger(env["LOG_FOLDER_PATH"], env["LOG_FOLDER"])
            self.logger.print(
                "time,e_d_x,e_p_x,e_i_x,e_d_y,e_p_y,e_i_y,e_d_z,e_p_z,e_i_z,roll,pitch,yaw,thrust,x,y,z",
                init=True,
            )

    def talker_pub(self, roll, pitch, yaw, thrust):
        pitch = 1500 + pitch * (1800 / np.pi)
        pitch = np.clip(pitch, 1500 - self.pitch_clip, 1500 + self.pitch_clip)
        roll = 1500 + roll * (1800 / np.pi)
        roll = np.clip(roll, 1500 - self.roll_clip, 1500 + self.roll_clip)
        thrust = np.clip(thrust, np.min(self.thrust_clip), np.max(self.thrust_clip))
        yaw = 1500  # hardcoded

        if self.verbose:
            self.logger.print(
                roll,
                pitch,
                yaw,
                thrust,
                self.curr_pos[0],
                self.curr_pos[1],
                self.curr_pos[2],
                comma_seperated=True,
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

        if i == 0 and self.verbose:
            self.logger.print(time.time() - self.start_time, end=",")
        if self.verbose:
            self.logger.print(
                np.round(self.e_d[i], 0),
                e_p,
                self.e_i[i],
                comma_seperated=True,
                end=",",
            )

        return (
            (self.kp[i] * e_p) + (self.kd[i] * self.e_d[i]) + (self.ki[i] * self.e_i[i])
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
            (-1) * math.cos(self.curr_attitude[0]) * math.sin(self.curr_attitude[1])
        )
        self.re3[1] = math.sin(self.curr_attitude[0])
        self.re3[2] = math.cos(self.curr_attitude[0])
        self.curr_attitude[3] = np.linalg.norm(self.vel)

        # Publishing data to the drone
        self.talker_pub(
            self.curr_attitude[0],
            self.curr_attitude[1],
            self.curr_attitude[2],
            self.curr_attitude[3],
        )

    def autopilot(self, targ_pos, duration):
        """Implements position control for pluto

        Args:
            targ_pos (list): list of 3 numbers for the next position to go to
            duration (int): duration of attempt for reaching the next position
        """
        self.start = time.time()
        start = time.time()
        while time.time() - start < duration:
            message = self.server.receive()
            self.curr_pos = np.array(message.split(","))
            self.curr_pos[0] = self.x_filter.predict_kalman(self.curr_pos[0])
            self.curr_pos[1] = self.y_filter.predict_kalman(self.curr_pos[1])
            self.curr_pos[2] = self.z_filter.predict_kalman(self.curr_pos[2])
            if (
                max(
                    [
                        abs(targ_pos[0] - self.curr_pos[0]),
                        abs(targ_pos[1] - self.curr_pos[1]),
                    ]
                )
            ) < self.tol:
                print("Position Reached!!")
                break
            self.pos_change(targ_pos)
