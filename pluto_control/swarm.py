#!/usr/bin/env python3
from plutolib.controller import pidcontroller
from threading import Thread
from env import *
import queue

q1 = queue.Queue()
q2 = queue.Queue()


def run_drone1(point_list):
    pluto1 = pidcontroller(
        pose_port=7000,
        queue1=q1,
        queue2=q2,
        kp=[3, 3, 2.7],
        kd=[3.75, 3.75, 2.15],
        ki=[0.05, 0.05, 1.1],
        eqb_thrust=1550,
        IP=drone1_ip,
        PORT=PORT,
        drone_num=1,
        LOG_FOLDER_PATH = LOG_FOLDER_PATH
    )
    pluto1.talker.disarm()
    pluto1.talker.arm()
    pluto1.talker.actual_takeoff()
    i = 0
    while True:
        pluto1.concurrent_autopilot(point_list[i], duration=6)
        i = i + 1
        if i == 5:
            break
    pluto1.talker.actual_land()
    pluto1.talker.land()


def run_drone2(point_list):
    pluto2 = pidcontroller(
        queue1=q2,
        queue2=q1,
        pose_port=8000,
        kp=[3, 3, 2.7],
        kd=[3.75, 3.75, 2.15],
        ki=[0.05, 0.05, 1.1],
        eqb_thrust=1550,
        IP=drone2_ip,
        PORT=PORT,
        drone_num=2,
        LOG_FOLDER_PATH = LOG_FOLDER_PATH
    )
    pluto2.talker.disarm()
    pluto2.talker.arm()
    pluto2.talker.actual_takeoff()
    i = 0
    while True:
        pluto2.concurrent_autopilot(point_list[i], duration=6)
        i = i + 1
        if i == 5:
            pluto2.autopilot(point_list[i], duration=6)
            break
    pluto2.talker.actual_land()
    pluto2.talker.land()


drone1_points = [
    [-40, -40, 60],
    [40, -40, 60],
    [40, 40, 60],
    [-40, 40, 60],
    [-40, -40, 60],
]
drone2_points = [
    [-40, 40, 60],
    [-40, -40, 60],
    [40, -40, 60],
    [40, 40, 60],
    [-40, 40, 60],
    [-40, -40, 60],
]

if __name__ == "__main__":
    drone1_thread = Thread(target=run_drone1, args=(drone1_points,))
    drone2_thread = Thread(target=run_drone2, args=(drone2_points,))
    drone1_thread.start()
    drone2_thread.start()