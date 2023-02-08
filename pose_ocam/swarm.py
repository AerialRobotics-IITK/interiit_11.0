#!/usr/bin/env python3
from plutolib.controller import pidcontroller
from threading import Thread

escape = [False]


def run_drone1(point_list):
    pluto1 = pidcontroller(
        env={
            "IP": "192.168.0.10",
            "PORT": 23,
            "LOG_FOLDER_PATH": "../pluto_logs",
            "VERBOSE": True,
            "LOG_FOLDER": "controller",
            "SERVER_PORT": 5002,
        }
    )
    pluto1.talker.disarm()
    pluto1.talker.arm()
    pluto1.talker.actual_takeoff()
    global escape
    i = 0
    while True:
        pluto1.autopilot(point_list[i], duration=8, escape=escape)
        i = i + 1
        if i == len(point_list):
            break
    pluto1.talker.actual_land()
    pluto1.talker.land()


def run_drone2(point_list):
    pluto2 = pidcontroller(
        env={
            "IP": "192.168.0.20",
            "PORT": 23,
            "LOG_FOLDER_PATH": "../pluto_logs",
            "VERBOSE": False,
            "LOG_FOLDER": "controller",
            "SERVER_PORT": 5003,
        }
    )
    pluto2.talker.disarm()
    pluto2.talker.arm()
    pluto2.talker.actual_takeoff()
    global escape
    i = 0
    while True:
        pluto2.autopilot(point_list[i], duration=8, escape=escape)
        i = i + 1
        if i == len(point_list):
            break
    pluto2.talker.actual_land()
    pluto2.talker.land()


drone1_points = [
    [-40, -40, 80],
    [40, -40, 80],
    [40, 40, 80],
    [-40, 40, 80],
    [-40, -40, 80],
]
drone2_points = [
    [-10, 10, 80],
    [-40, -40, 80],
    [40, -40, 80],
    [40, 40, 80],
    [-40, 40, 80],
    [-40, -40, 80],
]

if __name__ == "__main__":
    drone1_thread = Thread(target=run_drone1, args=(drone1_points,))
    drone2_thread = Thread(target=run_drone2, args=(drone2_points,))
    drone1_thread.start()
    drone2_thread.start()
