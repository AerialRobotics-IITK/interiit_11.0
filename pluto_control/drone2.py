#!/usr/bin/env python3
from plutolib.controller import pidcontroller


def run_drone(point_list):
    pluto = pidcontroller(
        env={
            "IP": "192.168.4.1",
            "PORT": 23,
            "LOG_FOLDER_PATH": "../pluto_logs",
            "VERBOSE": True,
            "LOG_FOLDER": "controller",
            "SERVER_PORT": 8000,
        }
    )
    pluto.talker.disarm()
    pluto.talker.arm()
    pluto.talker.actual_takeoff()
    i = 0
    while True:
        pluto.autopilot(point_list[i], duration=8)
        i = i + 1
        if i == len(point_list):
            break
    pluto.talker.actual_land()
    pluto.talker.land()


drone_points = [
    [-40, -40, 80],
    [40, -40, 80],
    [40, 40, 80],
    [-40, 40, 80],
    [-40, -40, 80],
]

if __name__ == "__main__":
    run_drone(drone_points)