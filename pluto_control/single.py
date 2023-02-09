#!/usr/bin/env python3
from plutolib.controller import pidcontroller
import argparse

parser = argparse.ArgumentParser(
    description="file for flying single drone. ID=0 is alloted to server_port 5000, ID=10 is alloted to server_port 6000"
)
parser.add_argument("-port", type=int, help="server_port chooses ID", default=5000)
args = parser.parse_args()


def run_drone(point_list):
    pluto = pidcontroller(
        env={
            "IP": "192.168.4.1",
            "PORT": 23,
            "LOG_FOLDER_PATH": "../pluto_logs",
            "VERBOSE": True,
            "LOG_FOLDER": "controller",
            "SERVER_PORT": args.port,
        },
        kp=[3, 3, 2.7],
        kd=[5, 5, 2.15],
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
    [-40, -60, 80],
    [40, -60, 80],
    [40, 20, 80],
    [-40, 20, 80],
    [-40, -50, 80],
]

if __name__ == "__main__":
    run_drone(drone_points)
