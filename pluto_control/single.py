#!/usr/bin/env python3
from plutolib.controller import pidcontroller
import argparse
from env import *

parser = argparse.ArgumentParser(
    description="file for flying single drone. ID=0 is alloted to server_port 7000, ID=10 is alloted to server_port 7001"
)
parser.add_argument("-port", type=int, help="server_port chooses ID", default=7000)
parser.add_argument("-drone", type=int, help="drone_num chooses drone_num", default=1)
args = parser.parse_args()

def run_drone(point_list):
    pluto = pidcontroller(
        pose_port=args.port,
        kp=[3, 3, 2.7],
        kd=[3.75, 3.75, 2.15],
        ki=[0.05, 0.05, 1.1],
        eqb_thrust=1550,
        IP=IP,
        PORT=PORT,
        drone_num=args.drone,
        LOG_FOLDER_PATH = '../pluto_logs'
    )
    pluto.talker.disarm()
    pluto.talker.arm()
    pluto.talker.actual_takeoff()
    i = 0
    while True:
        pluto.autopilot(point_list[i], duration=6)
        i = i + 1
        if i == len(point_list):
            break
    pluto.talker.actual_land()
    pluto.talker.land()


drone_points = [
    [0, 0, 80],
    [-40, -40, 80],
    [40, -40, 80],
    [40, 40, 80],
    [-40, 40, 80],
    [-40, -40, 80],
]

if __name__ == "__main__":
    run_drone(drone_points)