#!/usr/bin/env python3
from controller import pidcontroller
from threading import Thread
from env import *

pluto1 = pidcontroller(
    server_port=5000,
    client_port=6000,
    pose_port=7000,
    kp=[3, 3, 2.7],
    kd=[3.75, 3.75, 2.15],    
    ki=[0.05, 0.05, 1.1],
    eqb_thrust=1550,
    IP=drone1_ip,
    PORT=PORT,
    drone_num=1
)

pluto2 = pidcontroller(
    server_port=6000,
    client_port=5000,
    pose_port=7001,
    kp=[3.5, 3.5, 2.7],
    kd=[4.3, 4.3, 2.15],
    ki=[0.05, 0.05, 1.1],
    eqb_thrust=1550,
    IP=drone2_ip,
    PORT=PORT,
    drone_num=2
)

def run_drone1(point_list):
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
    [-40, -40, 80],
    [40, -40, 80],
    [40, 40, 80],
    [-40, 40, 80],
    [-40, -40, 80],
]
drone2_points = [
    [-40, 40, 80],
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