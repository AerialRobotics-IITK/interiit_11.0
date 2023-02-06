#!/usr/bin/env python3

import rospy
from env import *
from controller import pidcontroller
from threading import Thread

ct = 1

rospy.init_node("controller_2", anonymous=True)

pluto1 = pidcontroller(
    "position_1",
    kp=[4, 4, 2.7],
    kd=[5, 5, 2.15],
    ki=[0.05, 0.05, 1.1],
    eqb_thrust=1550,
    IP=drone1_ip,
    PORT=PORT,
)
pluto2 = pidcontroller(
    "position_2",
    kp=[4, 4, 2.7],
    kd=[5, 5, 2.15],
    ki=[0.05, 0.05, 1.1],
    eqb_thrust=1550,
    IP=drone2_ip,
    PORT=PORT,
)

pluto1.talker.disarm()
pluto1.talker.arm()
pluto1.talker.actual_takeoff()

pluto2.talker.disarm()
pluto2.talker.arm()
pluto2.talker.actual_takeoff()

def run_drone1():
    global ct
    while ct < 11:
      if ct == 1:
          pluto1.autopilot([-40, -40, 80], 8)
          ct += 1 #2
      if ct == 2:
          pluto1.autopilot([40, -40, 80], 8)
          ct += 1 #3
      if ct == 4:
          pluto1.autopilot([40, 40, 80], 8)
          ct += 1
      if ct == 6:
          pluto1.autopilot([-40, 40, 80], 8)
          ct += 1
      if ct == 8:
          pluto1.autopilot([-40, -40, 80], 8)
          ct += 1
      if ct == 10:
          pluto1.talker.actual_land()
          pluto1.talker.land()
          ct +=1
        
def run_drone2():
    global ct
    while ct < 13:
      if ct == 2:
          pluto2.autopilot([-40, -40, 80], 8)
          ct += 1
      if ct == 4:
          pluto2.autopilot([40, -40, 80], 8)
          ct += 1
      if ct == 6:
          pluto2.autopilot([40, 40, 80], 8)
          ct += 1
      if ct == 8:
          pluto2.autopilot([-40, 40, 80], 8)
          ct += 1
      if ct == 10:
          pluto2.autopilot([-40, -40, 80], 8)
          ct += 1
      if ct == 12:
          pluto2.talker.actual_land()
          pluto2.talker.land()
          ct +=1

if __name__ == "__main__":
  drone1_thread = Thread(target=run_drone1)
  drone2_thread = Thread(target=run_drone2)

  drone1_thread.start()
  drone2_thread.start()
