#!/usr/bin/env python3
import math
import time
import os
import rospy
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from protocol import Protocol
from env import *
from controller import pidcontroller
rospy.init_node("controller_1", anonymous=True)

pluto = pidcontroller(
    "position_1",
    kp=[3.5, 3.5, 2.7],
    kd=[4.3, 4.3, 2.15],
    ki=[0.05, 0.05, 1.1],
    eqb_thrust=1550,
    IP=drone1_ip,
    PORT=PORT,
    drone_num = 1
)
pluto.talker.disarm()
pluto.talker.arm()
pluto.talker.actual_takeoff()
pluto.autopilot([-40, -40, 80], 6)
pluto.autopilot([40, -40, 80], 6)
pluto.autopilot([40, 40, 80], 6)
pluto.autopilot([-40, 40, 80], 6)
pluto.autopilot([-40, -40, 80], 6)
pluto.talker.actual_land()
pluto.talker.land()