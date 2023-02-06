from protocol import Protocol 
from env import *

talker= Protocol(IP, PORT)
talker.arm()
talker.actual_takeoff()
talker.takeoff()
talker.actual_land()
talker.land()