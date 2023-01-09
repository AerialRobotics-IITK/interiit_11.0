from protocol import Protocol
import time

talker = Protocol("192.168.4.1", 23)

talker.arm()
talker.actual_takeoff()
talker.takeoff()
# talker.actual_land()
thrust = 1650
curr = time.time()
while time.time() < 2.2:
    talker.set_RPY_THR(pitch=1600, thrust=thrust)
while time.time() < 2.3:
    talker.set_RPY_THR(pitch=1500, thrust=thrust)
while time.time() < 2.2:
    talker.set_RPY_THR(pitch=1400, thrust=thrust)

talker.land()
