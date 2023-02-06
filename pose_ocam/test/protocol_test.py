from plutolib.protocol import Protocol

IP = "192.168.4.1"
PORT = 23

talker = Protocol(IP, PORT)
talker.arm()
talker.actual_takeoff()
while True:
    talker.set_RPY_THR(1500, 1500, 1500, 1800)
