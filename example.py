from protocol import Protocol

talker = Protocol("192.168.4.1", 23)

talker.arm()
while True:
    talker.set_RPY_THR(roll=1500, pitch=1500, yaw=1500, thrust=900)
