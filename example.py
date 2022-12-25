from protocol import Protocol

talker = Protocol("192.168.4.1", 23)
talker.arm()
talker.takeoff()
talker.land() 
