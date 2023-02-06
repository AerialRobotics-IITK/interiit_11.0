import rospy
from plutolib.controller import pidcontroller

rospy.init_node("controller_2", anonymous=True)

env = {
    "IP": "192.168.4.1",
    "PORT": 23,
    "LOG_FOLDER_PATH": "../../logs",
    "VERBOSE": False,
    "LOG_FOLDER": "controller2",
}
pluto = pidcontroller(
    kp=[4, 4, 2.7],
    kd=[5, 5, 2.15],
    ki=[0.05, 0.05, 1.1],
    eqb_thrust=1550,
    env=env,
    pose_topic="position_2",
)
pluto.talker.disarm()
pluto.talker.arm()
pluto.talker.actual_takeoff()
pluto.autopilot([-40, -40, 80], 8)
pluto.autopilot([40, -40, 80], 8)
pluto.autopilot([40, 40, 80], 8)
pluto.autopilot([-40, 40, 80], 8)
pluto.autopilot([-40, -40, 80], 8)
pluto.talker.actual_land()
pluto.talker.land()
