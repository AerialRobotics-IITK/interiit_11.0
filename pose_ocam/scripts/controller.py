from lee_position_controller import pidcontroller

pluto = pidcontroller(kp=[2.5, 2.5, 5], kd=[4, 4, 4.5], ki=[
                      0.001, 0.001, 0], eqb_thrust=1500)
pluto.talker.arm()
pluto.talker.actual_takeoff()

pluto.autopilot([-40, -40, 100], 10)
# pluto.autopilot([40, -40, 100], 10)
# pluto.autopilot([40, 40, 100], 10)
# pluto.autopilot([-40, 40, 100], 10)
# pluto.autopilot([-40, -40, 100], 10)
