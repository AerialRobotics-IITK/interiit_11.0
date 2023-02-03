#!/usr/bin/env python3
import math 
import time
import os
import numpy as np
from protocol import Protocol
from params import *
from multiprocessing import shared_memory

# -----------------------
# | For logging Purpose |
# -----------------------

os.makedirs(os.path.join(LOG_FOLDER_PATH, "controller"), exist_ok=True)
LOGFILE = os.path.join(LOG_FOLDER_PATH, f"controller/{int(time.time())}.log")
log_file = open(LOGFILE, 'w')
start_time = time.time()

class pidcontroller:

    def __init__(self, kp, kd, ki, eqb_thrust, IP="192.168.4.1", PORT=23):
        self.existing_shm = shared_memory.SharedMemory(name="test")
        self.talker = Protocol(IP, PORT)
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.prev_time = [0.0,0.0,0.0]
        self.error_tol=0.01
        self.prev_error = [0.0,0.0,0.0]
        self.e_i = [0.0,0.0,0]
        self.e_d = [0.0, 0.0, 0.0]
        self.vel = np.array([0.0,0.0,0.0])
        self.speed = 1 
        self.repeat_pilot = False
        self.single_point_error = 0.001
        self.multiple_point_error = 0.2 
        self.ang_error = 0.05 
        self.length = 1
        self.curr_pos=np.ndarray((3,), dtype=np.float32, buffer=self.existing_shm.buf)
        self.curr_attitude=[0,0,0,0]
        self.equilibrium_thrust=eqb_thrust
        self.equilibrium_attitude = [0,0,0,15]
        self.b3d = np.array([0.0,0.0,0.0])
        self.re3 = np.array([0.0,0.0,0.0])
        self.time=0.0

# ----------------------------------------------------
# | Clipping funtion for the RPY,Thrust of the drone |
# ----------------------------------------------------

    def clip(self,parameter,low,high):

        if parameter<low:
            parameter = low
        elif parameter>high:
            parameter = high
        return parameter

# -------------------------------------------------
# | Publishing Function to send RPYT to the drone |
# -------------------------------------------------

    def talker_pub(self ,roll, pitch, yaw, thrust):

        #RPYT are converted to drones scale and clipped
        pitch=1500 + pitch*(1800/pi)
        pitch = self.clip(pitch,1300,1700) 
        roll=1500 + roll*(1800/pi)
        roll = self.clip(roll,1300,1700)
        thrust = self.clip(thrust,1000,2099)
        yaw = 1500

        #Use of protocol for sending RPYT
        print(roll,",",pitch,",",yaw,",",thrust,",",self.curr_pos[0],",",self.curr_pos[1],",",self.curr_pos[2],file=log_file)
        self.talker.set_RPY_THR(int(roll), int(pitch), int(yaw), int(thrust))
          
# ---------------------------------------------------------
# | Function for calculating control variable from Errors |
# ---------------------------------------------------------

    def calc_error(self,i,error):

        curr_time = time.time() 
        dt = 0.0
        if self.prev_time[i] != 0.0:
            dt = curr_time - self.prev_time[i]
        de = error - self.prev_error[i]
        e_p = error
        if self.prev_time[i] != 0:
            self.e_i[i] += error*dt
        self.e_d[i] = 0
        if dt > 0:
            self.e_d[i] = de/dt
        self.prev_time[i] = curr_time
        self.prev_error[i] = error
        if i == 0:
            print(time.time()-start_time,end=",",file=log_file)
        print(np.round(self.e_d[i],0),",",e_p,",",self.e_i[i],end=",", file=log_file)
        return (self.kp[i] * e_p) + (self.kd[i]*self.e_d[i]) + (self.ki[i]*self.e_i[i])

# ----------------------------------------------------------
# | Function for changing RPY,Thrust from control Variable |
# ----------------------------------------------------------

    def pos_change(self,targ_pos=([0,0,0]), curr_attitude = ([0,0,0,15])):

        errors = (targ_pos[0]-self.curr_pos[0], targ_pos[1]-self.curr_pos[1], targ_pos[2]-self.curr_pos[2])
        for i in range(len(errors)):
            self.vel[i] = self.calc_error(i,errors[i])
        self.vel[2] += self.equilibrium_thrust
        for i in range(3):
            self.b3d[i] = self.vel[i]
        self.b3d = self.b3d / np.linalg.norm(self.b3d)
        self.curr_attitude[0] = math.atan(self.b3d[0] / self.b3d[2]) 
        self.curr_attitude[1] = (-1)*math.asin(self.b3d[1])
        
        if self.curr_attitude[0]>1:
            self.curr_attitude[0]=1
        if self.curr_attitude[0]<-1:
            self.curr_attitude[0]=-1
        if self.curr_attitude[1]>1:
            self.curr_attitude[1]=1
        if self.curr_attitude[1]<-1:
            self.curr_attitude[1]=-1

        self.re3[0] = (-1)*math.cos(self.curr_attitude[0])*math.sin(self.curr_attitude[1])
        self.re3[1] = math.sin(self.curr_attitude[0])
        self.re3[2] = (math.cos(self.curr_attitude[0]))
        self.curr_attitude[3] = np.linalg.norm(self.vel)

        #Publishing data to the drone
        self.talker_pub(self.curr_attitude[0], self.curr_attitude[1], self.curr_attitude[2], self.curr_attitude[3])

# --------------------------------------------------------
# | Wrapper function to call pos_change at required rate |
# --------------------------------------------------------

    def autopilot(self,targ_pos,duration):

        self.start=time.time()
        start = time.time()
        while time.time()-start<duration:
            now = time.time()
            self.pos_change(targ_pos,self.curr_attitude)
            elapsed = time.time()-now
            if(elapsed<0.04):
                time.sleep(0.04-elapsed)
            
print("time,e_d_x,e_p_x,e_i_x,e_d_y,e_p_y,e_i_y,e_d_z,e_p_z,e_i_z,roll,pitch,yaw,thrust,x,y,z", file=log_file)