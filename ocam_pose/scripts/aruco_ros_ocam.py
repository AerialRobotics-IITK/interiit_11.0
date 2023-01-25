#!/usr/bin/env python3
import time
import rospy
import cv2 as cv
from cv2 import aruco
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import numpy as np
import message_filters
import socket
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from multiprocessing import shared_memory
from kalman import KalmanFilter

#---------------------------------------------------
#| Global Parameters, change according to use case |
#---------------------------------------------------
# In Centimetres
MARKER_SIZE = 5.3

#Publish to 'detected' topic
pub_1 = rospy.Publisher('detected', Image, queue_size=10)
pub_2 = rospy.Publisher('position', numpy_msg(Floats), queue_size=10)

#Dictionary containing the AruCo Markers being used
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

#Name of this node
Node_Name = 'listener'

#topic containing cam info of used camera
cam_info_topic = '/camera/camera_info'

#topic where camera publishes image
image_topic = '/camera/image_raw'

#shared memory details
# a = np.array([0.0,0.0,0.0])
# shm = shared_memory.SharedMemory(name="aruco_pose",create=True, size=a.nbytes)
# b = np.ndarray(a.shape, dtype=a.dtype, buffer=shm.buf)
# b[:] = a[:]

#Testing only!!!
b = np.array([0,0,0], dtype = np.float32)
b_uncorrected = np.array([0,0,0], dtype = np.float32)
b_temp = np.array([0,0,0], dtype = np.float32)
u = np.array([0,0,0], dtype = np.float32)
a = np.array([0,0,0], dtype = np.float32)
proj = [[0,0,0],[0,0,0],[0,0,0]]
lambda_a = 0.6
lambda_u = 0.6
count = 0
not_count = 0
dt = 1.0/60
F1 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H1 = np.array([1, 0, 0]).reshape(1, 3)
Q1 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R1 = np.array([0.5]).reshape(1, 1)
kf1 = KalmanFilter(F = F1, H = H1, Q = Q1, R = R1)
F2 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H2 = np.array([1, 0, 0]).reshape(1, 3)
Q2 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R2 = np.array([0.5]).reshape(1, 1)
kf2 = KalmanFilter(F = F2, H = H2, Q = Q2, R = R2)
F3 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H3 = np.array([1, 0, 0]).reshape(1, 3)
Q3 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R3 = np.array([0.5]).reshape(1, 1)
kf3 = KalmanFilter(F = F3, H = H3, Q = Q3, R = R3)

def importance(distance):
    if(distance > 20.0):
        return 1.0
    return distance**2/400.0

def distance(pose):
    dist = np.sqrt(pose[0]**2 + pose[1]**2 + pose[2]**2)
    return dist

def smoothen():
    global lambda_a, lambda_u, a, u, b, proj

    proj0 = np.array(proj[0], dtype = np.float32)
    proj1 = np.array(proj[1], dtype = np.float32)
    proj2 = np.array(proj[2], dtype = np.float32)

    projected = proj2 + u + a/2
    imp = importance(abs(distance(projected) - distance(b)))
    proj_send = ((projected*(1-imp) + b*imp))

    u_new = proj2- proj1
    a_new = proj2-proj1-(proj1-proj0)
    u_send = lambda_u*u + (1-lambda_u)*u_new
    a_send = lambda_a*a + (1-lambda_a)*a_new

    return[proj_send, u_send, a_send]

#----------------------------------------------------
#| Callback function that does the image processing |
#----------------------------------------------------

def callback(cam,data):
    global count, not_count, b, a, u, proj, b_temp, b_uncorrected
    global H1, H2, H3, kf1, kf2, kf3

    cam_mat = np.array(cam.K).reshape([3, 3])
    dist_coef = np.array(cam.D)

    # To change image from rosmsg to OpenCV type
    bridge = CvBridge() 
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')

    #Grayscale the frame and run marker detection
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:
        count = count + 1
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):

            #Get the marker corners in integer type
            cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)

            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()

            #Get the marker distance and draw the axes on the marker
            distance = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

            #Update values to the shared memory
            b[0] = round(tVec[i][0][0],1)
            b[1] = round(tVec[i][0][1],1)
            b[2] = 220 - round(tVec[i][0][2],1)
            b_uncorrected = b.copy()

            #smoothen values according to the importance function
            # smoothened_list = smoothen()
            # proj.append(b)
            # proj.pop(0)
            # b = smoothened_list[0]
            # u = smoothened_list[1]
            # a = smoothened_list[2]

            #Kalman Filtering
            b[0] = np.dot(H1, kf1.predict())[0] 
            b[1] = np.dot(H2, kf2.predict())[0]
            b[2] = np.dot(H3, kf3.predict())[0]
            kf1.update(b_uncorrected[0])
            kf2.update(b_uncorrected[1])
            kf3.update(b_uncorrected[2])

    else:   
        # print("Marker not detected, old values sent!")
        b = b_temp.copy()
        not_count = not_count + 1

    #Convert back to rosmsg and publish
    img = bridge.cv2_to_imgmsg(frame,'rgb8')
    pub_1.publish(img)
    pub_2.publish(b)
    b_temp = b.copy()
    print(b[0],",",b[1],",",b[2],",",b_uncorrected[0],",",b_uncorrected[1],",",b_uncorrected[2])

def listener():

    #Initialize the node, subscribe to camera matrix and Image topics
    rospy.init_node(Node_Name, anonymous=True)
    cam_info = message_filters.Subscriber(cam_info_topic, CameraInfo)
    img = message_filters.Subscriber(image_topic, Image)

    #Sync the multiple subscribed topics, and process image thru callback
    ts = message_filters.TimeSynchronizer([cam_info,img],30,1)
    ts.registerCallback(callback)                                                                                      
    rospy.spin()

if __name__ == '__main__':
    print("x,y,z,xx,yy,zz")
    listener()
    # print("Frames detected: ", count)
    # print("Not Detected: ", not_count)