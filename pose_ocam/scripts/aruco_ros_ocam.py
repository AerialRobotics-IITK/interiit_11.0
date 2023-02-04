#!/usr/bin/env python3

import cv2 as cv
from cv2 import aruco
from cv_bridge import CvBridge

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
import message_filters

import time
import socket
import os

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from multiprocessing import shared_memory
from kalman import KalmanFilter
from params import *

# ---------------------------------------------------
# | Global Parameters, change according to use case |
# ---------------------------------------------------
# In Centimetres
MARKER_SIZE = 5.3
dist_ceiling = 233

# Publish image to 'detected' topic and pose to 'position' topic
pub_1 = rospy.Publisher('detected', Image, queue_size=10)
pub_2 = rospy.Publisher('position', numpy_msg(Floats), queue_size=10)

# Dictionary containing the AruCo Markers being used
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# topic containing cam info of used camera and image recieved
Node_Name = 'listener'  # name of this node
cam_info_topic = '/camera/camera_info'
image_topic = '/camera/image_raw'

os.makedirs(os.path.join(LOG_FOLDER_PATH, "pose"), exist_ok=True)
FILE = os.path.join(LOG_FOLDER_PATH, f"pose/{int(time.time())}.log")
log_file = open(FILE, 'w')
start_time = time.time()

# Testing only!!!
count = 0
not_count = 0

# Position arrays for drone
b = np.array([0, 0, 0], dtype=np.float32)  # current position
b_uncorrected = np.array([0, 0, 0], dtype=np.float32)  # uncorrected position
b_temp = np.array([0, 0, 0], dtype=np.float32)  # for last detected value

# Bridge to convert rosmsg to cv
bridge = CvBridge()

# Parameters specific to Kalman Filtering
dt = 1.0/60
lambda_a = 0.6
lambda_u = 0.6
F1 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H1 = np.array([1, 0, 0]).reshape(1, 3)
Q1 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R1 = np.array([0.5]).reshape(1, 1)
kf1 = KalmanFilter(F=F1, H=H1, Q=Q1, R=R1)
F2 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H2 = np.array([1, 0, 0]).reshape(1, 3)
Q2 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R2 = np.array([0.5]).reshape(1, 1)
kf2 = KalmanFilter(F=F2, H=H2, Q=Q2, R=R2)
F3 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H3 = np.array([1, 0, 0]).reshape(1, 3)
Q3 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R3 = np.array([0.5]).reshape(1, 1)
kf3 = KalmanFilter(F=F3, H=H3, Q=Q3, R=R3)

# clahe objects
clahe = cv.createCLAHE(clipLimit=20)

# ---------------------------------------------------------
# | Function to update according to predictions by Kalman |
# ---------------------------------------------------------


def predict_kalman():
    global b, kf1, kf2, kf3, H1, H2, H3

    b[0] = np.dot(H1, kf1.predict())[0]
    b[1] = np.dot(H2, kf2.predict())[0]
    b[2] = np.dot(H3, kf3.predict())[0]
    kf1.update(b_uncorrected[0])
    kf2.update(b_uncorrected[1])
    kf3.update(b_uncorrected[2])

# ----------------------------------------------------
# | Update the position according to latest detection|
# ----------------------------------------------------


def pose_update(tVector, it):
    global b, b_uncorrected

    b[0] = round(tVector[it][0][0], 1)
    b[1] = round(tVector[it][0][1], 1)
    b[2] = dist_ceiling - round(tVector[it][0][2], 1)
    b_uncorrected = b.copy()

# ----------------------------------------------------
# | Callback function that does the image processing |
# ----------------------------------------------------


def callback(cam, data):
    global count, not_count, b, b_temp, b_uncorrected, bridge

    # Camera properties
    cam_mat = np.array(cam.K).reshape([3, 3])
    dist_coef = np.array(cam.D)

    # Image processing
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # gray_frame = clahe.apply(gray_frame)

    # Marker detection
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:

        count = count + 1
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):

            # Get the marker corners in integer type
            cv.polylines(frame, [corners.astype(np.int32)],
                         True, (0, 255, 255), 4, cv.LINE_AA)
            point = cv.drawFrameAxes(
                frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            distance = np.sqrt(tVec[i][0][2] ** 2 +
                               tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)
            estimate = (top_right + top_left + bottom_left + bottom_right)/4
            tan = distance/np.sqrt(tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)
            # print("Pixel: ", estimate)
            # print("Distance: ", distance)
            # print("Tan: ", tan)
            # print(b[0],",",b[1],",",b[2],",",b_uncorrected[0],",",b_uncorrected[1],",",b_uncorrected[2])
            # print(top_left[0],",",top_left[1],",",top_right[0],",",top_right[1],",",bottom_left[0],",",bottom_left[1],",",bottom_right[0],",",bottom_right[1])

            pose_update(tVec, i)  # Update values according to latest detection
            predict_kalman()  # Kalman Filtering

    else:
        # print("Marker not detected, old values sent!")
        b = b_temp.copy()
        not_count = not_count + 1

    # Convert back to rosmsg and publish
    img = bridge.cv2_to_imgmsg(frame, 'rgb8')
    pub_1.publish(img)
    pub_2.publish(b)
    b_temp = b.copy()
    # print(time.time() - start_time)
    print(f"{time.time() - start_time}, {b[0]}, {b[1]}, {b[2]}, {b_uncorrected[0]}, {b_uncorrected[1]}, {b_uncorrected[2]}",
          file=log_file)  # for logging only!


def listener():
    # --------------------------------------
    # | Listener function to call the nodes|
    # --------------------------------------

    # Initialize the node, subscribe to camera matrix and Image topics
    rospy.init_node(Node_Name, anonymous=True)
    cam_info = message_filters.Subscriber(cam_info_topic, CameraInfo)
    img = message_filters.Subscriber(image_topic, Image)

    # Sync the multiple subscribed topics, and process image thru callback
    ts = message_filters.TimeSynchronizer([cam_info, img], 54, 1)
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    print("time,x,y,z,x_o,y_o,z_o", file=log_file)  # for logging only!
    # print("tl_x, tl_y, tr_x, tr_y, bl_x, bl_y, br_x, br_y")
    listener()
