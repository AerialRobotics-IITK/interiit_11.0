#!/usr/bin/env python3
import cv2 as cv
from cv2 import aruco
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
import message_filters
import time
import os
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from kalman import KalmanFilter
from env import *

# ---------------------------------------------------
# | Global Parameters, change according to use case |
# ---------------------------------------------------

# Publish image to 'detected' topic and pose of ith marker to corresponding 'position_i' topic
pub_1 = rospy.Publisher('detected', Image, queue_size=10)
pub_2 = rospy.Publisher('position_1', numpy_msg(Floats), queue_size=10)
pub_3 = rospy.Publisher('position_2', numpy_msg(Floats), queue_size=10)

# Dictionary containing the AruCo Markers being used
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# Topic containing cam info of used camera and image recieved
Node_Name = 'listener'  # name of this node
cam_info_topic = '/camera/camera_info'
image_topic = '/camera/image_raw'

# Logging details
os.makedirs(os.path.join(LOG_FOLDER_PATH, "pose"), exist_ok=True)
FILE = os.path.join(LOG_FOLDER_PATH, f"pose/{int(time.time())}.log")
log_file = open(FILE, 'w')
start_time = time.time()

# Position arrays for drones
b = np.array([[0, 0, 0], [0, 0, 0]], dtype=np.float32)  # current position
b_uncorrected = np.array([[0, 0, 0], [0, 0, 0]], dtype=np.float32)  # uncorrected position
b_temp = np.array([[0, 0, 0], [0, 0, 0]], dtype=np.float32)  # for last detected value

# Bridge to convert rosmsg to cv
bridge = CvBridge()

# ------------------------------------------------------
# | Function to make kalman object with default values |
# ------------------------------------------------------
def make_kalman(dtime):
    F = np.array([[1, dtime, 0], [0, 1, dtime], [0, 0, 1]])
    H = np.array([1, 0, 0]).reshape(1, 3)
    Q = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
    R = np.array([0.5]).reshape(1, 1)
    kf = KalmanFilter(F=F, H=H, Q=Q, R=R)
    return F, H, Q, R, kf

# ---------------------------------------------------------
# | Function to update according to predictions by Kalman |
# ---------------------------------------------------------
def predict_kalman(id):
    global b, kf1, kf2, kf3, kf4, kf5, kf6, H1, H2, H3, H4, H5, H6

    if id==d1_ID:
        b[0][0] = np.dot(H1, kf1.predict())[0]
        b[0][1] = np.dot(H2, kf2.predict())[0]
        b[0][2] = np.dot(H3, kf3.predict())[0]
        kf1.update(b_uncorrected[0][0])
        kf2.update(b_uncorrected[0][1])
        kf3.update(b_uncorrected[0][2])

    elif id==d2_ID:
        b[1][0] = np.dot(H1, kf4.predict())[0]
        b[1][1] = np.dot(H2, kf5.predict())[0]
        b[1][2] = np.dot(H3, kf6.predict())[0]
        kf4.update(b_uncorrected[1][0])
        kf5.update(b_uncorrected[1][1])
        kf6.update(b_uncorrected[1][2])

# ----------------------------------------------------
# | Update the position according to latest detection|
# ----------------------------------------------------
def pose_update(tVector, it, id):
    global b, b_uncorrected, dist_ceiling

    if id==d1_ID:
        b[0][0] = round(tVector[it][0][0], 1)
        b[0][1] = round(tVector[it][0][1], 1)
        b[0][2] = dist_ceiling - round(tVector[it][0][2], 1)
        b_uncorrected[0] = b[0]

    if id==d2_ID:
        b[1][0] = round(tVector[it][0][0], 1)
        b[1][1] = round(tVector[it][0][1], 1)
        b[1][2] = dist_ceiling - round(tVector[it][0][2], 1)
        b_uncorrected[1] = b[1]

# ---------------------------
# | Kalman Filtering Objects|
# ---------------------------
dt = 1.0/60
F1, H1, Q1, R1, kf1 = make_kalman(dt) # For X of marker 1
F2, H2, Q2, R2, kf2 = make_kalman(dt) # For Y of marker 1
F3, H3, Q3, R3, kf3 = make_kalman(dt) # For Z of marker 1
F4, H4, Q4, R4, kf4 = make_kalman(dt) # For X of marker 2
F5, H5, Q5, R5, kf5 = make_kalman(dt) # For Y of marker 2
F6, H6, Q6, R6, kf6 = make_kalman(dt) # For Z of marker 2

# ----------------------------------------------------
# | Callback function that does the image processing |
# ----------------------------------------------------
def callback(cam, data):
    global b, b_temp, b_uncorrected, bridge

    # Camera properties
    cam_mat = np.array(cam.K).reshape([3, 3])
    dist_coef = np.array(cam.D)
    # Image processing
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # Marker detection
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef)
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            # Get the marker corners in integer type
            cv.polylines(frame, [corners.astype(np.int32)],
                         True, (0, 255, 255), 4, cv.LINE_AA)
            point = cv.drawFrameAxes(
                frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

            if ids[0]==d1_ID:
                pose_update(tVec, i, d1_ID)  # Update values for marker of drone1
                predict_kalman(d1_ID)
            elif ids[0]==d2_ID: 
                pose_update(tVec, i, d2_ID)  # Update values for marker of drone2
                predict_kalman(d2_ID)

        if len(marker_IDs)==1:
            if marker_IDs[0][0]==d1_ID:
                # print(f"Marker ID {d2_ID} not detected, old values sent!")
                b[1] = b_temp[1]
            elif marker_IDs[0][0]==d2_ID:
                # print(f"Marker ID {d1_ID} not detected, old values sent!")
                b[0] = b_temp[0]
    else:
        # print("Both markers not detected, old values sent!")
        b[0] = b_temp[0]
        b[1] = b_temp[1]

    # Convert back to rosmsg and publish
    img = bridge.cv2_to_imgmsg(frame, 'rgb8')
    pub_1.publish(img)
    pub_2.publish(b[0])
    pub_3.publish(b[1])
    b_temp[0] = b[0]
    b_temp[1] = b[1]

    # for logging only!
    print(time.time()-start_time,",",b[0][0],",",b[0][1],",",b[0][2],",",b_uncorrected[0][0],",",
            b_uncorrected[0][1],",",b_uncorrected[0][2],",",b[1][0],",",b[1][1],",",b[1][2],",",
            b_uncorrected[1][0],",",b_uncorrected[1][1],",",b_uncorrected[1][2], file=log_file) 

# --------------------------------------
# | Listener function to call the nodes|
# --------------------------------------
def listener():

    # Initialize the node, subscribe to camera matrix and Image topics
    rospy.init_node(Node_Name, anonymous=True)
    cam_info = message_filters.Subscriber(cam_info_topic, CameraInfo)
    img = message_filters.Subscriber(image_topic, Image)

    # Sync the multiple subscribed topics, and process image thru callback
    ts = message_filters.TimeSynchronizer([cam_info, img], CAM_FRAME_RATE, 1)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    print("time,x_1,y_1,z_1,x_o_1,y_o_1,z_o_1,x_2,y_2,z_2,x_o_2,y_o_2,z_o_2", file=log_file)  # for logging only!
    listener()