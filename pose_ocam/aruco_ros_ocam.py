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
import os
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from plutolib.kalman import KalmanFilter
from plutolib.logger import Logger

# from multiprocessing import shared_memory

# ---------------------------------------------------
# | Global Parameters, change according to use case |
# ---------------------------------------------------
# In Centimetres
MARKER_SIZE = 5.3
dist_ceiling = 233

# Publish image to 'detected' topic and pose to 'position' topic
pub_1 = rospy.Publisher("detected", Image, queue_size=10)
pub_2 = rospy.Publisher("position_1", numpy_msg(Floats), queue_size=10)
pub_3 = rospy.Publisher("position_2", numpy_msg(Floats), queue_size=10)
# Dictionary containing the AruCo Markers being used
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# topic containing cam info of used camera and image recieved
Node_Name = "listener"  # name of this node
cam_info_topic = "/camera/camera_info"
image_topic = "/camera/image_raw"
LOG_FOLDER_PATH = "../../logs"

logger = Logger(LOG_FOLDER_PATH, "pose")

start_time = time.time()

# Position arrays for drone
b = np.array([[0, 0, 0], [0, 0, 0]], dtype=np.float32)  # current position
b_uncorrected = np.array(
    [[0, 0, 0], [0, 0, 0]], dtype=np.float32
)  # uncorrected position
b_temp = np.array([[0, 0, 0], [0, 0, 0]], dtype=np.float32)  # for last detected value

# Bridge to convert rosmsg to cv
bridge = CvBridge()

# Parameters specific to Kalman Filtering
dt = 1.0 / 60
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
F4 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H4 = np.array([1, 0, 0]).reshape(1, 3)
Q4 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R4 = np.array([0.5]).reshape(1, 1)
kf4 = KalmanFilter(F=F4, H=H4, Q=Q4, R=R4)
F5 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H5 = np.array([1, 0, 0]).reshape(1, 3)
Q5 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R5 = np.array([0.5]).reshape(1, 1)
kf5 = KalmanFilter(F=F5, H=H5, Q=Q5, R=R5)
F6 = np.array([[1, dt, 0], [0, 1, dt], [0, 0, 1]])
H6 = np.array([1, 0, 0]).reshape(1, 3)
Q6 = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.0], [0.0, 0.0, 0.0]])
R6 = np.array([0.5]).reshape(1, 1)
kf6 = KalmanFilter(F=F6, H=H6, Q=Q6, R=R6)

# ---------------------------------------------------------
# | Function to update according to predictions by Kalman |
# ---------------------------------------------------------


def predict_kalman(id):
    global b, kf1, kf2, kf3, H1, H2, H3

    if id == 0:
        b[0][0] = np.dot(H1, kf1.predict())[0]
        b[0][1] = np.dot(H2, kf2.predict())[0]
        b[0][2] = np.dot(H3, kf3.predict())[0]
        kf1.update(b_uncorrected[0][0])
        kf2.update(b_uncorrected[0][1])
        kf3.update(b_uncorrected[0][2])
    elif id == 10:
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
    global b, b_uncorrected

    if id == 0:
        b[0][0] = round(tVector[it][0][0], 1)
        b[0][1] = round(tVector[it][0][1], 1)
        b[0][2] = dist_ceiling - round(tVector[it][0][2], 1)
        b_uncorrected[0] = b[0]
    if id == 10:
        b[1][0] = round(tVector[it][0][0], 1)
        b[1][1] = round(tVector[it][0][1], 1)
        b[1][2] = dist_ceiling - round(tVector[it][0][2], 1)
        b_uncorrected[1] = b[1]


# ----------------------------------------------------
# | Callback function that does the image processing |
# ----------------------------------------------------


def callback(cam, data):
    global count, not_count, b, b_temp, b_uncorrected, bridge

    # Camera properties
    cam_mat = np.array(cam.K).reshape([3, 3])
    dist_coef = np.array(cam.D)

    # Image processing
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Marker detection
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )

    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            # Get the marker corners in integer type
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            if ids[0] == 0:
                pose_update(tVec, i, 0)  # Update values according to latest detection
                predict_kalman(0)
            elif ids[0] == 10:
                pose_update(tVec, i, 10)  # Update values according to latest detection
                predict_kalman(10)

        if len(marker_IDs) == 1:
            if marker_IDs[0][0] == 0:
                # print("Marker ID 10 not detected, old values sent!")
                b[1] = b_temp[1]
            elif marker_IDs[0][0] == 10:
                # print("Marker ID 0 not detected, old values sent!")
                b[0] = b_temp[0]
    else:
        # print("Both markers not detected, old values sent!")
        b[0] = b_temp[0]
        b[1] = b_temp[1]

    # Convert back to rosmsg and publish
    img = bridge.cv2_to_imgmsg(frame, "rgb8")
    pub_1.publish(img)
    pub_2.publish(b[0])
    pub_3.publish(b[1])
    b_temp[0] = b[0]
    b_temp[1] = b[1]
    logger.print(
        time.time() - start_time,
        b[0][0],
        b[0][1],
        b[0][2],
        b_uncorrected[0][0],
        b_uncorrected[0][1],
        b_uncorrected[0][2],
        comma_seperated=True,
    )
    # f"{time.time() - start_time}, {b[0][0]}, {b[0][1]}, {b[0][2]}, {b_uncorrected[0][0]}, {b_uncorrected[0][1]}, {b_uncorrected[0][2]}",


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


if __name__ == "__main__":
    logger.print("time,x,y,z,x_o,y_o,z_o", init=True)
    listener()
