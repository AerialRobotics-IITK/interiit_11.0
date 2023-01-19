#!/usr/bin/env python3
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

#---------------------------------------------------
#| Global Parameters, change according to use case |
#---------------------------------------------------

# In Centimetres
MARKER_SIZE = 4 

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
b=np.array([0,0,0], dtype = np.float32)

#----------------------------------------------------
#| Callback function that does the image processing |
#----------------------------------------------------

def callback(cam,data):

    cam_mat = np.array(cam.K).reshape([3, 3])
    dist_coef = np.array(cam.D)

    # To change image from rosmsg to OpenCV type
    bridge = CvBridge() 
    frame = bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')

    #Grayscale the frame and run marker detection
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:
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

            #Put the calculated estimate on the image
            # cv.putText(frame,f"id: {ids[0]} Dist: {round(distance, 2)}",top_right,cv.FONT_HERSHEY_PLAIN,1.3,(0, 0, 255),2,cv.LINE_AA,)
            print(f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} z: {round(tVec[i][0][2],1)}")

            #Update values to the shared memory
            b[0] = round(tVec[i][0][0],1)
            b[1] = round(tVec[i][0][1],1)
            b[2] = 101 - round(tVec[i][0][2],1)

    #Convert back to rosmsg and publish
    img=bridge.cv2_to_imgmsg(frame,'rgb8')
    pub_1.publish(img)
    pub_2.publish(b)

def listener():

    #Initialize the node, subscribe to camera matrix and Image topics
    rospy.init_node(Node_Name, anonymous=True)
    cam_info = message_filters.Subscriber(cam_info_topic, CameraInfo)
    img = message_filters.Subscriber(image_topic, Image)

    #Sync the multiple subscribed topics, and process image thru callback
    ts = message_filters.TimeSynchronizer([cam_info,img],10,1)
    ts.registerCallback(callback)                                                                                                     
    rospy.spin()

if __name__ == '__main__':
    listener()