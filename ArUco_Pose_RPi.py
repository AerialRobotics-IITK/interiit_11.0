from picamera2 import Picamera2, Preview
import time
import cv2 as cv
from cv2 import aruco
import numpy as np

rpi_cam_mat = [[1266.78359602357, 0, 608.4607470200075],[ 0, 1267.334024187105, 374.6054937400205],[ 0, 0, 1]]
rpi_dist_coeff = [0.06107748316798726, -0.2708303958945525, 0.002115970398267557, -0.008895203586217777, 0]

ARUCO_SIZE = 20.9 #In centimeters, change accordingly
ARUCO_DICT = aruco.DICT_4X4_50 #Dictionary of the marker in use

class Pose:

    def __init__(self):

        self.picam = Picamera2()
        self.config = self.picam.create_preview_configuration(lores={"size":(640,360)})
        self.picam.configure(self.config)

        self.cam_mat = rpi_cam_mat
        self.dist_coef = rpi_dist_coeff

        self.cam_mat = np.array(self.cam_mat)
        self.cam_mat = 0.5*(self.cam_mat)
        self.cam_mat[2][2] = 1.0
        self.dist_coef = np.array(self.dist_coef)

        self.MARKER_SIZE = ARUCO_SIZE
        self.marker_dict = aruco.Dictionary_get(ARUCO_DICT)
        self.param_markers = aruco.DetectorParameters_create()
    
    def getPose(self):

        self.picam.start()
        time.sleep(2)
        frame = self.picam.capture_array("main")

        #Convert to grayscale and run marker detection
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, self.marker_dict, parameters=self.param_markers)
        pose = []
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, self.MARKER_SIZE, self.cam_mat, self.dist_coef)
            total_markers = range(0, marker_IDs.size)

            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                #Getting the marker corner coordinates
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                distance = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)
                pose.append = [round(tVec[i][0][0],1), round(tVec[i][0][1],1), round(tVec[i][0][2],1)]
        if len(pose) >= 0:
            return pose
        else:
            return [0,0,0]

getter = Pose()
position = getter.getPose
print(position)