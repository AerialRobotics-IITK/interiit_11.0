from picamera2 import Picamera2, Preview
import time
import cv2 as cv
from cv2 import aruco
import numpy as np
picam = Picamera2()

config = picam.create_preview_configuration(lores={"size":(640,360)})

picam.configure(config)

cam_mat = [[1266.78359602357, 0, 608.4607470200075],[ 0, 1267.334024187105, 374.6054937400205],[ 0, 0, 1]]
dist_coef = [0.06107748316798726, -0.2708303958945525, 0.002115970398267557, -0.008895203586217777, 0]
cam_mat = np.array(cam_mat)
cam_mat = 0.5*cam_mat
cam_mat[2][2] = 1.0
dist_coef = np.array(dist_coef)

#In centimeters, chaneg accordingly
MARKER_SIZE = 20.9

 #Dictionary of the marker in use
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

# picam.start_preview(Preview.QTGL)

picam.start()
time.sleep(2)
i=0
count=0
start = time.time()
while(True):
    frame = picam.capture_array("main")
    i = i + 1
    count += 1
    

    #Convert to grayscale and run marker detection
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(marker_corners, MARKER_SIZE, cam_mat, dist_coef)
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

            # Draw the pose of the marker and draw the pose
            distance = np.sqrt(tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2)
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)

            #Put the marker ID and Distance on top-right, and the real-world coordinates with camera as origin on bottom-right
            cv.putText(frame, f"id: {ids[0]} Dist: {round(distance, 2)}", top_right, cv.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv.LINE_AA,)
            cv.putText(frame, f"x:{round(tVec[i][0][0],1)} y:{round(tVec[i][0][1],1)} z:{round(tVec[i][0][2],1)}", bottom_right, cv.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 2, cv.LINE_AA,)
            print(f"x:{round(tVec[i][0][0],1)} y:{round(tVec[i][0][1],1)} z:{round(tVec[i][0][2],1)}")
    
    if count%20 == 0:
        cv.imwrite("test.jpg", frame)
    #print(f"x:{round(tVec[i][0][0],1)} y:{round(tVec[i][0][1],1)} z:{round(tVec[i][0][2],1)}")
print("Rate : " + str(i/(time.time()-start)))
