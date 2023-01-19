import cv2 as cv
from cv2 import aruco
import numpy as np

#Path to the calibration matrix of the camera in use
calib_data_path = "/home/legendarygene/InterIIT_2023/calib_data/MultiMatrix.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

#In centimeters, chaneg accordingly
MARKER_SIZE = 8

 #Dictionary of the marker in use
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters_create()

Video = cv.VideoCapture(2)
while True:

    ret, frame = Video.read()
    if not ret: #Break if the video is not captured
        print("Unable to get video feed")
        break

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

    cv.imshow("Feed", frame)
    key = cv.waitKey(1)

    #Press q to stop the video
    if key == ord("q"):
        break
Video.release()
cv.destroyAllWindows()