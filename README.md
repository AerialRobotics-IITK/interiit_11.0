# InterIIT_2023
# ArUCo Pose Estimate
## ArUCo_Pose
OpenCV script for pose estimate using webcam. Calibration data needed in the same directory of the webcam in use, in directory /calib_data/
## ArUCo_Pose_Rpi
Method-Class breakdown of detection on RPi camera, returns a list with current position of the ArUCo marker, if any
## ocam_pose
ROS package for testing using O_cam and ROS, integrated with controller. Also needs the ocam ros package to run. To be physically tested

- To install

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
```
Copy the package to src, then cd to catkin_ws/ and build
```bash
catkin build
```

- To run

To run the detection, run the [ocam ROS](https://github.com/AerialRobotics-IITK/ocam) framework first
```bash
roslaunch ocam ocam_ros.launch
```
Then for the detection node, source the workspace and run the script
```bash
source catkin_ws/devel/setup.bash
cd catkin_ws/src/ocam_pose/scripts
./aruco_ros_ocam
```
To view the detected image, use rqt_image_viewer
```bash
rqt_image_viewer
```
For the controller, source the same workspace in a new terminal and run the controller node
```bash
source catkin_ws/devel/setup.bash
cd catkin_ws/src/ocam_pose/scripts
./lee_position_controller.py
```
