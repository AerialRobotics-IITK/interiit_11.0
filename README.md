# InterIIT_2023
# ArUCo Pose Estimate
## ArUCo_Pose
OpenCV script for pose estimate using webcam. Calibration data needed in the same directory of the webcam in use, in directory /calib_data/
## package
ROS package for testing using O_cam, testing needed
- To install
```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
```
Copy the package to src, then cd to catkin_ws/src and build
```bash
catkin build
```
- To run 

To run the node, source the workspace and run ArUCo_Pose_ROS.py
```bash
source catkin_ws/devel/setup.bash
rosrun package ArUCO_Pose_ROS.py
```
