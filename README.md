# InterIIT_2023

## Video Demonstration

<video width="320" height="240" controls>
  <source src="https://user-images.githubusercontent.com/98644005/217606026-c6da2e55-d6ce-437d-b046-64e2549282c3.mp4" type="video/mp4">
</video>
<!-- https://user-images.githubusercontent.com/98644005/217606026-c6da2e55-d6ce-437d-b046-64e2549282c3.mp4 -->

<!-- https://user-images.githubusercontent.com/98644005/217607227-f256c901-6ccb-4dab-825c-b3694c9fd840.mp4 -->


## Installation

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone git@github.com:AerialRobotics-IITK/interiit_11.0.git
catkin build
source ../devel/setup.bash # setup.zsh otherwise
```

## Execution

Run the following to open ocam_ros driver with ros pipeline. Check the file of ocam using `ls /dev/video*`, putting that in device_id

```bash
roslaunch ocam ocam_ros.launch device_id:=/dev/video2
```

In another terminal, run the following to obtain /detected topic

```bash
cd pose_ocam/scripts/
python aruco_ros_ocam.py # this will start a log in src/logs/pose
```

Run the controller in another terminal using

```bash
python controller.py # this starts a log in src/logs/controller
```

To view the logs

```bash
cd ../../visualizer
python3 controller.py ../logs/controller/{LOG_NAME} # replace controller with pose for pose logs
```
