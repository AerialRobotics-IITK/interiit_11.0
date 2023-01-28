# InterIIT_2023

## Installation

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
git clone -b refactored git@github.com:AerialRobotics-IITK/InterIIT_2023.git src
cd src
git submodule init
git submodule update
catkin build
source ../devel/setup.bash # setup.zsh otherwise
```

## Execution

Run the following to open ocam_ros driver with ros pipeline. Check the file of ocam using `ls /dev/video*`, putting that in device_id

```bash
roslaunch ocam ocam_ros.launch show_rqt_image_view:=true device_id:=/dev/video2
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
