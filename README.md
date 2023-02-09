# InterIIT_2023

## Installation

```bash
git clone -b main git@github.com:<removed>/interiit_11.0.git
cd interiit_11.0
git submodule init
git submodule update
```

Install OpenCV 4.2.0

```bash
sudo apt install libopencv-dev python3-opencv
```

Create a virtualenv

```bash
sudo apt install python3.8-venv
python -m venv ./.venv
source ./.venv/bin/activate
```

Install python requirements

```bash
pip install -r requirements.txt
```

## Execution

To build `pose_ocam`

```bash
cd pose_ocam
mkdir build && cd build
cmake ..
make
```

You should find the executable `ocam` inside `build` now.  
Check the file of ocam using `ls /dev/video*`, check that it's the same in `./pose_ocam/src/main.cpp` as `devpath`. If not, change it and run the script again. Otherwise, to run pose estimation through oCam-1CGN-U, or any of the similar series,

```bash
./pose_ocam/build/ocam
```

Note that quality of image would decide it's latency. To improve the parameters, try changing `exposure` and `gain` the same file.

In another terminal, run the following to install package for the controller

```bash
cd pluto_control
pip install . # if not done by the above script by chance
```

Run the controller in another terminal using

```bash
python single.py # this starts a log in src/logs/controller
```
