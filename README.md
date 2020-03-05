# Online Next-Best-View Planner for Low-Cost Underwater Robots

Preliminary work on a NBV planner.

## Getting Started

This package has been tested on Ubuntu 16.04.

Set up your catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

## Dependencies

* Modified monocular ORB-SLAM
```
git clone https://github.com/dartmouthrobotics/ORB_SLAM2
cd ORB_SLAM2/
cd Thirdparty/
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin/
mkdir build
cd build/
cmake ..
cmake --build .
cd ../../../
chmod +x build.sh
./build.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Examples/ROS/ORB_SLAM2ROS
chmod +x build_ros.sh
./build_ros.sh
```

* [BlueROV ROS Playground](https://github.com/dartmouthrobotics/bluerov_ros_playground)

## Install

Clone the repository. Then build and source:
```
git clone https://github.com/cs169-wi20-robotics-perception-systems/nbv_underwater.git
cd ..
catkin_make
source devel/setup.bash
```

Check if the `rough_nbv_planner.py` file in `scripts` folder are executable. If not, run:
```
chmod +x rough_nbv_planner.py
```

## Run

Follow steps in [BlueROV ROS Playground](https://github.com/dartmouthrobotics/bluerov_ros_playground) for running, specifically part 2 for Gazebo.

To initiate the rough NBV planner:
```
roslaunch nbv_underwater nbv_planner.launch [record:=0/1]
```

If one wants to log the ROS topics, check if the `data` directory exists in the package. If not, please create one:
```
mkdir data
```
