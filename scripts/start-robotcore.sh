#!/bin/bash

source /opt/ros/indigo/setup.bash
export ROS_IP=10.0.0.2
export ROS_MASTER_URI=http://10.0.0.2:11311
export ROS_PACKAGE_PATH=/opt/ros/indigo/stacks:$ROS_PACKAGE_PATH
sleep 2
roslaunch ~/git/NASGR/Guides/Cameras/usb_cam-single.launch
