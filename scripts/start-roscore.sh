#!/bin/bash

sleep 1
source /opt/ros/indigo/setup.bash
export ROS_IP=10.0.0.2
export ROS_MASTER_URI=http://10.0.0.2:11311
export ROS_PACKAGE_PATH=/opt/ros/indigo/stacks:$ROS_PACKAGE_PATH
roscore
