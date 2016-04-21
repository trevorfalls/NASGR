#!/bin/bash

source /opt/ros/indigo/setup.bash
sudo iwconfig wlan0 power off
rosrun joy joy_node
