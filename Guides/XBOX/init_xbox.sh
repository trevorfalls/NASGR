#!/bin/bash
# This script sees where the Xbox controller data as outputted by xboxdrv will be
# It then sets that location as the place that joy should open
# You should plug the Xbox controller in, run roscore, then run this script, then run xboxdrv, then run the roslaunch file that opens all the nodes

clear
echo "Xbox Init Script."

echo
joystickPath=$(sudo timeout 0.1 xboxdrv --silent | grep '/dev/input/js')
joystickPathTrim="$(echo "${joystickPath}" | sed -e 's/^[[:space:]]*//')"
echo ":$joystickPathTrim:"
rosparam set joy_node/dev "${joystickPathTrim}"
