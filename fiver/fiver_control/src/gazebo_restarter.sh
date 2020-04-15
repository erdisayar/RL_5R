#!/bin/bash

# this will kill the gazebo scene
source /opt/ros/kinetic/setup.bash
rosnode kill -a
pkill gzserver
pkill gzclient
killall  rosmaster

secs=1
echo "Waiting 1 second"
sleep $secs
cd ~/catkin_ws
source devel/setup.bash

roslaunch fiver_gazebo main.launch
