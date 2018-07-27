#!/bin/bash

libdir="/home/pi/catkin_ws/src/tycho/arduino/libraries"

source ~/catkin_ws/devel/setup.bash

rm -rf "$libdir/ros_lib/"

rosrun rosserial_arduino make_libraries.py "$libdir"


