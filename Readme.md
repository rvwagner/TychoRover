# Tycho Rover Code

This project contains the code needed to drive the Tycho prototype lunar rover.  Currently, it is set up to drive a 1/10th-scale model using an XBox 360 controller.

## Dependencies

Requires that [ROS][] be installed (tested on ROS Kinetic Kame on Ubuntu 16.04), and requires the following ROS packages: 
- [Joy][]: `sudo apt-get install ros-kinetic-joy`
- [ROSSerial][]: `sudo apt-get install ros-kinetic-rosserial`
- [ROSSerial_Arduino][]: `sudo apt-get install ros-kinetic-rosserial-arduino`

Requires that the [Arduino][] IDE be installed: `sudo apt-get install arduino`


## Running the code

There is a launch file that starts up all the required ROS nodes:

    roslaunch tycho tycho.launch


## Useful command-line snippets

Rebuilding ROS Arduino libraries:

    source ~/catkin_ws/devel/setup.bash      # Must re-source the workspace, even if you already have?
    rm -rf ~/sketchbook/libraries/ros_lib/   # Need to delete the libraries before rebuilding them
    rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries/


[ROS]: http://wiki.ros.org/ROS/Installation
[Joy]: http://wiki.ros.org/joy
[ROSSerial]: http://wiki.ros.org/rosserial
[ROSSerial_Arduino]: http://wiki.ros.org/rosserial_arduino
[Arduino]: http://arduino.cc
