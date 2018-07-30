# Tycho Rover Code

This project contains the code needed to drive the Tycho prototype lunar rover.  Currently, it is set up to drive a 1/10th-scale model using an XBox 360 controller.

There is also an [OpenSCAD][] file that can be used to 3D print a copy of the scale model.  There are detailed assembly instructions and a parts manifest at the top of that file.  Be warned that the central frame part is larger than most 3D printers- it should be possible to change the scale factor (variable `X`) to a smaller number (e.g. `25.4/11` or `25.4/12` for 1/11 or 1/12th scale, respectively) to make it fit on your printer's bed, although I have not tested that everything fits together at other scales.  You may also need to adjust the `margin` and `innermargin` variables for your printer.

## Dependencies

Requires that [ROS][] be installed (tested on ROS Kinetic Kame on Ubuntu 16.04), and requires the following ROS packages: 
- [Joy][]: `sudo apt-get install ros-kinetic-joy`
- [ROSSerial][]: `sudo apt-get install ros-kinetic-rosserial`
- [ROSSerial_Arduino][]: `sudo apt-get install ros-kinetic-rosserial-arduino`
- [ROSBridge][]: `sudo apt-get install ros-kinetic-rosbridge_suite`

Requires that the [Arduino][] IDE be installed: `sudo apt-get install arduino`

### For the CAN stuff [in progress]

- [CANard][]

    `git clone https://github.com/normaldotcom/CANard.git`
    
    `cd CANard`
    
    `[sudo] python setup.py install`

- (if using SocketCAN) [can-utils][]: `sudo apt install can-utils`
- (if using SocketCAN) slcan kernel module, if not already built [instructions](https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi).

### Firefox

The GUI requires Firefox.  Unfortunately, Firefox 53 and later are broken on Raspberry Pi MATE Ubuntu.  To install, first uninstall Firefox, then download and install version 52:

    sudo apt-get purge firefox
    wget 'http://ports.ubuntu.com/pool/main/f/firefox/firefox_52.0.2+build1-0ubuntu0.12.04.1_armhf.deb'
    sudo dpkg -i firefox_52.0.2+build1-0ubuntu0.12.04.1_armhf.deb
    sudo apt-mark hold firefox

## Running the code

There is a launch file that starts up all the required ROS nodes:

    roslaunch tycho tycho.launch

Once the roslaunch file has finished, go to the `web/index.html` file in Firefox to view the GUI interface.

## Controls

On an XBox 360 controller, the controls are as follows:
- Left stick: Drive and steer
- A button: Enable driving in reverse (otherwise pulling stik back brakes)
- B button: Change steering mode to strafing
- X button: Emergency stop (disabled by allowing stick to return to neutral)


## Useful command-line snippets

Rebuilding ROS Arduino libraries:

    source ~/catkin_ws/devel/setup.bash      # Must re-source the workspace, even if you already have?
    rm -rf ~/sketchbook/libraries/ros_lib/   # Need to delete the libraries before rebuilding them
    rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries/


[ROS]: http://wiki.ros.org/ROS/Installation
[Joy]: http://wiki.ros.org/joy
[ROSSerial]: http://wiki.ros.org/rosserial
[ROSSerial_Arduino]: http://wiki.ros.org/rosserial_arduino
[ROSBridge]: http://wiki.ros.org/rosbridge_suite
[Arduino]: http://arduino.cc
[OpenSCAD]: http://www.openscad.org
[can-utils]: https://github.com/linux-can/can-utils
[CANard]: https://github.com/normaldotcom/CANard
