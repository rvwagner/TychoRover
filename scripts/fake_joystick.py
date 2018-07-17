#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy


def pub(x,y):
    msg = Joy()
    msg.axes = [x,y]
    msg.buttons = [0,0,0,0,0,0]
    joy_pub.publish(msg)
#

if __name__ == '__main__':
    joy_pub = rospy.Publisher('tycho/joy', Joy, queue_size=1)
    rospy.init_node('FakeJoystick')
    x = 0.0
    y = 0.0
    x_sign = 1
    y_sign = 1
    # Send a new joystick value every 10ms
    rate = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():
        rate.sleep()
        dt = 0.01;
        x += x_sign * dt * 0.5;
        if x >= 1: x=1; x_sign = -1;
        if x <= -1: x=-1; x_sign = 1;
        y += y_sign * dt * 0.05;
        if y >= 1: y=1; y_sign = -1;
        if y <= 0: y=0; y_sign = 1;
        pub(x,y)
    #
#
