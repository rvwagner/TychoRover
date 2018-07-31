#!/usr/bin/env python
import rospy
from tycho.msg import PowerMonitor


def pub(x,y):
    
#

if __name__ == '__main__':
    msg = PowerMonitor()
    msg.battery_1_v = 0
    msg.battery_2_v = 0
    msg.battery_3_v = 12.0
    msg.battery_4_v = 24.0
    msg.panel_12v_v = 13.8
    msg.panel_5v_v  = 4.95
    msg.master_a        = 2.0
    msg.front_relay_a   = 0.5
    msg.rear_relay_a    = 0.5
    msg.panel_display_a = 2.0
    msg.panel_12v_a     = 2.0
    msg.panel_5v_a      = 1.5
    
    rospy.init_node('FakeJoystick')
    joy_pub = rospy.Publisher('tycho/joy', Joy, queue_size=1)
    
    x = 0.0
    x_sign = 1
    # Send a new joystick value every 10ms
    rate = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():
        rate.sleep()
        dt = 0.01;
        x += x_sign * dt;
        if x >= 1: x=1; x_sign = -1;
        if x <= -1: x=-1; x_sign = 1;
        
        
        msg.battery_3_v += x * 0.5 * dt
        msg.battery_4_v += x * 0.5 * dt
        msg.panel_12v_v += x * 0.05 * dt
        msg.panel_5v_v  += x * 0.05 * dt
        msg.master_a        += x * 1.0 * dt
        msg.front_relay_a   += x * 0.1 * dt
        msg.rear_relay_a    += x * 0.1 * dt
        msg.panel_display_a += x * 1.0 * dt
        msg.panel_12v_a     += x * 1.0 * dt
        msg.panel_5v_a      += x * 0.5 * dt
        
        joy_pub.publish(msg)
    #
#
