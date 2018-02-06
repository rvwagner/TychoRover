#!/usr/bin/env python
import rospy
from tycho.msg import WheelStatus, RoverPosition, DisplayPanel
from math import atan, atan2, sqrt, pi

# Builds up packets to send to the display panel
# Sends combined packets at a fixed rate, based on the most recent data
# from the various sensors
# TODO: Send immediate packets for error and warning states
# Author: Robert Wagner

update_delay = 0.1 # seconds between updates


class CommandToAngles:
    def __init__(self):
        rospy.Subscriber("tycho/wheel_status", WheelStatus, self.wheel_callback)
        rospy.Subscriber("tycho/rover_position_orientation", RoverPosition, self.pose_callback)
        
        # Direct message parts
        self.speed               = 0.0
        self.front_left_angle    = 0.0
        self.front_right_angle   = 0.0
        self.back_left_angle     = 0.0
        self.back_right_angle    = 0.0
        self.roll                = 0.0
        self.pitch               = 0.0
        self.heading             = 0.0
        
        # Helper variables
        self.last_pose_time      = -1
        self.last_wheel_times    = [self.last_pose_time,self.last_pose_time,self.last_pose_time,self.last_pose_time]
        self.max_amp_hours       = 100.0
        self.remaining_amp_hours = self.max_amp_hours # TODO: Handle battery percent better
        self.amp_list            = [0.,0.,0.,0.]
        self.drive_temp_list     = [0.,0.,0.,0.]
        
        self.pub = rospy.Publisher('tycho/display_values', DisplayPanel)
    #
    
    
    
    # Extract data from WheelStatus messages
    def wheel_callback(self, data):
        """
        
        """
        
        if self.last_wheel_times[data.wheel_id-1] == -1:
            self.last_wheel_times[data.wheel_id-1] = data.header.stamp
            return
        else:
            dt = (data.header.stamp - self.last_wheel_times[data.wheel_id-1]).secs # TODO: Is this in seconds? It should be
            self.last_wheel_times[data.wheel_id-1] = data.header.stamp
        #
        
        if data.wheel_id == 1:   # Front-left
            self.front_left_angle = data.steering_angle
        elif data.wheel_id == 2: # Front-right
            self.front_right_angle = data.steering_angle
        elif data.wheel_id == 3: # Back-left
            self.back_left_angle = data.steering_angle
        elif data.wheel_id == 4: # Back-right
            self.back_right_angle = data.steering_angle
        #
        
        # TODO: Handle remaining amp-hours better
        self.amp_list[data.wheel_id-1] = data.drive_amps + data.steering_amps
        self.remaining_amp_hours      -= dt*self.amp_list[data.wheel_id-1]/3600.0
        
        # TODO: Maybe allow for non-drive motors being hottest?
        self.drive_temp_list[data.wheel_id-1] = data.drive_temp
        
        # TODO: Derive speed from avg RPM and wheel diameter?
        # TODO: Should we display distance driven?
        # TODO: Should we display crows-flight distance to start?
        
        # TODO: Should flag states be determined here, or on the Arduino?
        
        # TODO: Immediate send for error conditions, e.g. T>90
        if max(data.drive_temp, data.steering_temp)>= 90:
            self.publishMessage()
    #
    
    
    def pose_callback(self, data):
        """
        
        """
        
        if self.last_pose_time == -1:
            self.last_pose_time = data.header.stamp
            return
        else:
            dt = (data.header.stamp - self.last_pose_time).secs
            self.last_pose_time = data.header.stamp
        #
        
        self.speed   = data.speed # TODO: Derive speed from avg RPM and wheel diameter?
        self.roll    = data.roll
        self.pitch   = data.pitch
        self.heading = data.heading
    #
    
    
    def publishMessage(self):
        m = DisplayPanel()
        m.header.stamp = rospy.Time.now()
        m.speed             = self.speed
        m.front_left_angle  = self.front_left_angle
        m.front_right_angle = self.front_right_angle
        m.back_left_angle   = self.back_left_angle
        m.back_right_angle  = self.back_right_angle
        m.total_amps        = sum(self.amp_list)
        m.battery_pct       = self.remaining_amp_hours/self.max_amp_hours
        m.max_drive_temp    = max(self.drive_temp_list)
        m.hottest_motor_id  = self.drive_temp_list.index(m.max_drive_temp) + 1
        m.roll              = self.roll
        m.pitch             = self.pitch
        m.heading           = self.heading
        
        self.pub.publish(m)
    #
    

# Intializes everything
def start():
    interpreter = CommandToAngles() # subscribes to joystick inputs on topic "joy"
    
    # starts the node
    rospy.init_node('Model2Raw')
    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(update_delay)
        interpreter.publishMessage()
#

if __name__ == '__main__':
    start()
    
