#!/usr/bin/env python
import rospy
from tycho.msg import RoverDriveCommand, RoverPosition
from math import cos, sin, atan, atan2, sqrt, pi

# Uses a very rough dead-reckoning routine to estimate the current heading
# TODO: Change from a custom message to a builtin position/orientation type
# # May need to use Quaternions in that, which would be obnoxious.
# # See https://answers.ros.org/question/69754/quaternion-transformations-in-python/
# TODO: Change from using commanded travel to estimated actual travel
# TODO: Currently this topic also includes speed.  Should it?
# Author: Robert Wagner


class DeadReckoning:
    def __init__(self):
        rospy.Subscriber("tycho/final_commands", RoverDriveCommand, self.callback)
        self.heading_r = 0.0
        self.roll = 0.0 # Will eventually get from sensors
        self.pitch = 0.0
        self.speed = 0.0
        
        self.northing = 0.0
        self.easting = 0.0
        
        self.timestamp = -1
        
        self.pub = rospy.Publisher('tycho/rover_position_orientation', RoverPosition)
    #
    
    
    
    # Very rough dead-reckoning routine
    def callback(self, data):
        """
        
        """
        
        # Get time delta
        if self.timestamp == -1:
            self.timestamp = data.header.stamp
            return
        else:
            dt = (data.header.stamp - self.timestamp).secs # TODO: Is this in seconds? It should be
            self.timestamp = data.header.stamp
        #
        
        self.speed = data.speed
        distance_travelled = data.speed * dt
        
        
        # Calculate new heading
        if not data.is_strafing:
            if data.turn_center_x > 0:
                turn_direction = -1 # TODO: is this the right sign?
            else:
                turn_direction = 1
            
            turn_radius = sqrt(data.turn_center_x**2 + data.turn_center_y**2)
            turn_angle = distance_travelled / turn_radius
            self.heading_r = self.heading_r + turn_angle * turn_direction
            # TODO: Update position
        else:
            # Strafing: Just update the position
            self.northing = self.northing + distance_travelled*cos(self.heading_r)
            self.easting  = self.easting  + distance_travelled*sin(self.heading_r)
        #
        
        
        self.createMessage()
    #
    
    def createMessage(self):
        print("r/p/y: %.1f/%.1f/%.1f; n/e: %8.1f, %8.1f"%(self.roll, self.pitch, self.heading_r*360.0/pi, self.northing, self.easting))
        
        m = RoverPosition()
        m.header.stamp = rospy.Time.now()
        m.roll  = self.roll
        m.pitch  = self.pitch
        m.heading = self.heading_r*360.0/pi
        m.northing = self.northing
        m.easting   = self.easting
        m.speed   = self.speed
        self.pub.publish(m)
    #
    

# Intializes everything
def start():
    # starts the node
    rospy.init_node('DeadReckoning')
    
    interpreter = DeadReckoning()
    
    rospy.spin()
#

if __name__ == '__main__':
    start()
    
