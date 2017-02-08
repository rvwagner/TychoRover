#!/usr/bin/env python
import rospy
#import std_msgs.msg
from sensor_msgs.msg import Joy
from tycho.msg import RoverDriveCommand
from math import atan2, sqrt, pi, sin, cos

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim


class JoyToCommand:
    def __init__(self):
        rospy.Subscriber("joy", Joy, self.callback)
        
        # Extra dead zone for transition from straight ahead to full strafe
        # In this dead zone, speed=0 and angle is a linear transition from
        # 0 to atan(x,y)
        self.strafeDeadZone = 1.0/2.0;
        
        self.steeringMode = "off"
        
        
        self.joyState = {}
        self.joyState['joyXAxis'] = 0.0
        self.joyState['joyYAxis'] = 0.0
        self.joyState['modeNormal'] = 0
        self.joyState['modeSideways'] = 0
        self.joyState['modeFrontLeft'] = 0
        self.joyState['modeFrontRight'] = 0
        self.joyState['reverseModeButton'] = 0
        self.joyState['stopButton'] = 0
        self.pub = rospy.Publisher('tycho/joystick_commands', RoverDriveCommand)
    #
    
     
    
    # Receives joystick messages (subscribed to Joy topic)
    # then converts the joysick inputs into Twist commands
    # axis 1 aka left stick vertical controls linear speed
    # axis 0 aka left stick horizonal controls angular speed
    def callback(self, data):
        """
        
        """
        def updateValueIfNeeded(newdata, key):
            if self.joyState[key] != newdata:
                self.joyState[key] = newdata
                return True
            return False
        #
        
        # TODO: Why is it never registering the return to 0,0 properly?
        # TODO: Why is stopButton disabling the turning mechanism as well? isBraking.
        # TODO: Why does stopButton stop working if I don't specify isBraking?
        hasChanged = False
        hasChanged = updateValueIfNeeded(data.axes[0], 'joyXAxis') or hasChanged
        hasChanged = updateValueIfNeeded(data.axes[1], 'joyYAxis') or hasChanged
        hasChanged = updateValueIfNeeded(data.buttons[3], 'modeNormal') or hasChanged
        hasChanged = updateValueIfNeeded(data.buttons[0], 'modeSideways') or hasChanged
        hasChanged = updateValueIfNeeded(data.buttons[2], 'modeFrontLeft') or hasChanged
        hasChanged = updateValueIfNeeded(data.buttons[1], 'modeFrontRight') or hasChanged
        
        # Prevent killswitch disable unless joystick is neutral
        if self.joyState['stopButton'] == 0 or (self.joyState['joyXAxis'] == 0 and self.joyState['joyYAxis'] == 0):
            hasChanged = updateValueIfNeeded(data.buttons[8], 'stopButton') or hasChanged
        
        if hasChanged:
            self.interpretJoystick()
    #
    
    def interpretJoystick(self):
        
        # Only switch modes if joystick is centered
        if self.joyState['joyXAxis'] == 0.0 and self.joyState['joyYAxis'] == 0.0:
            if self.joyState['modeNormal'] == 1:
                self.steeringMode = "normal"
            elif self.joyState['modeSideways'] == 1:
                self.steeringMode = "sideways"
            elif self.joyState['modeFrontLeft'] == 1:
                self.steeringMode = "strafeleft"
            elif self.joyState['modeFrontRight'] == 1:
                self.steeringMode = "straferight"
        #
        
        if self.steeringMode == "normal":
            self.interpretNormal()
        elif self.steeringMode == "straferight":
            self.interpretStrafe(45)
        elif self.steeringMode == "strafeleft":
            self.interpretStrafe(-45)
        elif self.steeringMode == "sideways":
            self.interpretStrafe(90)
        return
    #
    
    def interpretNormal(self):
        speed = self.joyState['joyYAxis']
        strafeAngle=0
        turnX = 0
        if self.joyState['joyXAxis'] != 0.0:
            turnY = -1/self.joyState['joyXAxis']
            isStrafing = False
        else:
            turnY = 0
            isStrafing = True
        #
        
        self.publishCommandMessage(speed, turnX, turnY, strafeAngle, isStrafing, isBraking=False);
    #
    
    def interpretStrafe(self, angle):
        speed = -self.joyState['joyXAxis'] * sin(angle*pi/180.0) + self.joyState['joyYAxis'] * cos(angle*pi/180.0)
        
        
        strafeAngle = angle
        
        self.publishCommandMessage(speed, turnX=0, turnY=0, strafeAngle=strafeAngle, isStrafing=True, isBraking=False);
    #
    
    
    def publishCommandMessage(self, speed, turnX, turnY, strafeAngle, isStrafing, isBraking):
        # In case of kill switch, override all other values
        if self.joyState['stopButton'] == 1:
            speed=0
            isBraking=True
        #
        m = RoverDriveCommand()
        m.header.stamp = rospy.Time.now()
        m.speed          = speed
        m.turn_center_x  = turnX
        m.turn_center_y  = turnY
        m.strafing_angle = strafeAngle
        m.is_strafing    = isStrafing
        m.is_braking     = isBraking
        print("%.2f, (%.1f,%.1f), %.0f %s %s"%(speed, turnX, turnY, strafeAngle, isStrafing, isBraking))
        
        self.pub.publish(m)
    #
    
    # RoverDriveCommand is:
    # Header  header         
    # float32 speed          -1 to +1 = min to max full scale
    # float32 turn_center_x  m, + is forward
    # float32 turn_center_y  m, + is left
    # float32 strafing_angle degrees
    # bool    is_strafing
    # bool    is_braking


# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    interpreter = JoyToCommand() # subscribes to joystick inputs on topic "joy"
    
    # starts the node
    rospy.init_node('Joy2Tycho')
    rospy.spin()
#

if __name__ == '__main__':
    start()
    
