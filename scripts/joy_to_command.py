#!/usr/bin/env python
import rospy
#import std_msgs.msg
from sensor_msgs.msg import Joy
from tycho.msg import RoverDriveCommand
from math import atan2, sqrt, pi

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
        

        self.buttonMap = {}
        self.buttonMap['joyXAxis'] = 0
        self.buttonMap['joyYAxis'] = 1
        self.buttonMap['modeSideways'] = 1
        self.buttonMap['modeCircleStrafe'] = 4
        self.buttonMap['modeReverse'] = 0
	#Testing modeNeutral option for turnInPlace - Jacob Hicks
	self.buttonMap['modeNeutral'] = 3
        self.buttonMap['stopButton'] = 2
        
        self.joyState = {}
        self.joyState['joyXAxis'] = 0.0
        self.joyState['joyYAxis'] = 0.0
        self.joyState['modeSideways'] = 0
        self.joyState['modeCircleStrafe'] = 0
        self.joyState['modeReverse'] = 0
	#Testing modeNeutral option for turnInPlace - Jacob Hicks
	self.joyState['modeNeutral'] = 0
        self.joyState['stopButton'] = 0
        self.pub = rospy.Publisher('tycho/joystick_commands', RoverDriveCommand)

    # Receives joystick messages (subscribed to Joy topic)
    # then converts the joysick inputs into Twist commands
    # axis 1 aka left stick vertical controls linear speed
    # axis 0 aka left stick horizonal controls angular speed
    def callback(self, data):
        """
        
        """
        def updateValueIfNeeded(key, isAxis=False):
            if isAxis:
                newdata = data.axes[ self.buttonMap[key] ]
            else:
                newdata = data.buttons[ self.buttonMap[key] ]
            if self.joyState[key] != newdata:
                self.joyState[key] = newdata
                return True
            return False
        #
        
        # TODO: Why is it never registering the return to 0,0 properly?
        # TODO: Why is stopButton disabling the turning mechanism as well? isBraking.
        # TODO: Why does stopButton stop working if I don't specify isBraking?
        hasChanged = False
        hasChanged = updateValueIfNeeded('joyXAxis', isAxis=True) or hasChanged
        hasChanged = updateValueIfNeeded('joyYAxis', isAxis=True) or hasChanged
        hasChanged = updateValueIfNeeded('modeSideways') or hasChanged
        hasChanged = updateValueIfNeeded('modeReverse') or hasChanged
	hasChanged = updateValueIfNeeded('modeNeutral') or hasChanged
	hasChanged = updateValueIfNeeded('modeCircleStrafe') or hasChanged
        
        # Prevent killswitch disable unless joystick is neutral
        if self.joyState['stopButton'] == 0 or (self.joyState['joyXAxis'] == 0 and self.joyState['joyYAxis'] == 0):
            hasChanged = updateValueIfNeeded('stopButton') or hasChanged

        if hasChanged:
            self.interpretJoystick()
    #
    
    # TODO: Why don't wheels return forward after leaving turnInPlace?
    def interpretJoystick(self):
	
        if self.joyState['modeSideways'] == 1:
	    print('sending to interpretStrafe()')
	    self.interpretStrafe()

	elif self.joyState['modeNeutral'] == 1:
	    isBraking = True
	    if abs(self.joyState['joyYAxis'] == 0):
		print('sending to interpretTurnInPlace()')
	        self.interpretTurnInPlace()
	elif self.joyState['modeCircleStrafe'] == 1:
	    print ('sending to interpretCircleStrafe()')
	    self.interpretCircleStrafe()
	elif self.joyState['modeReverse'] == 1:
	    print('sending to resetWheelState()')
	    self.resetWheelState()
	else:
            self.turnY = 0
	    self.isStrafing = True
	    print('sending to interpretNormal()')
	    self.interpretNormal()

        return
        
	
        if self.steeringMode != "off" and self.joyState['joyXAxis'] == 0.0 and self.joyState['joyYAxis'] == 0.0:
	    self.steeringMode = "off"
            self.joyState['modeReverse'] = 1
	    self.interpretNormal()
        elif self.steeringMode == "off" and self.joyState['joyYAxis'] < -0.5:
            self.steeringMode = "strafe"
        elif self.steeringMode == "off" and self.joyState['joyYAxis'] > 0.15:
            self.steeringMode = "normal"
        elif self.steeringMode == "off" and abs(self.joyState['joyXAxis']) > 0.1:
	    self.steeringMode = "inplace"
        #
        
        print(self.steeringMode)
        if self.joyState['modeSideways'] == 1 or self.steeringMode == "strafe":
            self.interpretStrafe()
        elif self.steeringMode == "normal":
            self.interpretNormal()
        elif self.steeringMode == "inplace":
            self.interpretTurnInPlace()
        else:
            pass
        #
    #
    
    def resetWheelState(self):
	
	speed = self.joyState['joyYAxis']
        strafeAngle=0
        turnX = 0

	self.publishCommandMessage(speed=0, turnX=0.01, turnY=0.01, strafeAngle=0.01, isStrafing=True, isBraking=True);

    #

    def interpretNormal(self):
	print('reached interpretNormal()')
        speed = self.joyState['joyYAxis']
        strafeAngle=0
        turnX = 0

	#

        if (self.joyState['joyXAxis'] != 0.0):
            turnY = -1/self.joyState['joyXAxis']
            isStrafing = False
	else:
	    turnY = 0
	    isStrafing = True
	#
	 
	# Added to brake the rover if joystick is in neutral pos. -Jacob Hicks
	if (self.joyState['joyYAxis'] == 0) or (self.joyState['stopButton'] == 1):
	    isBraking = True

	#

	# Put rover into reverse mode if joystick is oriented backwards. -Jacob Hicks
	if (self.joyState['joyYAxis'] < 0):
	    self.joyState['modeReverse'] = 1

        # Rotate the steering axis so that strafeAngle is forward
        # Not actually useful, but fun.
        #strafeAngle = 45
        #turnX = turnY*sin(strafeAngle*pi/180.0)
        #turnY = turnY*cos(strafeAngle*pi/180.0)
        
        self.publishCommandMessage(speed, turnX, turnY, strafeAngle, isStrafing, isBraking);
    #
    
    def interpretStrafe(self):
	print('reached interpretStrafe()')
        speed = sqrt(self.joyState['joyXAxis']**2 + self.joyState['joyYAxis']**2)
        if self.joyState['joyYAxis'] < 0: speed = abs(self.joyState['joyXAxis'])
        
        strafeAngle = atan2(-self.joyState['joyXAxis'], self.joyState['joyYAxis'])*180/pi
        if speed < self.strafeDeadZone: # Near deadzone, slowly shift towards sideways
            strafeAngle = (speed/self.strafeDeadZone)*strafeAngle
            speed = 0
        else:
            speed = (speed-self.strafeDeadZone) #/(1-self.strafeDeadZone)
        #
        
        # TODO If reverse mode active
        #if self.joyState['joyYAxis'] < 0: speed = -speed
        if self.joyState['joyYAxis'] == 0:
	    speed = 0
            isBraking = True
        else:
            isBraking = False

        if abs(self.joyState['joyXAxis']) < 0.05 or abs(strafeAngle > 90):
            strafeAngle = 0.0
        #

        isBraking = (self.joyState['modeReverse'] == 1)
        self.publishCommandMessage(speed, turnX=0, turnY=0, strafeAngle=strafeAngle, isStrafing=True, isBraking=isBraking);
    #
    
    def interpretTurnInPlace(self):
	print('reached interpretTurnInPlace()')
	speed = self.joyState['joyXAxis']
	
        isBraking = (self.joyState['joyYAxis'] <= -0.75)
        self.publishCommandMessage(speed, turnX=0, turnY=0, strafeAngle=0, isStrafing=False, isBraking=isBraking);
    #

    def interpretCircleStrafe(self):
	print('reached interpretCircleStrafe()')
	speed = self.joyState['joyXAxis']
	
	isBraking = (self.joyState['joyYAxis'] <= -0.75)
        self.publishCommandMessage(speed, turnX=2, turnY=0, strafeAngle=0, isStrafing=False, isBraking=isBraking);

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
    
