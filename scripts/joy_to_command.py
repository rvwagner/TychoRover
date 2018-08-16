#!/usr/bin/env python
import rospy
import time
from enum import Enum
#import std_msgs.msg
from sensor_msgs.msg import Joy
from tycho.msg import RoverDriveCommand
from math import atan2, sqrt, pi, atan

# Author: Robert Wagner
# This ROS Node converts Joystick inputs from the joy node
# into commands for the Tycho rover

# FIXME: Should load this from a parameter file
# Max speed in mm/s
TYCHO_MAX_SPEED = 1000.0
TYCHO_MAX_STRAFE_SPEED = TYCHO_MAX_SPEED / 1
TYCHO_MAX_SPIN_SPEED = 1000.0
TYCHO_MAX_CIRCLE_STRAFE_SPEED = TYCHO_MAX_STRAFE_SPEED

TYCHO_MINIMUM_TURN_RADIUS = 1

TYCHO_DEFAULT_CIRCLE_STRAFE_DISTANCE = 2.5
TYCHO_MINIMUM_CIRCLE_STRAFE_DISTANCE = 1.5
TYCHO_CSTRAFE_ADJUST_RATE = 0.75

class DriveMode(Enum):
    STOP = 0
    NORMAL = 1
    STRAFE = 2
    INPLACE = 3
    CIRCLESTRAFE = 5
#

class JoyToCommand:
    
    def __init__(self):
        rospy.Subscriber("tycho/joy", Joy, self.callback, queue_size=1)
        
        # Extra dead zone for transition from straight ahead to full strafe
        # In this dead zone, speed=0 and angle is a linear transition from
        # 0 to atan(x,y)
        self.strafeDeadZone = 0.3
        
        self.steeringMode = DriveMode.STOP
        self.canReverse = True;
        self.lastNonZeroTime = rospy.Time.now() # Used for buttonless mode updater
        self.lastCStrafeUpdateTime = rospy.Time.now() # Used for getting the right change rate on circle-strafe mode
        self.circleStrafeDistance = TYCHO_DEFAULT_CIRCLE_STRAFE_DISTANCE
        
        # Internal state of the high-level command to send
        # Used to allow stop button to maintain state on steering with minimal
        # interaction with other drive modes
        self.speed = 0
        self.turnX = 0
        self.turnY = 0
        self.strafeAngle = 0
        self.isStrafing  = True
        self.isBraking   = True
        
        
        self.buttonMap = {}
        self.buttonMap['joyXAxis'] = 0
        self.buttonMap['joyYAxis'] = 1
        self.buttonMap['buttonNormal']       = 5
        self.buttonMap['buttonStrafe']       = 1
        self.buttonMap['buttonCircleStrafe'] = 4
        self.buttonMap['buttonReverse']      = 0
        self.buttonMap['buttonTurnInPlace']  = 3
        self.buttonMap['buttonStop']         = 2
        
        self.joyState = {}
        self.joyState['joyXAxis'] = 0.0
        self.joyState['joyYAxis'] = 0.0
        self.joyState['buttonNormal']       = False
        self.joyState['buttonStrafe']       = False
        self.joyState['buttonCircleStrafe'] = False
        self.joyState['buttonReverse']      = False
        self.joyState['buttonTurnInPlace']  = False
        self.joyState['buttonStop']         = False
        self.pub = rospy.Publisher('tycho/joystick_commands', RoverDriveCommand, queue_size=1)
    #
    
    
    def callback(self, data):
        """
        Receives joystick messages (subscribed to Joy topic)
        
        Checks if any values have changed, and if so, sends the message on to
        the interpreter.
        """
        
        # Helper function that determines if a value is new
        # Updates the value and returns True if so, otherwise False
        def updateValueIfNeeded(key, isAxis=False):
            if isAxis:
                newdata = data.axes[ self.buttonMap[key] ]
            else:
                newdata = (data.buttons[ self.buttonMap[key] ] == 1)
            #
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
        hasChanged = updateValueIfNeeded('buttonStrafe') or hasChanged
        hasChanged = updateValueIfNeeded('buttonNormal') or hasChanged
        hasChanged = updateValueIfNeeded('buttonReverse') or hasChanged
        hasChanged = updateValueIfNeeded('buttonTurnInPlace') or hasChanged
        hasChanged = updateValueIfNeeded('buttonCircleStrafe') or hasChanged
        
        # Ignore stop button release unless joystick is neutral
        if self.joyState['buttonStop'] == 0 or (self.joyState['joyXAxis'] == 0 and self.joyState['joyYAxis'] == 0):
            hasChanged = updateValueIfNeeded('buttonStop') or hasChanged
        if hasChanged:
            self.updateDriveMode()
            self.interpretJoystick()
    #
    
    def updateDriveMode(self):
        """
        Uses the current button state to set the self.steeringMode variable
        If multiple buttons are pressed, prioritizes them
        If no buttons are pressed, defaults to DriveMode.NORMAL
        """
        if self.joyState['buttonStop']:
            self.steeringMode = DriveMode.STOP
        elif self.joyState['buttonNormal']:
            self.steeringMode = DriveMode.NORMAL
        elif self.joyState['buttonStrafe']:
            self.steeringMode = DriveMode.STRAFE
        elif self.joyState['buttonTurnInPlace']:
            self.steeringMode = DriveMode.INPLACE
        elif self.joyState['buttonCircleStrafe']:
            if self.steeringMode != DriveMode.CIRCLESTRAFE:
                self.circleStrafeDistance = TYCHO_DEFAULT_CIRCLE_STRAFE_DISTANCE
            self.steeringMode = DriveMode.CIRCLESTRAFE
        else:
            self.steeringMode = DriveMode.NORMAL
        #
        
        print("Set driving mode to %s"%self.steeringMode.name)
    #
    
    def updateDriveModeButtonless(self):
        # Abortive "no-buttons" mode.
        # Normal: push stick forward
        # Strafe: Pull stick a bit back, then push to either side
        # In-Place: Pull stick all the way back, then swing side-to-side
        
        # TODO: Add stop button...
        
        # Alternate option: pull a bit back is in-place, and all the way back switches to an adjustable circle-strafe mode
        self.canReverse = False
        # For extra credit, add a Mouse Mode button combo to activate it.
        if self.steeringMode != DriveMode.STOP and self.joyState['joyXAxis'] == 0.0 and self.joyState['joyYAxis'] == 0.0:
# Make the "auto-off" option require a delay of a second or so...
# (rospy.Time.now() - self.lastNonZeroTime).secs > 1.0 ...
            self.steeringMode = DriveMode.STOP
            return
        elif self.steeringMode == DriveMode.STOP and self.joyState['joyYAxis'] < -0.1:
            self.steeringMode = DriveMode.STRAFE
        elif self.steeringMode == DriveMode.STOP and self.joyState['joyYAxis'] > 0.05:
            self.steeringMode = DriveMode.NORMAL
        elif (self.steeringMode == DriveMode.STOP or self.steeringMode == DriveMode.STRAFE) and self.joyState['joyYAxis'] < 0.95:
            # Change InPlace to be pulling all the way back
            self.steeringMode = DriveMode.INPLACE
        #
        self.lastNonZeroTime = rospy.Time.now()
    #
    
    # TODO: Why don't wheels return forward after leaving turnInPlace?
    def interpretJoystick(self):
        
        
        if self.steeringMode == DriveMode.STOP:
            self.interpretStop()
            
        if self.steeringMode == DriveMode.STRAFE:
            self.interpretStrafe()
            
        elif self.steeringMode == DriveMode.INPLACE:
            self.interpretTurnInPlace()
            
        elif self.steeringMode == DriveMode.CIRCLESTRAFE:
            self.interpretCircleStrafe()
            
        elif self.steeringMode == DriveMode.NORMAL:
            self.interpretNormal()
            
        else:
            self.interpretStop()
        #
    #
    
    def scaleAndLimitSpeed(self, raw_speed, max_speed):
        if raw_speed >  1: raw_speed =  1.0
        if raw_speed < -1: raw_speed = -1.0
        return raw_speed * max_speed
    #
    
    # Stop movement, but don't touch the wheel angles
    def interpretStop(self):
        self.isBraking   = True
        self.speed = 0
        self.publishCommandMessage()
    #
    
    
    # Drive like a normal car
    def interpretNormal(self):
        print('Using interpretNormal()')
        self.strafeAngle = 0
        self.turnX = 0
        self.isBraking = False
        
        # Set speed
        self.speed = self.scaleAndLimitSpeed(self.joyState['joyYAxis'], TYCHO_MAX_SPEED)
        print ("Limited %.2f to %.2f: %.2f"%(self.joyState['joyYAxis'], TYCHO_MAX_SPEED, self.speed) )
        
        # Set steering values depending on joystick left/right axis
        if (self.joyState['joyXAxis'] != 0.0):
            # TODO: Maybe a different function? (sqrt?)
            # TODO: Set an appropriate minimum turn radius?  Currently 1m, kind of by accident
            self.turnY = -1/self.joyState['joyXAxis'] * TYCHO_MINIMUM_TURN_RADIUS
            self.isStrafing = False
        else:
            self.turnY = 0
            self.isStrafing = True
        #
         
        # Request brake engagement if needed
        # This isn't actually good, as it will prevent steering while stopped...
        #if self.speed == 0.0 or self.joyState['buttonStop']:
        #    self.isBraking = True
        #
        
        # Block backwards driving if reverse mode is off
        if self.speed < 0 and not self.canReverse:
            self.isBraking = True # Should this exist?
            self.speed = 0.0
        #
        
        # Rotate the steering axis so that strafeAngle is forward
        # Not actually useful, but fun.
        #strafeAngle = 45
        #turnX = turnY*sin(strafeAngle*pi/180.0)
        #turnY = turnY*cos(strafeAngle*pi/180.0)
        
        self.publishCommandMessage()
    #
    
    # Drive straight in any direction
    def interpretStrafe(self):
        print('Using interpretStrafe()')
        
        # Set speed
        speed = sqrt(self.joyState['joyXAxis']**2 + self.joyState['joyYAxis']**2)
        speed = self.scaleAndLimitSpeed(speed, 1.0) # TODO: Change this to limit by maximum possible value in the chosen direction?
        isBraking = False
        print ("Base speed: %.2f"%(speed) )
        
        if self.joyState['joyYAxis'] != 0:
            strafeAngle = atan(-self.joyState['joyXAxis'] / self.joyState['joyYAxis'])*180/pi
        else:
            strafeAngle = -90.0 * min(max(-self.strafeDeadZone, self.joyState['joyXAxis']), self.strafeDeadZone) / self.strafeDeadZone
        print ("Base angle: %.2f"%(strafeAngle) )
        
        # Adjust for deadzone
        if abs(speed) < self.strafeDeadZone: # Near deadzone, slowly shift towards sideways
        # TODO: deadzone should actually be a pair of wedges going to the sides
        # the speed shouldn't cut off at the edge of the deadzone
        #if speed < self.strafeDeadZone and abs(self.joyState['joyXAxis']) > abs(self.joyState['joyYAxis']):
            strafeAngle = (speed/self.strafeDeadZone)*strafeAngle
            speed = 0
        else:
            speed = (speed-self.strafeDeadZone) / (1-self.strafeDeadZone)
        #
        speed = self.scaleAndLimitSpeed(speed, TYCHO_MAX_STRAFE_SPEED)
        print ("Adjusted speed: %.2f, Angle: %.2f"%(speed, strafeAngle) )
        
        # TODO If reverse mode active
        if self.joyState['joyYAxis'] < 0:
            if self.canReverse:
                speed = -speed
            else:
                speed = 0
                isBraking = True
            #
        #
        
        # TODO: What is this fixing?
        if abs(self.joyState['joyXAxis']) < 0.05 or abs(strafeAngle > 90):
            strafeAngle = 0.0
        #
        
        self.updateFullMessage(speed=speed, turnX=0, turnY=0, strafeAngle=strafeAngle, isStrafing=True, isBraking=isBraking);
        self.publishCommandMessage()
    #
    
    # Spin about the center of the rover
    # TODO: Might want to change this to the location of a camera onboard the rover for perfect panoramas
    def interpretTurnInPlace(self):
        print('Using interpretTurnInPlace()')
        speed = self.joyState['joyXAxis']
        speed = self.scaleAndLimitSpeed(self.joyState['joyXAxis'], TYCHO_MAX_SPIN_SPEED)
        
        #if speed == 0.0 or self.joyState['buttonStop']:
        #    isBraking = True
        #else:
        #    isBraking = False
        #
        
        self.updateFullMessage(speed, turnX=0, turnY=0, strafeAngle=0, isStrafing=False, isBraking=False);
        self.publishCommandMessage()
    #
    
    # Spin about a point in front of the rover
    def interpretCircleStrafe(self):
        print ('Using interpretCircleStrafe()')
        speed = -self.joyState['joyXAxis']
        speed = self.scaleAndLimitSpeed(-self.joyState['joyXAxis'], TYCHO_MAX_CIRCLE_STRAFE_SPEED)
        
        # Adjust circle-strafing distance using forward-backward joystick position
        # (with a big deadzone!)
        time = rospy.Time.now() 
        dt = (time - self.lastCStrafeUpdateTime).to_sec()
        self.lastCStrafeUpdateTime = time
        
        deadzone = 0.5
        if self.joyState['joyYAxis'] > deadzone:
            self.circleStrafeDistance += dt * TYCHO_CSTRAFE_ADJUST_RATE * (self.joyState['joyYAxis']-deadzone)/(1-deadzone)
        if self.joyState['joyYAxis'] < -deadzone and self.circleStrafeDistance >= TYCHO_MINIMUM_CIRCLE_STRAFE_DISTANCE:
            self.circleStrafeDistance += dt * TYCHO_CSTRAFE_ADJUST_RATE * (self.joyState['joyYAxis']+deadzone)/(1-deadzone)
        #
        if self.circleStrafeDistance <= TYCHO_MINIMUM_CIRCLE_STRAFE_DISTANCE:
            self.circleStrafeDistance = TYCHO_MINIMUM_CIRCLE_STRAFE_DISTANCE
        #
        
        self.updateFullMessage(speed, turnX=self.circleStrafeDistance, turnY=0, strafeAngle=0, isStrafing=False, isBraking=False);
        self.publishCommandMessage()
    #
    
    # Update all values of saved command state at once
    def updateFullMessage(self, speed, turnX, turnY, strafeAngle, isStrafing, isBraking):
        self.speed = speed
        self.turnX = turnX
        self.turnY = turnY
        self.strafeAngle = strafeAngle
        self.isStrafing  = isStrafing
        self.isBraking   = isBraking
     #
    
    # Publish command based on the saved state
    def publishCommandMessage(self):
        # In case of kill switch, override all other values
        # Should never be needed
        if self.joyState['buttonStop']:
            speed=0
            isBraking=True
        #
        m = RoverDriveCommand()
        m.header.stamp = rospy.Time.now()
        m.speed          = self.speed
        m.turn_center_x  = self.turnX
        m.turn_center_y  = self.turnY
        m.strafing_angle = self.strafeAngle
        m.is_strafing    = self.isStrafing
        m.is_braking     = self.isBraking
        print("%.2f, (%.1f,%.1f), %.0f %s %s"%(self.speed, self.turnX, self.turnY, self.strafeAngle, self.isStrafing, self.isBraking))
        
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
    rospy.init_node('Joy2Tycho')
    interpreter = JoyToCommand() # subscribes to joystick inputs on topic "joy"
    
    # starts the node
    rospy.spin()
#

if __name__ == '__main__':
    start()
#
