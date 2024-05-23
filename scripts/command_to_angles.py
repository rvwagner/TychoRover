#!/usr/bin/env python

# This node converts overarching "go this direction at this speed" commands into specific
# orientations and drive speeds for each wheel.
#
# Input topic: "tycho/final_commands", type RoverDriveCommand
#
# Output topic: "tycho/low_level_motor_values", type WheelAnglesSpeeds
#
# Author: Robert Wagner

import rospy
from tycho.msg import RoverDriveCommand, WheelAnglesSpeeds
from math import atan, atan2, sqrt, pi, sin, cos


# Tycho physical dimensions and limits
# TODO: Move into a config file, 
# Note that apparently the mini-Tycho and full-size Tycho are no longer perfectly to scale
# with each other, so mini-tycho ("Tyke") should get its own parameter file.
TYCHO_MAX_WHEEL_SPEED = 3667 # mm/s, matches speed controller limit of 220 m/minute
# I think this is how system-wide parameter reading works in ROS if you load
# a parameter file in the launch file:
# TYCHO_MAX_WHEEL_SPEED = rospy.get_param("/tycho/speeds/max_wheel")

TYCHO_STEERING_RATE = 60 # Currently unused, should be degrees/sec that steering motors move at. 60 is a rough guess

# The following dimensions are used to define where the wheels are in 2D space
# The code assumes that the vehicle is symmetrical. If it is not, then you'll need to
# edit the self.jointX, etc. lines below.  Possibly put the exact locations/arm lengths
# in the parameter file, instead of these more general measures.  Similarly, change the
# "is turning point inside vehicle" check in setAnglesFromTurnCenter
TYCHO_WHEELBASE = 2.62 # Meters "axle"-to-"axle", may be off a bit
TYCHO_STEERING_WIDTH = 1.115 # Meters between left and right steering pivots, should be accurate
TYCHO_STEER_ARM_LENGTH = 0.31115 # Meters from center of steering pivot to center of wheel

# Rover reference frame is theoretically this:
# +X is forward, +Y is left, numbers in scale meters
# But I think +X might actually be backwards, and possibly all the wheel names are wrong?
# Test by putting turnX at -TYCHO_WHEELBASE/2, see if it steers like a normal car.

# This class contains all the code for this node
class CommandToAngles:
    def __init__(self):
        rospy.Subscriber("tycho/final_commands", RoverDriveCommand, self.callback, queue_size=1)
        self.pub = rospy.Publisher('tycho/low_level_motor_values', WheelAnglesSpeeds, queue_size=1)
        
        #####################
        # Dimensional Model #
        #####################
        
        self.maxWheelSpeed = TYCHO_MAX_WHEEL_SPEED
        self.steerRate = 36.0 # Approximate speed of steering motors in deg/sec
        # TODO: On big steering changes, try to synchronize wheel turning so that wheels 
        # are always on matching arcs- will avoid skidding.
        self.jointX = {'FrontLeft': TYCHO_WHEELBASE/2, 'FrontRight': TYCHO_WHEELBASE/2,
                       'BackLeft': -TYCHO_WHEELBASE/2, 'BackRight': -TYCHO_WHEELBASE/2}
        self.jointY = {'FrontLeft': TYCHO_STEERING_WIDTH/2, 'FrontRight': -TYCHO_STEERING_WIDTH/2,
                       'BackLeft':  TYCHO_STEERING_WIDTH/2, 'BackRight':  -TYCHO_STEERING_WIDTH/2}
        self.steeringArmLength = {  'FrontLeft':  TYCHO_STEER_ARM_LENGTH,
                                    'FrontRight': TYCHO_STEER_ARM_LENGTH,
                                    'BackLeft':   TYCHO_STEER_ARM_LENGTH,
                                    'BackRight':  TYCHO_STEER_ARM_LENGTH}
        self.steerRate = TYCHO_STEERING_RATE
        
        # Storage variables for the last input command
        self.isStrafing = True
        self.strafingAngle = 0.0
        self.turnX = 0.0
        self.turnY = 0.0
        self.speed = 0.0
        
        # Internal variables for calculating/storing speeds to command
        # Speed multiplier is the amount that wheel needs to move faster or slower than
        # vehicle average speed, depending of differing arc lengths
        self.speedMultipliers = {}
        self.speedMultipliers['FrontLeft']  = 1.0
        self.speedMultipliers['FrontRight'] = 1.0
        self.speedMultipliers['BackLeft']   = 1.0
        self.speedMultipliers['BackRight']  = 1.0
        
        self.wheelAngles = {}
        self.wheelAngles['FrontLeft']  = 0.0
        self.wheelAngles['FrontRight'] = 0.0
        self.wheelAngles['BackLeft']   = 0.0
        self.wheelAngles['BackRight']  = 0.0
        
        self.wheelSpeeds = {}
        self.wheelSpeeds['FrontLeft']  = 0.0
        self.wheelSpeeds['FrontRight'] = 0.0
        self.wheelSpeeds['BackLeft']   = 0.0
        self.wheelSpeeds['BackRight']  = 0.0
        
        # System to rate-limit messages
        self.messageFrequency = 30 # Hz
        self.hasNewMessage = False
        self.lastMessageTime = rospy.Time(0)
        self.messageInterval = 1.0/self.messageFrequency
        
        self.m = WheelAnglesSpeeds()
    #
    
    
    # Receives high-level commands (RoverDriveCommand)
    # then converts the joysick inputs into wheel commands (WheelAnglesSpeeds)
    def callback(self, data):
        # If parking brake should be on, set speed to 0 instead of anything fancy
        # TODO: This may be the cause of the wheels centering on first boot?
        # Or it's somewhere in the motor controller.  Possibly just need to not 
        # send new commands at all if in "park" mode?
        if data.is_braking:
            self.speed = 0
            self.setVehicleSpeed(self.speed)
            self.createAndPublishCommandMessage()
            return
        #
        
        # Check if the input data has changed, if not, do nothing
        if data.is_strafing:
            dataIsNew = (data.speed != self.speed or data.strafing_angle != self.strafingAngle) or not self.isStrafing
        else:
            dataIsNew = (data.speed != self.speed or data.turn_center_x != self.turnX or data.turn_center_y != self.turnY) or self.isStrafing
        #
        if not dataIsNew:
        	return
        #
        
        self.speed = data.speed
        
        if data.is_strafing: # Moving in a straight line in some direction
            print("Strafing, %.2f"%(data.strafing_angle) )
            self.isStrafing = True
            self.strafingAngle = data.strafing_angle
            
            # First set wheel angles and speed multipliers
            self.setAnglesStrafing(data.strafing_angle)
            
            # Then set wheel speeds
            self.setVehicleSpeed(self.speed)
            
            # Then queue up the new message
            self.createAndPublishCommandMessage()
        else: # Moving about some arc
            print("Not strafing, %.2f %.2f"%(data.turn_center_x,data.turn_center_y) )
            self.isStrafing = False
            self.turnX = data.turn_center_x
            self.turnY = data.turn_center_y
            self.setAnglesFromTurnCenter(data.turn_center_x, data.turn_center_y)
            self.setVehicleSpeed(self.speed)
            self.createAndPublishCommandMessage()
        #
    #
    
    # Set the wheel angles to be commanded in the next message.
    # Returns the estimated time until they all reach that angle, in seconds
    # This estimate is based on the last command, not the current position, 
    # so it's probably useless. At any rate, I'm not currently using it.
    # TODO: Have this node subscribe to the motor controller return values, store/use those.
    def setSteeringAngles(self, fl, fr, bl, br):
        # Get maximum change in angle for calculating update duration
        maxAngleChange = abs(fl - self.wheelAngles['FrontLeft'])
        change = abs(fr - self.wheelAngles['FrontRight'])
        if change > maxAngleChange: maxAngleChange = change
        change = abs(bl - self.wheelAngles['BackLeft'])
        if change > maxAngleChange: maxAngleChange = change
        change = abs(br - self.wheelAngles['BackRight'])
        if change > maxAngleChange: maxAngleChange = change
  		
  		# Actually set the values
        self.wheelAngles['FrontLeft']  = fl
        self.wheelAngles['FrontRight'] = fr
        self.wheelAngles['BackLeft']   = bl
        self.wheelAngles['BackRight']  = br

        return maxAngleChange/self.steerRate
    #
    
    # Command the steering motors to go to appropriate angles for the vehicle to turn
    # about the specified position in x/y space (+X is forward, +Y is left, units in m)
    # Also adjusts the speed multipliers to be correct for the differing ditances from
    # the turning center. 
    # Returns estimated time until the wheels reach the target angles in seconds.
    # Time estimate is from setSteeringAngles, and is useless and unused
    def setAnglesFromTurnCenter(self, turnCenterX, turnCenterY):
        angles = {};
        turnPointDistance = sqrt(turnCenterX*turnCenterX + turnCenterY*turnCenterY);
        maxMultiplier = 0.0;
        
        for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
            distY = turnCenterY-self.jointY[i];
            distX = turnCenterX-self.jointX[i];
            angles[i] = atan(distX/distY)*180/pi+90;
            circlingPoint = False # circlingPoint should be true if the point is in inside the vehicle bounds
            # this variable seems to be doing nothing- the same bounds check is done below.
            # TODO: check vehicle behaviour with turning points manually set at
            # 2,2, 2,0, -2,0, 0,2, 0,0, 0,-2, 2,-2, 0,-2, -2,-2
            # and if they all work as expected, remove circlingPoint.
            
            if abs(turnCenterX) > TYCHO_WHEELBASE/2 and abs(turnCenterY) < TYCHO_STEERING_WIDTH/2:
                circlingPoint = True
            
            # Calculate the distance to the turning point based on whether
            # steering arm points towards or away from the turn point
            thisWheelDistance = sqrt(distX*distX + distY*distY);
            if turnCenterY/self.jointY[i] > 1:
                # Same sign and abs(turnCenterY) > abs(self.jointY)
                # -> Arm points towards point
                thisWheelDistance -= self.steeringArmLength[i]
            else: # Arm points away from point
                thisWheelDistance += self.steeringArmLength[i]
            #

            # For very small offsets from turn-in-place, make all wheel speeds match here to avoid asymtote issues
            if turnPointDistance <= 0.01:
                self.speedMultipliers[i] = 1.0;
            else:
                self.speedMultipliers[i] = thisWheelDistance/turnPointDistance;
            if self.speedMultipliers[i] > maxMultiplier: # Store maximum for later potential capping
                maxMultiplier = self.speedMultipliers[i];
        # end "for each motor" loop
        
        # If turn point is interior to wheels, adjust multipliers to:
        # A) Max out at 1.0
        # B) Reverse the left-hand wheels
        if (abs(turnCenterX) > TYCHO_WHEELBASE/2 and abs(turnCenterY) < TYCHO_STEERING_WIDTH/2) or circlingPoint == True:

            self.speedMultipliers['FrontLeft']  =  -self.speedMultipliers['FrontLeft']  / maxMultiplier;
            self.speedMultipliers['FrontRight'] =  self.speedMultipliers['FrontRight'] / maxMultiplier;
            self.speedMultipliers['BackLeft']   =  -self.speedMultipliers['BackLeft']   / maxMultiplier;
            self.speedMultipliers['BackRight']  =  self.speedMultipliers['BackRight']  / maxMultiplier;
           
        #
        
        # Save the wheel angles for the next message
        # Apply a -90 offset because the axel is perpendicular to the direciton of travel
        # (aka the -90 is empirically needed, don't worry about it)
        return self.setSteeringAngles(angles['FrontLeft']-90, angles['FrontRight']-90,
                                      angles['BackLeft']-90, angles['BackRight']-90);
    #

    # Set the steering angle to put all wheels parallel at the specified angle
    # Valid angles go from -90 to +90, with 0 being straight ahead and +90 being to the left
    # Returns estimated time until the wheels reach the target angles in s.
    # Time estimate is from setSteeringAngles, and is useless and unused
    def setAnglesStrafing(self, angle):
        # No speed multiplication needed in this drive mode
        self.speedMultipliers['FrontLeft']  = 1.0
        self.speedMultipliers['FrontRight'] = 1.0
        self.speedMultipliers['BackLeft']   = 1.0
        self.speedMultipliers['BackRight']  = 1.0
  
        # Put a bit of toe-in on the wheels (suggested by Chris Skiba)
        # TODO: Do some fancy math, with amount of toe-in based on vehicle orientation/wheel position?
        # FOr now, just disabling it if the vehicle isn't driving straight forward/backward
        # See currently-unused setToeInAngles function below
        if angle == 0:
        	toein = 1.5
        else:
        	toein = 0
        if self.speed > 0: # Reverse
            toein = -toein
        #
        
        # Apply the angles
        return self.setSteeringAngles(angle+toein, angle-toein, angle+toein, angle-toein);
    #
    
    
    # Set the steering angle to put all wheels sub-parallel at the specified angle
    # Angles wheels to point towards a point 50m away in the desired direction
    # TODO: Use speed +/- to switch toe-in direction (center x/y sign)
    # Valid angles go from -90 to +90, with 0 being straight ahead and +90 being to the left
    # (+X is forward, +Y is left, units in m)
    # I don't remember what this function does.  It's unused, so it probably doesn't work right.
    def setToeInAngles(self, angle=1.5):
        turnCenterX = 50*sin(angle*pi/180);
        turnCenterY = 50*cos(angle*pi/180);
        angles = {};
        turnPointDistance = sqrt(turnCenterX*turnCenterX + turnCenterY*turnCenterY);
        maxMultiplier = 0.0;
        
        for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
            distY = turnCenterY-self.jointY[i];
            distX = turnCenterX-self.jointX[i];
            angles[i] = atan(distX/distY)*180/pi+90;
            
            self.speedMultipliers[i] = 1.0;
        #
        print(angles)
        # Actually command the servos
        return self.setSteeringAngles(angles['FrontLeft']+0, angles['FrontRight']+0,
                                      angles['BackLeft']+0, angles['BackRight']+0);
    #
	
    # Set the motor speeds such that the center of the vehicle travels at the commanded speed.
    # Speed values are in meters per second
    # Or, if the vehicle is turning about a point within the frame, sets the speed of
    # the outer-most wheel to the commanded speed (and other speeds are scaled accordingly).
    def setVehicleSpeed(self, spd):
        currentSpeed = spd;
        print ("Setting speed to", spd)
        # Set individual wheel speeds based on multiplier array
        # That array inherently takes care of the direction-reversal for spin-in-place
        maxSpd = 0;
        for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
            self.wheelSpeeds[i] = (spd*self.speedMultipliers[i])
            if abs(self.wheelSpeeds[i]) > maxSpd:
                maxSpd = abs(self.wheelSpeeds[i]);
        #
        
        # If any wheel is above max speed, scale all speeds down
        if maxSpd > self.maxWheelSpeed:
            scaleFactor = self.maxWheelSpeed / maxSpd;
            print("scale factor", scaleFactor)
            for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
                print(self.wheelSpeeds[i])
                self.wheelSpeeds[i] *= scaleFactor;
                print(self.wheelSpeeds[i])
        #
    #
    
    # Does exactly what it says, except that it might not actually publish the message
    # if one has gone out in the last 1/30th of a second. In that case, just stores it,
    # and that message will be sent later if it hasn't been replaced before the next try
    def createAndPublishCommandMessage(self):
        for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
            print("%s, %.2f, %.2f"%(i, self.wheelAngles[i], self.wheelSpeeds[i]))
        
        self.m.header.stamp = rospy.Time.now()
        self.m.front_left_angle  = self.wheelAngles['FrontLeft']
        self.m.front_left_speed  = self.wheelSpeeds['FrontLeft']
        self.m.front_right_angle = self.wheelAngles['FrontRight']
        self.m.front_right_speed = self.wheelSpeeds['FrontRight']
        self.m.back_left_angle   = self.wheelAngles['BackLeft']
        self.m.back_left_speed   = self.wheelSpeeds['BackLeft']
        self.m.back_right_angle  = self.wheelAngles['BackRight']
        self.m.back_right_speed  = self.wheelSpeeds['BackRight']
        self.hasNewMessage = True
        self.publishMessage()
    #
    
    # Rate-limited message publication
    def publishMessage(self):
        if (not self.hasNewMessage) and (rospy.Time.now() - self.lastMessageTime).to_sec() < self.messageInterval:
            return
        self.pub.publish(self.m)
        self.lastMessageTime = rospy.Time.now()
        self.hasNewMessage = False
    #
    

# Intialize everything and go into infinite loop
def start():
    # starts the node
    rospy.init_node('Model2Raw')
    
    # subscribes to joystick inputs on topic "joy", creates publisher
    interpreter = CommandToAngles() 
    
    # Check to see if a message should be sent every 1ms, send if needed
    rate = rospy.Rate(1000) # Hz
    while not rospy.is_shutdown():
        rate.sleep()
        interpreter.publishMessage() 
    #
#

if __name__ == '__main__':
    start()
    
