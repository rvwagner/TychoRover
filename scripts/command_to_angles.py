#!/usr/bin/env python
import rospy
from tycho.msg import RoverDriveCommand, WheelAnglesSpeeds
from math import atan, atan2, sqrt, pi

# Author: Robert Wagner


class CommandToAngles:
    def __init__(self):
        rospy.Subscriber("tycho/final_commands", RoverDriveCommand, self.callback)
        self.strafingAngle = 0.0
        self.turnX = 0.0
        self.turnY = 0.0
        self.speed = 0.0
        
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
        
        #####################
        # Dimensional Model #
        #####################
        
        self.maxWheelSpeed = 1; # m/s
        self.steerRate = 500.0; # Servo nominally rotates at 500deg/sec
        # +X is forward, +Y is left, numbers in scale meters
        self.jointX = {'FrontLeft': 1.17, 'FrontRight': 1.17,
                       'BackLeft': -1.17, 'BackRight': -1.17};
        self.jointY = {'FrontLeft': 0.635, 'FrontRight': -0.635,
                       'BackLeft':  0.635, 'BackRight':  -0.635};
        self.steeringArmLength = 0.365; # Pivot to center of tire
        
        self.pub = rospy.Publisher('tycho/low_level_motor_values', WheelAnglesSpeeds)
    #
    
    
    
    # Receives joystick messages (subscribed to Joy topic)
    # then converts the joysick inputs into Twist commands
    # axis 1 aka left stick vertical controls linear speed
    # axis 0 aka left stick horizonal controls angular speed
    def callback(self, data):
        """
        
        """
        print("callback speed=%f"%data.speed)
        
        if data.is_braking:
            self.speed = 0
        else:
            self.speed = data.speed
            #self.setVehicleSpeed(0)
            #self.createCommandMessage()
            #return
        #
        
        if data.is_strafing and (data.speed != self.speed or data.strafing_angle != self.strafingAngle):
            self.strafingAngle = data.strafing_angle
            #self.speed = data.speed
            self.setAnglesStrafing(data.strafing_angle)
            self.setVehicleSpeed(self.speed)
            self.createCommandMessage()
        #
        
        if not data.is_strafing and (data.speed != self.speed or data.turn_center_x != self.turnX or data.turn_center_y != self.turnY):
            print("Not strafing, %.2f %.2f"%(data.turn_center_x,data.turn_center_y) )
            self.turnX = data.turn_center_x
            self.turnY = data.turn_center_y
            #self.speed = data.speed
            self.setAnglesFromTurnCenter(data.turn_center_x, data.turn_center_y)
            self.setVehicleSpeed(self.speed)
            self.createCommandMessage()
        #
        
    #
    
    # Commands the servos to go to specified angles
    # Returns the estimated time until they reach that angle in seconds
    def setSteeringAngles(self, fl, fr, bl, br):
        # Get maximum change in angle for calculating update duration
        maxAngleChange = abs(fl - self.wheelAngles['FrontLeft'])
        change = abs(fr - self.wheelAngles['FrontRight'])
        if change > maxAngleChange: maxAngleChange = change
        change = abs(bl - self.wheelAngles['BackLeft'])
        if change > maxAngleChange: maxAngleChange = change
        change = abs(br - self.wheelAngles['BackRight'])
        if change > maxAngleChange: maxAngleChange = change
  
        self.wheelAngles['FrontLeft']  = fl
        self.wheelAngles['FrontRight'] = fr
        self.wheelAngles['BackLeft']   = bl
        self.wheelAngles['BackRight']  = br

        return maxAngleChange/self.steerRate
    #

    # Set the steering angle to put all wheels parallel at the specified angle
    # Valid angles go from -90 to +90, with 0 being straight ahead and +90 being to the left
    # Returns estimated time until the wheels reach the target angles in ms.
    def setAnglesStrafing(self, angle):
        # No speed multiplication needed in this drive mode
        self.speedMultipliers['FrontLeft']  = 1.0
        self.speedMultipliers['FrontRight'] = 1.0
        self.speedMultipliers['BackLeft']   = 1.0
        self.speedMultipliers['BackRight']  = 1.0
  
        #setVehicleSpeed(currentSpeed);
        
        return self.setSteeringAngles(angle, angle, angle, angle);
    #
    
    #// Commands the steering servos to go to appropriate angles for the vehicle to turn
    #// about the specified position in x/y space (+X is forward, +Y is left, units in cm)
    #// Also adjusts the current motor speeds to be correct for the differing ditances from
    #// the turning center. 
    #// Returns estimated time until the wheels reach the target angles in ms.
    def setAnglesFromTurnCenter(self, turnCenterX, turnCenterY):
        angles = {};
        turnPointDistance = sqrt(turnCenterX*turnCenterX + turnCenterY*turnCenterY);
        maxMultiplier = 0.0;
        
        for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
            distY = turnCenterY-self.jointY[i];
            distX = turnCenterX-self.jointX[i];
            angles[i] = atan(distX/distY)*180/pi+90;
            
            # Calculate the distance to the turning point based on whether
            # steering arm points towards or away from the turn point
            distTotal = sqrt(distX*distX + distY*distY);
            if turnCenterY/self.jointY[i] > 1:
                # Same sign and abs(turnCenterY) > abs(self.jointY)
                # -> Arm points towards point
                distTotal -= self.steeringArmLength;
            else: # Arm points away from point
                distTotal += self.steeringArmLength;
            #

            # For very small offsets from turn-in-place, make all wheel speeds match here to avoid rounding issues
            if turnPointDistance <= 0.01:
                self.speedMultipliers[i] = 1.0;
            else:
                self.speedMultipliers[i] = distTotal/turnPointDistance;
            if self.speedMultipliers[i] > maxMultiplier:
                maxMultiplier = self.speedMultipliers[i];
        #
      
        # If turn point is interior to wheels, adjust multipliers to:
        # A) Max out at 1.0
        # B) Reverse the left-hand wheels
        # TODO: Add variables for joint X and Y positions in general, rather than relying on joint[1] being positive
        
        if abs(turnCenterX) < self.jointX['FrontLeft'] and abs(turnCenterY) < self.jointY['FrontLeft']:
            
            self.speedMultipliers['FrontLeft']  =  -self.speedMultipliers['FrontLeft']  / maxMultiplier;
            self.speedMultipliers['FrontRight'] =  self.speedMultipliers['FrontRight'] / maxMultiplier;
            self.speedMultipliers['BackLeft']   =  -self.speedMultipliers['BackLeft']   / maxMultiplier;
            self.speedMultipliers['BackRight']  =  self.speedMultipliers['BackRight']  / maxMultiplier;
            
        #
        #// Adjust the current drive speed based on the new speed multipliers
        #setVehicleSpeed(currentSpeed);
        
        # Actually command the servos
        return self.setSteeringAngles(angles['FrontLeft']-90, angles['FrontRight']-90,
                                      angles['BackLeft']-90, angles['BackRight']-90);
    #

    
    # Set the motor speeds such that the center of the vehicle travels at the commanded speed.
    # Valid speed values range from -1 to +1
    # Or, if the vehicle is turning about a point within the frame, sets the speed of
    # the outer-most wheel to the commanded speed (and other speeds are scaled accordingly).
    def setVehicleSpeed(self, spd):
        currentSpeed = spd;
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
            for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
                self.wheelSpeeds[i] *= scaleFactor;
        #
    #
    
    
    def createCommandMessage(self):
        for i in ['FrontLeft','FrontRight','BackLeft','BackRight']:
            print("%s, %.2f, %.2f"%(i, self.wheelAngles[i], self.wheelSpeeds[i]))
        
        m = WheelAnglesSpeeds()
        m.header.stamp = rospy.Time.now()
        m.front_left_angle  = self.wheelAngles['FrontLeft']
        m.front_left_speed  = self.wheelSpeeds['FrontLeft']
        m.front_right_angle = self.wheelAngles['FrontRight']
        m.front_right_speed = self.wheelSpeeds['FrontRight']
        m.back_left_angle   = self.wheelAngles['BackLeft']
        m.back_left_speed   = self.wheelSpeeds['BackLeft']
        m.back_right_angle  = self.wheelAngles['BackRight']
        m.back_right_speed  = self.wheelSpeeds['BackRight']
        self.pub.publish(m)
    #
    

# Intializes everything
def start():
    interpreter = CommandToAngles() # subscribes to joystick inputs on topic "joy"
    
    # starts the node
    rospy.init_node('Model2Raw')
    rospy.spin()
#

if __name__ == '__main__':
    start()
    
