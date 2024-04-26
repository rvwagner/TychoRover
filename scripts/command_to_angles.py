#!/usr/bin/env python
import rospy
from tycho.msg import RoverDriveCommand, WheelAnglesSpeeds
from math import atan, atan2, sqrt, pi, sin, cos

# Author: Robert Wagner
TYCHO_MAX_WHEEL_SPEED = 4200; # mm/s # FIXME: This should be pulled from a config file
# TYCHO_MAX_WHEEL_SPEED = rospy.get_param("/tycho/speeds/max_wheel")

class CommandToAngles:
    def __init__(self):
        rospy.Subscriber("tycho/final_commands", RoverDriveCommand, self.callback, queue_size=1)
        self.isStrafing = True
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
        
        self.maxWheelSpeed = TYCHO_MAX_WHEEL_SPEED
        self.steerRate = 36.0 # Approximate speed of steering motors in deg/sec
        # TODO: Should use the time-to-destination to tweak speeds?
        # +X is forward, +Y is left, numbers in scale meters
        # TODO: Load these from ROS parameter server
        # self.jointX = rospy.get_param("/tycho/joint_x_positions")
        # self.jointX = rospy.get_param("/tycho/joint_y_positions")
        # self.jointX = rospy.get_param("/tycho/steering_arm_lengths")
        width = 1.27 # Old code values
        length = 2.34
        width = 1.115 # Exact physical measure
        length = 2.62 # rough physical measure
        self.jointX = {'FrontLeft': length/2, 'FrontRight': length/2,
                       'BackLeft': -length/2, 'BackRight': -length/2}
        self.jointY = {'FrontLeft': width/2, 'FrontRight': -width/2,
                       'BackLeft':  width/2, 'BackRight':  -width/2}
        self.steeringArmLength = { # Pivot to center of tire
                                    'FrontLeft':  0.31115,
                                    'FrontRight': 0.31115,
                                    'BackLeft':   0.31115,
                                    'BackRight':  0.31115}
        
        # System to rate-limit messages
        self.hasNewMessage = False
        self.lastMessageTime = rospy.Time(0)
        self.messageInterval = 1.0/30 # second mnumber is Hz
        
        self.m = WheelAnglesSpeeds()
        
        self.pub = rospy.Publisher('tycho/low_level_motor_values', WheelAnglesSpeeds, queue_size=1)
    #
    
    
    
    # Receives joystick messages (subscribed to Joy topic)
    # then converts the joysick inputs into Twist commands
    # axis 1 aka left stick vertical controls linear speed
    # axis 0 aka left stick horizonal controls angular speed
    def callback(self, data):
        """
        
        """
        #print("callback speed=%f"%data.speed)
        
        if data.is_braking:
            self.speed = 0
            self.setVehicleSpeed(self.speed) #!!!
            self.createAndPublishCommandMessage() #!!!
            return
        #
        
        if data.is_strafing:
            dataIsNew = (data.speed != self.speed or data.strafing_angle != self.strafingAngle) or not self.isStrafing
        else:
            dataIsNew = (data.speed != self.speed or data.turn_center_x != self.turnX or data.turn_center_y != self.turnY) or self.isStrafing
        #
        
        self.speed = data.speed
        
        if dataIsNew and data.is_strafing:
            print("Strafing, %.2f"%(data.strafing_angle) )
            self.isStrafing = True
            self.strafingAngle = data.strafing_angle
            self.setAnglesStrafing(data.strafing_angle)
            self.setVehicleSpeed(self.speed)
            self.createAndPublishCommandMessage()
        elif dataIsNew and not data.is_strafing:
            print("Not strafing, %.2f %.2f"%(data.turn_center_x,data.turn_center_y) )
            self.isStrafing = False
            self.turnX = data.turn_center_x
            self.turnY = data.turn_center_y
            self.setAnglesFromTurnCenter(data.turn_center_x, data.turn_center_y)
            self.setVehicleSpeed(self.speed)
            self.createAndPublishCommandMessage()
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
        #return self.setToeInAngles(angle);
        # TODO: Maybe try to get the fancy version working, as it should be better for weird angles
        toein = 1.5
        if self.speed > 0: # Reverse
            toein = -toein
        #
        
        return self.setSteeringAngles(angle+toein, angle-toein, angle+toein, angle-toein);
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
            circlingPoint = False #!!!
            
            if abs(turnCenterX) > self.jointX['FrontLeft'] and abs(turnCenterY) < self.jointY['FrontLeft']:
                circlingPoint = True #!!!
            
            # Calculate the distance to the turning point based on whether
            # steering arm points towards or away from the turn point
            distTotal = sqrt(distX*distX + distY*distY);
            if turnCenterY/self.jointY[i] > 1:
                # Same sign and abs(turnCenterY) > abs(self.jointY)
                # -> Arm points towards point
                distTotal -= self.steeringArmLength[i]
            else: # Arm points away from point
                distTotal += self.steeringArmLength[i]
            #

            # For very small offsets from turn-in-place, make all wheel speeds match here to avoid rounding issues
            if turnPointDistance <= 0.01:
                self.speedMultipliers[i] = 1.0;
            else:
                self.speedMultipliers[i] = distTotal/turnPointDistance;
            if self.speedMultipliers[i] > maxMultiplier:
                maxMultiplier = self.speedMultipliers[i];
        #
        
        #angles['FrontLeft'] += 10
        #angles['FrontRight'] += 0
        #angles['BackLeft'] += -35
        #angles['BackRight'] += 30
        #
        # If turn point is interior to wheels, adjust multipliers to:
        # A) Max out at 1.0
        # B) Reverse the left-hand wheels
        # TODO: Add variables for joint X and Y positions in general, rather than relying on joint[1] being positive
        
        if (abs(turnCenterX) < self.jointX['FrontLeft'] and abs(turnCenterY) < self.jointY['FrontLeft']) or circlingPoint == True:

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
    
    
    # Set the steering angle to put all wheels sub-parallel at the specified angle
    # Angles wheels to point towards a point 50m away in the desired direction
    # TODO: Use speed +/- to switch toe-in direction (center x/y sign)
    # Valid angles go from -90 to +90, with 0 being straight ahead and +90 being to the left
    # (+X is forward, +Y is left, units in cm)
    # Returns estimated time until the wheels reach the target angles in ms.
    def setToeInAngles(self, angle):
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
    # Valid speed values range from -1 to +1
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
        #self.pub.publish(self.m)
    #
    
    # Rate-limited message publication
    def publishMessage(self):
        if (not self.hasNewMessage) and (rospy.Time.now() - self.lastMessageTime).to_sec() < self.messageInterval:
            return
        self.pub.publish(self.m)
        self.lastMessageTime = rospy.Time.now()
        self.hasNewMessage = False
    #
    

# Intializes everything
def start():
    # starts the node
    rospy.init_node('Model2Raw')
    
    # subscribes to joystick inputs on topic "joy", creates publisher
    interpreter = CommandToAngles() 
    
    # Try to flush the message buffer every 1ms
    rate = rospy.Rate(1000) # Hz
    while not rospy.is_shutdown():
        rate.sleep()
        interpreter.publishMessage()
    #
#

if __name__ == '__main__':
    start()
    
