#!/usr/bin/env python
import rospy
import time
from enum import Enum
from sensor_msgs.msg import Joy
from tycho.msg import RoverDriveCommand
from math import atan2, sqrt, pi, atan

# Author: Robert Wagner
# This ROS Node converts Joystick inputs from the joy node
# into commands for the Tycho rover
# Uses the standard ROS "Joy" topic type; control panel Arduino produces that message type
# Button IDs were based on XBox 360 controller buttons used for mini-rover demo

# FIXME: Should load this from a parameter file
# Max speeds in mm/s.  Physical limit is ~3000 mm/s, but can't go quite that fast in a turn.
TYCHO_MAX_SPEED = 2000.0
TYCHO_MAX_FWD_SPEED = 2200.0 # Front-wheel-steering mode speed limit
TYCHO_MAX_STRAFE_SPEED = 1000.0 # sideways-driving speed limit
TYCHO_MAX_SPIN_SPEED = 2200.0 # Spin-in-place wheel speed. 
# This may be a bit high, depends on if you're driving senators or astronauts.
TYCHO_MAX_CIRCLE_STRAFE_SPEED = TYCHO_MAX_STRAFE_SPEED # Spinning about a point in front of you

TYCHO_MAX_ACCEL = 2000.0 # in mm/s^2, primarily used for gentle braking
# Possibly unused? See below.
# There's also an acceleration parameter and PID loop settings in the motor controller 
# that play into smooth starting/stopping.

TYCHO_MINIMUM_TURN_RADIUS = 1 # meters, anything outside TYCHO_STEERING_WIDTH/2 (~0.56) is valid
# TODO: The MoonRacer-emulator function will have a different value for this.

TYCHO_DEFAULT_CIRCLE_STRAFE_DISTANCE = 2.5 # Meters
TYCHO_MINIMUM_CIRCLE_STRAFE_DISTANCE = 1.5 # Anything above TYCHO_WHEELBASE/2 (~1.32) is valid
TYCHO_CSTRAFE_ADJUST_RATE = 0.75 # Meters/second, speed of moving the turning centerpoint

TYCHO_WHEELBASE = 2.62

TYCHO_WHEEL_X_DISTANCE = TYCHO_WHEELBASE/2

# For tracking whether one end should be disabled.  Currently hard-coded in this script,
# should at least be a parameter, and preferably determined on the fly based on reported
# motor controller flags, so if one end is unpowered the steering compensates automatically
class EndStatus(Enum):
    BOTH = "BOTH"
    FRONT_ONLY = "FRONT_ONLY"
    BACK_ONLY = "BACK_ONLY"
#
TYCHO_DRIVE_END_STATUS = EndStatus.BOTH


# In normal mode, joystick must move this much from [current position] to register as a direction change
# This likely is not needed now that adjusting the steering motors a little bit doesn't come 
# with a chance of catastrophic system failure.
# TODO: Test setting this to 0 or maybe 0.05, see if driving improves.
NORMAL_JOYX_DEADZONE = 0.20 

# If joystick is within this much of the center, treat it as 0 (avoid accidental acceleration)
# TODO: It's possible there's another deadzone in the Arduino code for the joystick, I'm not sure.
JOY_DEADZONE = 0.05

class DriveMode(Enum):
    STOP = 0
    NORMAL = 1
    STRAFE = 2
    INPLACE = 3
    FRONT_STEER = 4 # TODO: Replace this with "MoonRacer Emulator Mode"
    CIRCLESTRAFE = 5
#

class JoyToCommand:
    
    def __init__(self):
        rospy.Subscriber("tycho/joy", Joy, self.callback, queue_size=1)
        self.pub = rospy.Publisher('tycho/joystick_commands', RoverDriveCommand, queue_size=1)
        
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
        # Used to allow stop button to maintain state of steering with minimal
        # interaction with other drive modes
        self.target_speed = 0
        self.actual_speed = 0
        self.turnX = 0
        self.turnY = 0
        self.strafeAngle = 0
        self.isStrafing  = True
        self.isBraking   = True
        self.lastSpeedUpdateTime = rospy.Time.now()
        
        # Mapping of Joy topic button IDs to modes
        self.buttonMap = {}
        self.buttonMap['joyXAxis'] = 0 # X is left/right, Y is forward-backward
        self.buttonMap['joyYAxis'] = 1
        self.buttonMap['buttonNormal']       = 5
        self.buttonMap['buttonStrafe']       = 1
        self.buttonMap['buttonCircleStrafe'] = 4
        self.buttonMap['buttonTurnInPlace']  = 3
        self.buttonMap['buttonStop']         = 2
        self.buttonMap['buttonFrontSteer']   = 0
        
        # Internal tracker of current joystick/button state
        self.joyState = {}
        self.joyState['joyXAxis'] = 0.0
        self.joyState['joyYAxis'] = 0.0
        self.joyState['buttonNormal']       = False
        self.joyState['buttonStrafe']       = False
        self.joyState['buttonCircleStrafe'] = False
        self.joyState['buttonTurnInPlace']  = False
        self.joyState['buttonStop']         = False
        self.joyState['buttonFrontSteer']   = False
    #
    
    # Use the time since the last update to accelerate towards the target speed
    def update_speed(self):
        # Short-circuiting for now, might cut the function entirely
        self.actual_speed = self.target_speed
        return
        
        # Always update the timestamp
        new_time = rospy.Time.now()
        delta_time = (new_time - self.lastSpeedUpdateTime).to_sec()
        self.lastSpeedUpdateTime = new_time
        
        # Get amount left to accelerate
        delta_speed = self.target_speed - self.actual_speed
        if delta_speed == 0.0: return
        
        # If a change is needed, apply it
        # TODO: Move these to a parameter file
        max_accel = TYCHO_MAX_ACCEL
       # if self.target_speed == 0: # Reduce accel limit when approaching 0
       #     if abs(self.actual_speed) < 30: # Remember: speeds are in mm/s, not m/s!
       #         max_accel = TYCHO_MAX_ACCEL/40.0
       #     elif abs(self.actual_speed) < 300:
       #         max_accel = TYCHO_MAX_ACCEL/4.0
       #     elif abs(self.actual_speed) < 1000:
       #         max_accel = TYCHO_MAX_ACCEL
        #
        max_delta = max_accel * delta_time
        
        #print(self.actual_speed, self.target_speed, delta_time, max_delta, delta_speed)
        
        #if abs(delta_speed) < max_delta: # Don't need the max accel to reach target
        #    self.actual_speed = self.target_speed
        #else:
        if delta_speed < 0:
            self.actual_speed -= max_delta
        else:
            self.actual_speed += max_delta
        #
        return
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
                if abs(newdata) < JOY_DEADZONE:
                    newdata = 0
                #
                # TODO: Apply an exponential remapping here?
                # Nah, probably better to do so in the various Interpret functions
                # https://www.chiefdelphi.com/t/paper-joystick-sensitivity-gain-adjustment/107280/3
                # TODO: What? Why am I doing this here when I just said I wouldn't?  Maybe move to normal-drive mode?
                # FIXME: Disable this code, see how the vehicle response changes
                # This code definitely should *not* be here, it was not implemented
                # correctly or in the right place, but there may be other stuff tuned
                # around its erroneous existence.
                if key == "joyXAxis": # FIXME: The original comment here implied that this was supposed to be applied to the speed axis.
                  joystick_exp_gain = 0.6   # Range [0-1]
                  newdata = joystick_exp_gain*(newdata**3) + (1-joystick_exp_gain)*newdata
                # See also https://www.chiefdelphi.com/t/paper-joystick-sensitivity-gain-adjustment/107280/12
                # for possible use in the normal steering mode to tweak minimum turning arc
            else:
                newdata = (data.buttons[ self.buttonMap[key] ] == 1)
            #
            
            if self.joyState[key] != newdata:
                self.joyState[key] = newdata
                return True
            return False
        #
        
        # TODO: Why is it never registering the return to 0,0 properly? (Not sure if this is still an issue)
        # TODO: Why is stopButton disabling the turning mechanism as well? isBraking.
        # TODO: Why does stopButton stop working if I don't specify isBraking? (Not sure if this is still an issue)
        hasChanged = False
        hasChanged = updateValueIfNeeded('joyXAxis', isAxis=True) or hasChanged
        hasChanged = updateValueIfNeeded('joyYAxis', isAxis=True) or hasChanged
        hasChanged = updateValueIfNeeded('buttonStrafe') or hasChanged
        hasChanged = updateValueIfNeeded('buttonNormal') or hasChanged
        hasChanged = updateValueIfNeeded('buttonTurnInPlace') or hasChanged
        hasChanged = updateValueIfNeeded('buttonCircleStrafe') or hasChanged
        hasChanged = updateValueIfNeeded('buttonFrontSteer') or hasChanged
        
        # Ignore stop button release unless joystick is neutral
        # TODO: Test how this works on real rover.  Outside.  With nobody nearby.
        if not self.joyState['buttonStop'] or (self.joyState['joyXAxis'] == 0 and self.joyState['joyYAxis'] == 0):
            hasChanged = updateValueIfNeeded('buttonStop') or hasChanged
        #
        
        # If things have changed, issue a new command
        # I think the circle-strafe exception is so that it will keep changing the wheel
        # angles even if the joystick stays steady if it's not at X=0.
        if hasChanged or self.steeringMode == DriveMode.CIRCLESTRAFE:
            self.updateDriveMode() # First set the right mode
            self.interpretJoystick() # Then interpret
    #
    
    def updateDriveMode(self):
        """
        Uses the current button state to set the self.steeringMode variable
        If multiple buttons are pressed, prioritizes them
        If no buttons are pressed, defaults to DriveMode.NORMAL
        If multiple buttons are pressed, stop takes priority, otherwise no thought was
        given to priority. Control panel does not allow multiple buttons pressed.
        """
        changedMode = False
        def changeModeIfNeeded(mode):
            if self.steeringMode != mode:
                self.steeringMode = mode
                return True
            return False
        #
        if self.joyState['buttonStop']:
            changedMode = changeModeIfNeeded(DriveMode.STOP)
        elif self.joyState['buttonCircleStrafe']:
            changedMode = changeModeIfNeeded(DriveMode.CIRCLESTRAFE)
            if changedMode:
                self.circleStrafeDistance = TYCHO_DEFAULT_CIRCLE_STRAFE_DISTANCE
        elif self.joyState['buttonFrontSteer']:
            changedMode = changeModeIfNeeded(DriveMode.FRONT_STEER)
        elif self.joyState['buttonTurnInPlace']:
            changedMode = changeModeIfNeeded(DriveMode.INPLACE)
        elif self.joyState['buttonStrafe']:
            changedMode = changeModeIfNeeded(DriveMode.STRAFE)
        elif self.joyState['buttonNormal']:
            changedMode = changeModeIfNeeded(DriveMode.NORMAL)
        else:
            changedMode = changeModeIfNeeded(DriveMode.NORMAL)
        #
        if changedMode:
            print("Set driving mode to %s"%self.steeringMode.name)
        #
    #
    
    # Highly experimental "only-one-stick-no-buttons" mode selector.
    # Unused.  Never really tested.
    # Does not allow driving in reverse, all reverse movements are to let you switch modes
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
    
    # Entry-point to joystick interpretation functions.
    # Calls the correct variant based ont eh current driving mode.
    # All interpreters end with publishing the command message
    # TODO: Maybe move that publish step into here, or even one level up in callback()
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
            
        elif self.steeringMode == DriveMode.FRONT_STEER:
            self.interpretNormal(True)
            
        else:
            self.interpretStop()
        #
    #
    
    # Takes an input that's +/- fraction of max speed, turns it into real values
    def scaleAndLimitSpeed(self, raw_speed, max_speed):
        if raw_speed >  1: raw_speed =  1.0
        if raw_speed < -1: raw_speed = -1.0
        return raw_speed * max_speed
    #
    
    # INTERPRETER
    # Stop movement, but don't touch the wheel angles
    def interpretStop(self):
        self.isBraking   = True
        self.actual_speed = 0 # The "stop" mode should allow slamming on the brakes
        self.target_speed = 0
        self.publishCommandMessage()
    #
    
    # INTERPRETER
    # Drive like a normal car, with 2- or 4-wheel steering
    def interpretNormal(self, force_front_steering=False):
        print('Using interpretNormal()')
        self.strafeAngle = 0
        
        # Force front-only steering if commanded and possible, otherwise fall back on defaults
        if force_front_steering and TYCHO_DRIVE_END_STATUS is not EndStatus.BACK_ONLY:
            self.turnX = -TYCHO_WHEEL_X_DISTANCE # Rear axle
            speedLimit = TYCHO_MAX_FWD_SPEED
        else:
            if TYCHO_DRIVE_END_STATUS is EndStatus.FRONT_ONLY:
                self.turnX = -TYCHO_WHEEL_X_DISTANCE # Rear axle
            elif TYCHO_DRIVE_END_STATUS is EndStatus.BACK_ONLY:
                self.turnX = TYCHO_WHEEL_X_DISTANCE # Front axle
            else: # TYCHO_DRIVE_END_STATUS is EndStatus.BOTH:
                self.turnX = 0 # Center of rover
            #
            speedLimit = TYCHO_MAX_SPEED
        #
        self.isBraking = False
        
        # Set speed
        self.target_speed = self.scaleAndLimitSpeed(self.joyState['joyYAxis'], speedLimit)
        print ("Limited %.2f to %.2f: %.2f"%(self.joyState['joyYAxis'], speedLimit, self.target_speed) )
        
        # Set steering values depending on joystick left/right axis
        if (self.joyState['joyXAxis'] != 0.0): # Turning: set radius
            # TODO: Maybe a different function? (sqrt?)
            self.turnY = -1/self.joyState['joyXAxis'] * TYCHO_MINIMUM_TURN_RADIUS
            self.isStrafing = False
        else: # Not turning: use "strafing" mode to drive straight
            self.turnY = 0
            self.isStrafing = True
        #
        
        # Wacky fun time!
        # These lines rotate the steering axis so that strafeAngle is forward
        # Not actually useful.  Like, VERY anti-useful.  But a great demonstration of the 
        # vehicle's more extreme capabilities.
        # Don't try this indoors with the full vehicle.
        # (But yes, I absolutely *am* implying that you should try it *outdoors* with the 
        # full vehicle after verifying that it works with the mini-rover.)
        # (Ideally without mentioning that this is possible until after you've started driving like this.)
        #strafeAngle = 30
        #self.turnX = self.turnY*sin(strafeAngle*pi/180.0)
        #self.turnY = self.turnY*cos(strafeAngle*pi/180.0)
        
        self.publishCommandMessage()
    #
    
    # INTERPRETER
    # Drive straight in any direction
    def interpretStrafe(self):
        print('Using interpretStrafe()')
        
        if TYCHO_DRIVE_END_STATUS is not EndStatus.BOTH:
            print ("ERROR: Cannot strafe without all wheels")
            self.interpretStop()
            # TODO: Fall back on something else, or just stop?
            return
        #
        
        # Never allow reversing in strafe mode- it causes messiness
        # Instead, treat all negative forward-backward joystick values as 0.
        # The issue is when going from sideways-and-a-hair-forward to sideways-and-a-hair-
        # backward, and that that's intractable because the wheels would need to turn 180 
        # instantly.  Could work around it by locking out the "other" half of the forward-
        # backward joystick axis as soon as the vehicle starts to move, until the joystick
        # gets zeroed out again.
        if self.joyState['joyYAxis'] < 0:
            joyYAxis = self.joyState['joyYAxis']
        else:
            joyYAxis = 0
        #
        
        # Set speed based of vector length of joystick tilt, limited to 1.0 (clips corners)
        speed = sqrt(self.joyState['joyXAxis']**2 + joyYAxis**2)
        speed = self.scaleAndLimitSpeed(speed, 1.0)
        isBraking = False
        print ("Base speed: %.2f"%(speed) )
        
        # If stick is tilting only forward or in two directions, match the wheels to that direction
        # If stick is not tilting forward at all (possibly is backwards), set wheel angle 
        # between 0 and 90 depending on amount tilted (so that wheels don't instantly whip to the side)
        if joyYAxis != 0:
            strafeAngle = atan(-self.joyState['joyXAxis'] / joyYAxis)*180/pi
        else:
            strafeAngle = 90.0 * min(max(-self.strafeDeadZone, self.joyState['joyXAxis']), self.strafeDeadZone) / self.strafeDeadZone
        print ("Base angle: %.2f"%(strafeAngle) )
        
        # Adjust for larger strafe-mode deadzone
        # Near deadzone, slowly shift towards sideways
        if abs(speed) < self.strafeDeadZone:
        # TODO: deadzone should actually be a pair of wedges going to the sides
        # the speed shouldn't cut off at the edge of the deadzone
        #if speed < self.strafeDeadZone and abs(self.joyState['joyXAxis']) > abs(self.joyState['joyYAxis']):
            strafeAngle = (speed/self.strafeDeadZone)*strafeAngle
            speed = 0
        else:
            speed = (speed-self.strafeDeadZone) / (1-self.strafeDeadZone)
        #
        speed = -self.scaleAndLimitSpeed(speed, TYCHO_MAX_STRAFE_SPEED)
        print ("Adjusted speed: %.2f, Angle: %.2f"%(speed, strafeAngle) )
        
        # TODO: What is this fixing?
        if abs(self.joyState['joyXAxis']) < 0.05 or abs(strafeAngle > 90):
            strafeAngle = 0.0
        #
        
        # This funciton uses a different approach to setting up the message; instead of 
        # setting all the parts (self.whatever) in this function, it uses local variables
        # and then dumps them into this helper function:
        self.updateFullMessage(speed=speed, turnX=0, turnY=0, strafeAngle=strafeAngle, isStrafing=True, isBraking=isBraking);
        self.publishCommandMessage()
    #
    
    # INTERPRETER
    # Spin about the center of the rover
    # TODO: Might want to change this to the location of a camera onboard the rover for perfect panoramas
    def interpretTurnInPlace(self):
        print('Using interpretTurnInPlace()')
        speed = self.joyState['joyXAxis']
        speed = self.scaleAndLimitSpeed(self.joyState['joyXAxis'], TYCHO_MAX_SPIN_SPEED)
        
        # If one end is disable, spin about that end instead of the center.  Works just fine.
        if TYCHO_DRIVE_END_STATUS is EndStatus.FRONT_ONLY:
            self.turnX = -TYCHO_WHEEL_X_DISTANCE # Rear axle
        elif TYCHO_DRIVE_END_STATUS is EndStatus.BACK_ONLY:
            self.turnX = TYCHO_WHEEL_X_DISTANCE # Front axle
        else:
            self.turnX = 0 # Center of rover
        #
        
        self.updateFullMessage(speed, turnX=self.turnX, turnY=0, strafeAngle=0, isStrafing=False, isBraking=False);
        self.publishCommandMessage()
    #
    
    # INTERPRETER
    # Spin about a point in front of the rover
    def interpretCircleStrafe(self):
        print ('Using interpretCircleStrafe()')
        
        if TYCHO_DRIVE_END_STATUS is not EndStatus.BOTH:
            print ("ERROR: Cannot strafe without all wheels")
            self.interpretStop()
            # TODO: Fall back on something else, or just stop?
            return
        #
        
        # Set speed based on left-right tilt of joystick
        speed = -self.joyState['joyXAxis']
        speed = self.scaleAndLimitSpeed(-self.joyState['joyXAxis'], TYCHO_MAX_CIRCLE_STRAFE_SPEED)
        
        # Adjust circle-strafing distance using forward-backward joystick position
        # (with a big deadzone! You need to MEAN a rotation point change!)
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
        self.target_speed = speed
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
        self.update_speed()
        m = RoverDriveCommand()
        m.header.stamp = rospy.Time.now()
        m.speed          = self.actual_speed
        m.turn_center_x  = self.turnX
        m.turn_center_y  = self.turnY
        m.strafing_angle = self.strafeAngle
        m.is_strafing    = self.isStrafing
        m.is_braking     = self.isBraking
        print("%.2f, (%.1f,%.1f), %.0f %s %s"%(self.actual_speed, self.turnX, self.turnY, self.strafeAngle, self.isStrafing, self.isBraking))
        
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


# Intialize everything
def start():
    rospy.init_node('Joy2Tycho')
    interpreter = JoyToCommand() # subscribes to joystick inputs on topic "joy"
    
    # starts the node
    #rospy.spin()
    # Send a new command every 10ms even if no change, to allow smooth acceleration
    rate = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():
        rate.sleep()
        interpreter.publishCommandMessage()
    #
#

if __name__ == '__main__':
    start()
#
