#!/usr/bin/env python
import rospy
from tycho.msg import WheelAnglesSpeeds, WheelStatus
from std_msgs.msg import Int16

# Create fake WheelStatus messages based on commands sent to the wheels
# Author: Robert Wagner


class MakeFakeWheelStatus:
    def __init__(self):
        rospy.Subscriber("tycho/low_level_motor_values", WheelAnglesSpeeds, self.lowlevel_callback)
        
        self.old_drive_temperatures = [40,40,40,40]
        self.old_steer_temperatures = [40,40,40,40]
        self.old_cntrl_temperatures = [40,40,40,40]
        
        self.timestamp = -1
        self.lastPublication = rospy.Time.now()
        
        self.wheel1Status = WheelStatus()
        self.wheel2Status = WheelStatus()
        self.wheel3Status = WheelStatus()
        self.wheel4Status = WheelStatus()
        
        self.pub = rospy.Publisher('tycho/wheel_status', WheelStatus)
        self.pagepub = rospy.Publisher('tycho/gui_page', Int16)
        
        self.pagepub.publish(Int16(1))
    #
    
    
    
    # Reads high-level commands (e.g. strafing at 1 m/s),
    # sets speed and r/p/y values
    def lowlevel_callback(self, data):
        """
        Reads low-level commands (e.g. front left wheel at x degrees),
        and creates fake "WheelStatus" messages
        """
        
        currentTime = rospy.Time.now()
        
        # Get time delta
        if self.timestamp == -1:
            self.timestamp = currentTime
            return
        #
        
        dt = (currentTime - self.timestamp).to_sec()
        print(dt)
        self.timestamp = currentTime
        
        
        self.wheel1Status.header.stamp    = currentTime
        self.wheel1Status.wheel_id        = 1
        self.wheel1Status.drive_rpm       = data.front_left_speed * 175.0 # Assumes speed is scaled -1 to 1
        self.wheel1Status.steering_angle  = data.front_left_angle
        print ("Front left: ")
        print data.front_left_angle
        self.wheel1Status.drive_amps      = abs(data.front_left_speed) * 25
        self.wheel1Status.steering_amps   = 1
        self.wheel1Status.drive_spin_count = self.wheel1Status.drive_rpm / 60.0 * dt
        self.wheel1Status.drive_temp      = self.old_drive_temperatures[0] + (self.wheel1Status.drive_amps - 30)**2*dt/100
        self.wheel1Status.steering_temp   = 40
        self.wheel1Status.controller_temp = self.old_cntrl_temperatures[0] + (self.wheel1Status.drive_amps - 30)**2*dt/500
        
        
        self.wheel2Status.header.stamp    = currentTime
        self.wheel2Status.wheel_id        = 2
        self.wheel2Status.drive_rpm       = data.front_right_speed * 175.0
        self.wheel2Status.steering_angle  = data.front_right_angle
        print ("Front right: ")
        print data.front_right_angle
        self.wheel2Status.drive_amps      = abs(data.front_right_speed) * 25
        self.wheel2Status.steering_amps   = 1
        self.wheel2Status.drive_spin_count = self.wheel2Status.drive_rpm / 60.0 * dt
        self.wheel2Status.drive_temp      = self.old_drive_temperatures[1] + (self.wheel2Status.drive_amps - 30)**2*dt/100
        self.wheel2Status.steering_temp   = 40
        self.wheel2Status.controller_temp = self.old_cntrl_temperatures[1] + (self.wheel2Status.drive_amps - 30)**2*dt/500
        
       
        self.wheel3Status = WheelStatus()
        self.wheel3Status.header.stamp    = currentTime
        self.wheel3Status.wheel_id        = 3
        self.wheel3Status.drive_rpm       = data.back_left_speed * 175.0
        self.wheel3Status.steering_angle  = data.back_left_angle
        print ("Back left: ")
        print data.back_left_angle
        self.wheel3Status.drive_amps      = abs(data.back_left_speed) * 25
        self.wheel3Status.steering_amps   = 1
        self.wheel3Status.drive_spin_count = self.wheel3Status.drive_rpm / 60.0 * dt
        self.wheel3Status.drive_temp      = self.old_drive_temperatures[2] + (self.wheel3Status.drive_amps - 30)**2*dt/100
        self.wheel3Status.steering_temp   = 40
        self.wheel3Status.controller_temp = self.old_cntrl_temperatures[2] + (self.wheel3Status.drive_amps - 30)**2*dt/500
        
        
        self.wheel4Status.header.stamp    = currentTime
        self.wheel4Status.wheel_id        = 4
        self.wheel4Status.drive_rpm       = data.back_right_speed * 175.0
        self.wheel4Status.steering_angle  = data.back_right_angle
        print ("Back right: ")
        print data.back_right_angle
        self.wheel4Status.drive_amps      = abs(data.back_right_speed) * 25
        self.wheel4Status.steering_amps   = 1
        self.wheel4Status.drive_spin_count = self.wheel4Status.drive_rpm / 60.0 * dt
        self.wheel4Status.drive_temp      = self.old_drive_temperatures[3] + (self.wheel4Status.drive_amps - 30)**2*dt/100
        self.wheel4Status.steering_temp   = 40
        self.wheel4Status.controller_temp = self.old_cntrl_temperatures[3] + (self.wheel4Status.drive_amps - 30)**2*dt/500
        
        
        self.old_drive_temperatures = [self.wheel1Status.drive_temp, self.wheel2Status.drive_temp, self.wheel3Status.drive_temp, self.wheel4Status.drive_temp]
        self.old_steer_temperatures = [self.wheel1Status.steering_temp, self.wheel2Status.steering_temp, self.wheel3Status.steering_temp, self.wheel4Status.steering_temp]
        self.old_cntrl_temperatures = [self.wheel1Status.controller_temp, self.wheel2Status.controller_temp, self.wheel3Status.controller_temp, self.wheel4Status.controller_temp]
        
        
        
        #self.publish()
    #
    
    def publish(self):
        currentTime = rospy.Time.now()
        if (currentTime - self.lastPublication).to_sec() >= 0.2:
            self.lastPublication = currentTime
            self.pub.publish(self.wheel1Status)
            self.pub.publish(self.wheel2Status)
            self.pub.publish(self.wheel3Status)
            self.pub.publish(self.wheel4Status)
    #
    

# Intializes everything
def start():
    # starts the node
    rospy.init_node('Raw2FakeStatus')
    
    interpreter = MakeFakeWheelStatus() # subscribes to low-level commands
    
    rate = rospy.Rate(5) # Hz
    while not rospy.is_shutdown():
        rate.sleep()
        interpreter.publish()
    #
#

if __name__ == '__main__':
    start()
    
