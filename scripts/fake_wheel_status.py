#!/usr/bin/env python
import rospy
from tycho.msg import WheelAnglesSpeeds, WheelStatus

# Create fake WheelStatus messages based on commands sent to the wheels
# Author: Robert Wagner


class MakeFakeWheelStatus:
    def __init__(self):
        rospy.Subscriber("tycho/low_level_motor_values", WheelAnglesSpeeds, self.lowlevel_callback)
        
        self.old_drive_temperatures = [40,40,40,40]
        self.old_steer_temperatures = [40,40,40,40]
        self.old_cntrl_temperatures = [40,40,40,40]
        
        self.timestamp = -1
        
        self.pub = rospy.Publisher('tycho/wheel_status', WheelStatus)
    #
    
    
    
    # Reads high-level commands (e.g. strafing at 1 m/s),
    # sets speed and r/p/y values
    def lowlevel_callback(self, data):
        """
        Reads low-level commands (e.g. front left wheel at x degrees),
        and creates fake "WheelStatus" messages
        """
        
        # Get time delta
        # Get time delta
        if self.timestamp == -1:
            self.timestamp = data.header.stamp
            return
        else:
            dt = (data.header.stamp - self.timestamp).secs # TODO: Is this in seconds? It should be
            self.timestamp = data.header.stamp
        #
        print(dt)
        wheel1Status = WheelStatus()
        wheel1Status.wheel_id        = 1
        wheel1Status.drive_rpm       = data.front_left_speed * 175.0 # Assumes speed is scaled -1 to 1
        wheel1Status.steering_angle  = data.front_left_angle
        wheel1Status.drive_amps      = data.front_left_speed * 150
        wheel1Status.steering_amps   = 1
        wheel1Status.drive_spin_count = wheel1Status.drive_rpm / 60.0 * dt
        wheel1Status.drive_temp      = self.old_drive_temperatures[0] + (wheel1Status.drive_amps - 30)**2*dt/100
        wheel1Status.steering_temp   = 40
        wheel1Status.controller_temp = self.old_cntrl_temperatures[0] + (wheel1Status.drive_amps - 30)**2*dt/500
        
        
        wheel2Status = WheelStatus()
        wheel2Status.wheel_id        = 2
        wheel2Status.drive_rpm       = data.front_right_speed * 175.0
        wheel2Status.steering_angle  = data.front_right_angle
        wheel2Status.drive_amps      = data.front_right_speed * 150
        wheel2Status.steering_amps   = 1
        wheel2Status.drive_spin_count = wheel2Status.drive_rpm / 60.0 * dt
        wheel2Status.drive_temp      = self.old_drive_temperatures[1] + (wheel2Status.drive_amps - 30)**2*dt/100
        wheel2Status.steering_temp   = 40
        wheel2Status.controller_temp = self.old_cntrl_temperatures[1] + (wheel2Status.drive_amps - 30)**2*dt/500
        
        
        wheel3Status = WheelStatus()
        wheel3Status.wheel_id        = 3
        wheel3Status.drive_rpm       = data.back_left_speed * 175.0
        wheel3Status.steering_angle  = data.back_left_angle
        wheel3Status.drive_amps      = data.back_left_speed * 150
        wheel3Status.steering_amps   = 1
        wheel3Status.drive_spin_count = wheel3Status.drive_rpm / 60.0 * dt
        wheel3Status.drive_temp      = self.old_drive_temperatures[2] + (wheel3Status.drive_amps - 30)**2*dt/100
        wheel3Status.steering_temp   = 40
        wheel3Status.controller_temp = self.old_cntrl_temperatures[2] + (wheel3Status.drive_amps - 30)**2*dt/500
        
        
        wheel4Status = WheelStatus()
        wheel4Status.wheel_id        = 4
        wheel4Status.drive_rpm       = data.back_right_speed * 175.0
        wheel4Status.steering_angle  = data.back_right_angle
        wheel4Status.drive_amps      = data.back_right_speed * 150
        wheel4Status.steering_amps   = 1
        wheel4Status.drive_spin_count = wheel4Status.drive_rpm / 60.0 * dt
        wheel4Status.drive_temp      = self.old_drive_temperatures[3] + (wheel4Status.drive_amps - 30)**2*dt/100
        wheel4Status.steering_temp   = 40
        wheel4Status.controller_temp = self.old_cntrl_temperatures[3] + (wheel4Status.drive_amps - 30)**2*dt/500
        
        
        self.old_drive_temperatures = [wheel1Status.drive_temp, wheel2Status.drive_temp, wheel3Status.drive_temp, wheel4Status.drive_temp]
        self.old_steer_temperatures = [wheel1Status.steering_temp, wheel2Status.steering_temp, wheel3Status.steering_temp, wheel4Status.steering_temp]
        self.old_cntrl_temperatures = [wheel1Status.controller_temp, wheel2Status.controller_temp, wheel3Status.controller_temp, wheel4Status.controller_temp]
        
        
        self.pub.publish(wheel1Status)
        self.pub.publish(wheel2Status)
        self.pub.publish(wheel3Status)
        self.pub.publish(wheel4Status)
    #
    

# Intializes everything
def start():
    interpreter = MakeFakeWheelStatus() # subscribes to low-level commands
    
    # starts the node
    rospy.init_node('Raw2FakeStatus')
    rospy.spin()
#

if __name__ == '__main__':
    start()
    
