#!/usr/bin/python3

import rospy
from tycho.msg import WheelAnglesSpeeds
from tycho.msg import WheelStatus
from tycho.msg import RoverDriveCommand
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16


#########
# SETUP #
#########


class RoverDriveCommandLogger:
    def __init__(self, directory):
        rospy.Subscriber("tycho/high_level_motor_values", RoverDriveCommand, self.log, queue_size=100)
        self.file = open("%s/high_level_command_log.txt"%directory, 'w')
    #
    
    def closeout(self):
        self.file.close()
    #
    
    def add_flag(self):
        t = datetime.datetime.today().strftime("%Y-%m-%d %H:%M:%S.%f")
        self.file.write("%s  FLAGGED EVENT "%t)
    #
    
    def log(self, msg):
        node=msg.wheel_id
        dt = datetime.datetime.fromtimestamp(msg.header.stamp.to_time())
        timestamp = dt.strftime("%Y-%m-%d %H:%M:%S.%f")
        self.file.write("%s  % 6.2f % 6.2f % 6.2f % 6.2f %d %d\n"%(timestamp, msg.speed, msg.turn_center_x, msg.turn_center_y, msg.strafing_angle, msg.is_strafing, msg.is_braking))
    #
#

class WheelAnglesSpeedsLogger:
    def __init__(self, directory):
        rospy.Subscriber("tycho/low_level_motor_values", WheelAnglesSpeeds, self.log, queue_size=100)
        self.file = open("%s/low_level_command_log.txt"%directory, 'w')
    #
    
    def closeout(self):
        self.file.close()
    #
    
    def add_flag(self):
        t = datetime.datetime.today().strftime("%Y-%m-%d %H:%M:%S.%f")
        self.file.write("%s  FLAGGED EVENT "%t)
    #
    
    def log(self, msg):
        node=msg.wheel_id
        dt = datetime.datetime.fromtimestamp(msg.header.stamp.to_time())
        timestamp = dt.strftime("%Y-%m-%d %H:%M:%S.%f")
        self.file.write("%s  % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f % 6.2f\n"%(timestamp, msg.front_left_angle, msg.front_left_speed, msg.front_right_angle, msg.front_right_speed, msg.back_left_angle, msg.back_left_speed, msg.back_right_angle, msg.back_right_speed))
    #
#


class WheelStatusLogger:
    def __init__(self, directory):
        rospy.Subscriber("tycho/wheel_status", WheelStatus, self.log, queue_size=100)
        # open files for writing
        self.nodefilelist = []
        self.nodefilelist.append(open("%s/node_1_responses_log.txt"%directory, 'w'))
        self.nodefilelist.append(open("%s/node_2_responses_log.txt"%directory, 'w'))
        self.nodefilelist.append(open("%s/node_3_responses_log.txt"%directory, 'w'))
        self.nodefilelist.append(open("%s/node_4_responses_log.txt"%directory, 'w'))
    #
    
    def closeout(self):
        #Close all files
        self.nodefilelist[0].close()
        self.nodefilelist[1].close()
        self.nodefilelist[2].close()
        self.nodefilelist[3].close()
    #
    
    def add_flag(self):
        t = datetime.datetime.today().strftime("%Y-%m-%d %H:%M:%S.%f")
        for node in [0, 1, 2, 3]:
            self.nodefilelist[node].write("%s  FLAGGED EVENT "%t)
    #
    
    def log(self, msg):
        node=msg.wheel_id
        dt = datetime.datetime.fromtimestamp(msg.header.stamp.to_time())
        timestamp = dt.strftime("%Y-%m-%d %H:%M:%S.%f")
        # Drive/steer coord debugging data
        self.nodefilelist[node-1].write("%s  Node %d: Drive: % 6.0f -> % 6.0f = % 6.0f; Steer: t % 6.0f = m % 6.0f ? cmd % 6.0f act % 6.0f\n"%(timestamp, node, msg.drive_rpm, msg.drive_amps, msg.drive_spin_count, msg.controller_temp, msg.steering_temp, msg.steering_amps, msg.drive_temp))
        #print("%s  Node %d: % 6.1f % 6.1f % 8d % 6.1f % 6.1f % 6.1f % 6.1f % 6.1f"%(timestamp, node, msg.drive_rpm, msg.drive_amps, msg.drive_spin_count, msg.controller_temp, msg.steering_temp, msg.steering_amps, msg.drive_temp))
    #
#



#############
# Main Loop #
#############

import datetime
import os

t = datetime.datetime.today()

# Set up log directory
# ~/tycho_logs/{date}_{time}
logdir = t.strftime("/home/pi/tycho_logs/%Y%m%d_%H%M%S")
os.makedirs(logdir, exist_ok=True)


# starts the node
rospy.init_node('Logger')

# Set up logging objects
print("Subscribing to topics")
wsl = WheelStatusLogger(logdir)
rdcl = RoverDriveCommandLogger
wasl = WheelAnglesSpeedsLogger


print ("Starting main loop")
rate = rospy.Rate(1) # Hz
while not rospy.is_shutdown():
    rate.sleep()
#

wsl.closeout()
rdcl.closeout()
wasl.closeout()

