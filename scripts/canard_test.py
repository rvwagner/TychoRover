#!/usr/bin/python

from canard import can
from canard.hw import cantact

import rospy
from tycho.msg import WheelAnglesSpeeds
from tycho.msg import WheelStatus

from glob import glob # For auto-detecting serial port (to be improved)
from enum import Enum
import struct # For converting incoming packet data to ints


#########
# SETUP #
#########

# Give serial port a swift kick in the pants
# Without opening, writing to, and closing the port, it apparently forgets
# how to write once the CANable stuff starts (full output buffer?).
import serial
port = glob("/dev/ttyACM?")[-1]
print "Executing serial SKITP protocol on %s"%port
ser = serial.Serial(port, 9600, timeout=1)
ser.write(b'\r')
ser.close()

# TODO: Organize this in an appropriate place
PacketTypes = Enum('PacketTypes', 'TPDO heartbeat other')

############################
# CAN Packet Handler Class #
############################

class TychoCANable:
  def __init__(self, port="auto"):
    self.specialCommands = {
    "estop" : {"header": 0x600, "dlc": 8,
               "data": [0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00]}
    }
    self.startCANable(port=port)
  #
  
  
  def startCANable(self, port="auto", bitrate=250000):
    if port == "auto":
      print("Finding serial port")
      port = glob("/dev/ttyACM?")[-1]
    #
    
    print("Connecting to serial port '%s'"%port)
    self.dev = cantact.CantactDev( port ) # just opens the serial port
    
    print("Setting bitrate to 250k")
    self.dev.set_bitrate(250000) # 250,000 = s5
    
    print("Starting CANable")
    self.dev.start()
  #
  
  def buildRawCommand(self, dest, headerBase, dlc, data):
    frame = can.Frame(headerBase | dest)
    frame.dlc=dlc
    frame.data=data
    return frame
  #
  #estop_frame = buildRawCommand(1, 0x600, 8, [0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00])
  
  def getSpecialCommand(self, dest, commandType):
    try:
      commandInfo = self.specialCommands[commandType]
    except e:
      print e
      # TODO: Better handling
      return False
    return self.buildRawCommand(dest, commandInfo.header, commandInfo.dlc, commandInfo.data)
  #
  
  def getPacketType(self, frame):
    # Remove bottom 7 bits (source node ID)
    numeric_type = (frame.id & 0xF80)
    if numeric_type in (0x180, 0x280, 0x380, 0x480):
      return PacketTypes.TPDO
    if numeric_type == 0x700:
      return PacketTypes.heartbeat
    else:
      return PacketTypes.other
  #
  
  def buildRPDO(self, dest, index, int1, int2):
    # TODO: Check for overflow when fixing inputs
    if not isinstance(int1, int): int1 = int(round(int1))
    if not isinstance(int2, int): int2 = int(round(int2))
    
    # Frame ID is 0x200/300/400/500 + node ID
    header = (0x100 + 0x100*index)|dest
    #print("Building frame %03X : %d, %d"%(header, int1, int2))
    frame = can.Frame(header)
    frame.dlc = 8
    frame.data = [
       int1     &0xFF,
      (int1>>8) &0xFF,
      (int1>>16)&0xFF,
      (int1>>24)&0xFF,
       int2     &0xFF,
      (int2>>8) &0xFF,
      (int2>>16)&0xFF,
      (int2>>24)&0xFF,
    ]
    return frame
  #
  
  # Take frame, returns TPDO ID, source node ID, and both integer values
  # TPDO ID will be 0 if the packet is not a TPDO
  def readTPDO(self, frame):
    source = frame.id & 0x07F # Node ID is 7 bits
    packet_type = frame.id & 0xF80
    index = packet_type >> 8
    #print "%03X : %03X %d %d"%(frame.id, packet_type, index, source)
    if not packet_type & 0x080 or not 0 < index < 5:
      return 0,0,0,0
    # print "%02X %02X %02X %02X %02X %02X %02X %02X"%tuple(frame.data)
    
    data1 =  struct.unpack("<i", str(bytearray(frame.data[:4])))[0]
    data2 =  struct.unpack("<i", str(bytearray(frame.data[4:])))[0]
    
    return index, source, data1, data2
  #
  
  def sendFrame(self, frame):
    return self.dev.send(frame)
  #
  
  def receiveFrame(self):
    return self.dev.recv()
  #
  
  def receiveFrameWithType(self):
    frame=self.receiveFrame()
    frameType = self.getPacketType(frame)
    return frame, frameType
  #
# END CLASS







frontLeftID  = 1
frontRightID = 2
backLeftID   = 3
backRightID  = 4
speedMultiplier = 100
angleMultiplier = 10

def new_command_callback(data):
    canable.sendFrame(canable.buildRPDO(frontLeftID, 1, round(data.front_left_speed * speedMultiplier), round(data.front_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(frontRightID, 1, round(data.front_right_speed * speedMultiplier), round( data.front_right_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backLeftID, 1, round(data.back_left_speed * speedMultiplier), round(data.back_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backRightID, 1, round(data.back_right_speed * speedMultiplier), round(data.back_right_angle * angleMultiplier)))
#

# starts the node

# Subscribe to low-level commands topic
rospy.Subscriber("tycho/low_level_motor_values", WheelAnglesSpeeds, new_command_callback)
    
# Publish topic with current wheel sensor statuses
# TODO: Maybe break this up into a couple of topics?
statuspub = rospy.Publisher('tycho/wheel_status', WheelStatus)

# starts the node
rospy.init_node('CAN_Handler')


ready_to_send = [0x0,0x0,0x0,0x0,0x0]
wheel_statuses = [0, WheelStatus(), WheelStatus(), WheelStatus(), WheelStatus()]
for i in (1,2,3,4): wheel_statuses[i].wheel_id = i

def handleTPDO(index, node, data1, data2):
	if index == 1: # TPDO1: RMP 1, Amps 1
		wheel_statuses[node].drive_rpm = data1
		wheel_statuses[node].drive_amps = data2
		ready_to_send[node] |= 0x1
	elif index == 2: # TPDO2: Count 1, MCU Temp
		wheel_statuses[node].drive_spin_count = data1
		wheel_statuses[node].controller_temp = data2
		ready_to_send[node] |= 0x2
	elif index == 3: # TPDO3: Angle 2, Amps 2
		wheel_statuses[node].steering_angle = data1
		wheel_statuses[node].steering_amps = data2
		ready_to_send[node] |= 0x4
	elif index == 4: # TPDO4: Temp 1, Temp 2
		wheel_statuses[node].drive_temp = data1
		wheel_statuses[node].steering_temp = data2
		ready_to_send[node] |= 0x8
	else:
		return # TODO: Throw an error
	#
	
	if ready_to_send[node] == 0xF:
		print "Publishing % 5d % 5d % 5d % 5d % 5d % 5d % 5d % 5d"%(wheel_statuses[node].drive_rpm, wheel_statuses[node].drive_amps, wheel_statuses[node].drive_spin_count, wheel_statuses[node].controller_temp, wheel_statuses[node].steering_angle, wheel_statuses[node].steering_amps, wheel_statuses[node].drive_temp, wheel_statuses[node].steering_temp)
		statuspub.publish(wheel_statuses[node])
		ready_to_send[node] = 0x0
	#
#


#############
# Main Loop #
#############

canable = TychoCANable(port=port)



print "Sending initial frame"
frame = canable.buildRawCommand(1, 0x200, 8, [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
canable.sendFrame(frame)
print(frame)

count = 0
while True:
  f, packet_type = canable.receiveFrameWithType()
  if packet_type == PacketTypes.TPDO:
    index, node, data1, data2 = canable.readTPDO(f)
    handleTPDO(index, node, data1, data2)
 #   print "TPDO%d: %d, %d"%(index, data1, data2)
 #   count += 1
 #   if count % 10 == 0:
 #     frame = canable.buildRPDO(1,1,data2+1, 0)
 #     print(frame)
 #     canable.sendFrame(frame)
    #elif count > 100:
    #  dev.send(estop_frame)
  elif packet_type == PacketTypes.heartbeat:
    pass
  else: # Unknwon packet type
    print "%03X"%f.id, f.data
#


dev.stop()

