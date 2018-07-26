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
ser = serial.Serial(port, 9600, timeout=1)
ser.write(b'\r')
ser.close()
print("Done")

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
    if header == 0x201: print("Building frame 0x%03X : %d, %d"%(header, int1, int2))
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
  
  
  def buildSetVariable(self, dest, var_id, data):
    # TODO: Check for overflow when fixing inputs
    if not isinstance(int1, int): int1 = int(round(int1))
    if not isinstance(int2, int): int2 = int(round(int2))
    
    # Frame ID is 0x600 + node ID
    header = (0x600)|dest
    frame = can.Frame(header)
    frame.dlc = 8
    frame.data = [
      0x20,            # Type = Command (2), no unused bytes
      0x05, 0x20,      # Set User Integer Variable
      var_id    &0xFF, # Variable index
       data     &0xFF, # Data (4 bytes)
      (data>>8) &0xFF,
      (data>>16)&0xFF,
      (data>>24)&0xFF,
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
  
  def readHeartbeat(self, frame):
    return frame.id & 0x07F # Node ID is 7 bits
  #
  
  
  
  
  def sendFrame(self, frame):
    #print("Sending frame "+str(frame))
    return self.dev.send(frame)
  #
  
  def queueFrame(self, frame):
    self.frame=frame
  #
  
  def receiveFrame(self):
    return self.dev.recv()
  #
  
  def receiveFrameWithType(self):
    frame=self.receiveFrame()
    frameType = self.getPacketType(frame)
    return frame, frameType
  #
  
  def stop(self):
    return self.dev.stop()
  #
# END CLASS




def updatePID(p,i,d):
    # Trying broadcast frames
    canable.sendFrame(canable.buildSetVariable(0, 18, p))
    canable.sendFrame(canable.buildSetVariable(0, 19, i))
    canable.sendFrame(canable.buildSetVariable(0, 20, d))
    canable.sendFrame(canable.buildSetVariable(0, 21, 1)) # Alert the controller to the change
#

def sendHeartbeat(event):
    # Trying broadcast frames
    # TODO: Change the 400 to read MC_Speed_Scale from ROS params file
    canable.sendFrame(canable.buildRPDO(0, 4, 400, 1))
#

def sendESTOP():
    canable.sendFrame(canable.buildRawCommand(0, 0x600, 8, [0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00]))
#

def clearESTOP(node):
    canable.sendFrame(canable.buildRawCommand(node, 0x600, 8, [0x2C,0x0D,0x20,0x00,0x01,0x00,0x00,0x00]))
#


frontLeftID  = 1
frontRightID = 2
backLeftID   = 3
backRightID  = 4
speedMultiplier = 100
angleMultiplier = 10

def new_command_callback(data):
    #print("Recieved command: %.2f m/s at %.1f deg"%(data.back_left_speed, data.back_left_angle))
    canable.sendFrame(canable.buildRPDO(frontLeftID, 1, round(data.front_left_speed * speedMultiplier), round(data.front_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(frontRightID, 1, round(data.front_right_speed * speedMultiplier), round( data.front_right_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backLeftID, 1, round(data.back_left_speed * speedMultiplier), round(data.back_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backRightID, 1, round(data.back_right_speed * speedMultiplier), round(data.back_right_angle * angleMultiplier)))
#

# starts the node

print("Subscribing to topic")
# Subscribe to low-level commands topic
rospy.Subscriber("tycho/low_level_motor_values", WheelAnglesSpeeds, new_command_callback)
    
# Publish topic with current wheel sensor statuses
# TODO: Maybe break this up into a couple of topics?
statuspub = rospy.Publisher('tycho/wheel_status', WheelStatus, queue_size=10)

# starts the node
rospy.init_node('CAN_Handler')


ready_to_send = [0x0,0x0,0x0,0x0,0x0]
wheel_statuses = [0, WheelStatus(), WheelStatus(), WheelStatus(), WheelStatus()]
for i in (1,2,3,4): wheel_statuses[i].wheel_id = i

# TODO: Scale these values
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

# Trying broadcast

print "Sending initial frame"
frame = canable.buildRawCommand(1, 0x200, 8, [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
canable.sendFrame(frame)
print(frame)

canable.sendFrame(canable.buildRPDO(frontLeftID, 1, 0, 0))
canable.sendFrame(canable.buildRPDO(frontRightID, 1, 0, 0))
canable.sendFrame(canable.buildRPDO(backLeftID, 1, 0, 0))
canable.sendFrame(canable.buildRPDO(backRightID, 1, 0, 0))

# TODO: Add bootup frames
print("Frame sent")


# Set up heartbeat timer- will fire every 200ms
# http://wiki.ros.org/rospy/Overview/Time#Timer
heartbeat_timer = rospy.Timer(rospy.Duration(0.2), sendHeartbeat)
now = rospy.get_rostime()
lastHeartbeat = [0, now, now, now, now]
maxHeartbeatDelay = rospy.Duration(0.5) # Delay until a controller is declared lost

count = 0
rate = rospy.Rate(100) # Hz
while not rospy.is_shutdown():
  now = rospy.get_rostime()
  
count = 0
rate = rospy.Rate(100) # Hz
while not rospy.is_shutdown():
  f, packet_type = canable.receiveFrameWithType()
  if packet_type == PacketTypes.TPDO:
    index, node, data1, data2 = canable.readTPDO(f)
    handleTPDO(index, node, data1, data2)
  elif packet_type == PacketTypes.heartbeat:
    node = canable.readHeartbeat(f)
    lastHeartbeat[node] = now
  else: # Unknwon packet type
    pass
    #print "%03X"%f.id, f.data
  #
  
  # Check heartbeats
  for i in (1,2,3,4):
    if now - lastHeartbeat[i] > maxHeartbeatDelay:
      # TODO: Send estop if a controller is lost!
      rospy.logwarn("Lost controller %d!", i)
      # sendESTOP()
    #
  #
  rate.sleep()
#


canable.stop()

