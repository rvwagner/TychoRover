#!/usr/bin/python

from canard import can
from canard.hw import cantact

import rospy
from tycho.msg import WheelAnglesSpeeds

from glob import glob # For auto-detecting serial port (to be improved)
from enum import Enum
import struct # For converting incoming packet data to ints


#########
# SETUP #
#########
# This code just magically started working for no apparent reason.  Is some Ubuntu process sniffing at any freshly-loaded serial ports for several minutes?

# PySerial test.
# Doesn't seem to do any better than the CANtact version, but
# it's less opaque.
if False: # Serial test
  print "Finding serial port"
  port = glob("/dev/ttyACM?")[-1]
  print "Found serial port '%s'"%port
  import serial
  ser = serial.Serial(port, 9600, timeout=0)
  print "reading..."
  ser.read(10)
  print "starting up"
  ser.write(b'S5\r') #.encode())
  ser.write(b'O\r') #.encode())
  print "reading..."
  ser.read(10)
  ser.write(b'C\r') #.encode())
  print "done"
  exit()
#

# TODO: Organize this in an appropriate place
PacketTypes = Enum('PacketTypes', 'TPDO heartbeat other')

#######################
# CAN Packet Handlers #
#######################

class TychoCANable:
  def __init__(self):
    self.specialCommands = {
    "estop" : {"header": 0x600, "dlc": 8,
               "data": [0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00]}
    }
    self.startCANable()
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
    if isinstance(int1, int): int1 = int(round(int1))
    if isinstance(int2, int): int2 = int(round(int2))
    
    # Frame ID is 0x200/300/400/500 + node ID
    header = (0x100 + 0x100*index)|dest
    print("Building frame %03X : %d, %d"%(header, int1, int2))
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


#############
# Main Loop #
#############

canable = TychoCANable()

print "Sending initial frame"
frame = canable.buildRawCommand(1, 0x200, 8, [0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00])
canable.sendFrame(frame)
print(frame)

count = 0
while True:
  f, packet_type = canable.receiveFrameWithType()
  if packet_type == PacketTypes.TPDO:
    index, node, data1, data2 = canable.readTPDO(f)
    print "TPDO%d: %d, %d"%(index, data1, data2)
    count += 1
    if count % 10 == 0:
      frame = canable.buildRPDO(1,1,data2+1, 0)
      print(frame)
      canable.sendFrame(frame)
    #elif count > 100:
    #  dev.send(estop_frame)
  elif packet_type == PacketTypes.heartbeat:
    pass
  else: # Unknwon packet type
    print "%03X"%f.id, f.data
#




frontLeftID  = 1
frontRightID = 2
backLeftID   = 3
backRightID  = 4
speedMultiplier = 1.0
angleMultiplier = 1.0

def new_command_callback(self, data):
    canable.sendFrame(canable.buildRPDO(frontLeftID, 1, round(data.front_left_speed * speedMultiplier), round(data.front_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(frontRightID, 1, round(data.front_right_speed * speedMultiplier), round( data.front_right_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backLeftID, 1, round(data.back_left_speed * speedMultiplier), round(data.back_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backRightID, 1, round(data.back_right_speed * speedMultiplier), round(data.back_right_angle * angleMultiplier)))
#


# starts the node
def start():
    # Subscribe to low-level commands topic
    rospy.Subscriber("tycho/low_level_motor_values", WheelAnglesSpeeds, new_command_callback)
    
    # Publish topic with current wheel sensor statuses
    # TODO: Maybe break this up into a couple of topics?
    pub = rospy.Publisher('tycho/wheel_status', WheelAnglesSpeeds)
    
    # starts the node
    rospy.init_node('CAN_Handler')
    rospy.spin()
#


dev.stop()

