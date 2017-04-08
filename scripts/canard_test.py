#!/usr/bin/python

from canard import can
from canard.hw import cantact

from glob import glob
from enum import Enum
import struct

PacketTypes = Enum('PacketTypes', 'TPDO heartbeat other')

#########
# SETUP #
#########
# This code just magically started working for no apparent reason.  Is some Ubuntu process sniffing at any freshly-loaded serial ports for several minutes?

port = glob("/dev/ttyACM?")[-1]
dev = cantact.CantactDev( port )
dev.set_bitrate(250000) # 250,000 = s5
dev.start()

# Pre-build special commands
estop_frame = can.Frame(0x601)
estop_frame.dlc=8
estop_frame.data=[0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00]


#######################
# CAN Packet Handlers #
#######################

def getPacketType(frame):
  # Remove bottom 7 bits (source node ID)
  numeric_type = (frame.id & 0xF80)
  if numeric_type in (0x180, 0x280, 0x380, 0x480):
    return PacketTypes.TPDO
  if numeric_type == 0x700:
    return PacketTypes.heartbeat
  else:
    return PacketTypes.other
#

def buildRPDO(dest, index, int1, int2):
  # Frame ID is 0x200/300/400/500 + node ID
  header = (0x100 + 0x100*index)|dest
  print "Building frame %03X : %d, %d"%(header, int1, int2)
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
def readTPDO(frame):
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




#############
# Main Loop #
#############

# Initial data send
frame = can.Frame(0x201)
frame.dlc=8
frame.data=[0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

dev.send(frame)
print(frame)

count = 0
while True:
  f = dev.recv()
  packet_type = getPacketType(f)
  if packet_type == PacketTypes.TPDO:
    index, node, data1, data2 = readTPDO(f)
    print "TPDO%d: %d, %d"%(index, data1, data2)
    count += 1
    if count % 10 == 0:
      frame = buildRPDO(1,1,data2+1, 0)
      print(frame)
      dev.send(frame)
    elif count > 100:
      dev.send(estop_frame)
  elif packet_type == PacketTypes.heartbeat:
    pass
  else: # Unknwon packet type
    print "%03X"%f.id, f.data

