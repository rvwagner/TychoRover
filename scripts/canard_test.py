#!/usr/bin/python3
# WARNING: This script must be in python3 for SocketCAN support to work!


from canard import can
#from canard.hw import cantact
from canard.hw import socketcan

import rospy
from tycho.msg import WheelAnglesSpeeds
from tycho.msg import WheelStatus
from std_msgs.msg import Float32MultiArray

from enum import Enum
import struct # For converting incoming packet data to ints


#########
# SETUP #
#########

port = "can0"

# TODO: Organize this in an appropriate place
PacketTypes = Enum('PacketTypes', 'TPDO heartbeat other')

############################
# CAN Packet Handler Class #
############################

class TychoCANable:
  def __init__(self, port="can0"):
    self.specialCommands = {
    "estop" : {"header": 0x600, "dlc": 8,
               "data": [0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00]}
    }
    self.startCANable(port=port)
    
    self.frameCount=0
    self.lastMessageTime = rospy.Time(0)
    self.messageInterval = 1.0/20 # second mnumber is Hz
  #
  
  
  def startCANable(self, port="can0", bitrate=250000):
   # if port == "auto":
    #  print("Finding serial port")
     # port = "/dev/CAN"
    #
    
    print("Connecting to serial port '%s'"%port)
    #self.dev = cantact.CantactDev( port ) # just opens the serial port
    self.dev = socketcan.SocketCanDev( port )
    
    #print("Setting bitrate to 250k")
    #self.dev.set_bitrate(250000) # 250,000 = s5
    
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
      print(e)
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
    #if header == 0x201: print("Building frame 0x%03X : %d, %d"%(header, int1, int2))
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
    if not isinstance(data, int): data = int(round(data))
    
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
    
    data1 =  struct.unpack("<i", bytearray(frame.data[:4]))[0]
    data2 =  struct.unpack("<i", bytearray(frame.data[4:]))[0]
    
    return index, source, data1, data2
  #
  
  def readHeartbeat(self, frame):
    return frame.id & 0x07F # Node ID is 7 bits
  #
  
  
  
  
  def sendFrame(self, frame):
    #if (rospy.Time.now() - self.lastMessageTime).to_sec() < self.messageInterval:
    #        return
    self.lastMessageTime = rospy.Time.now()
    self.frameCount = self.frameCount+1
    #print("Sending frame %d: %s"%(self.frameCount,str(frame)) )
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
    print("Sending new PID values %f %f %f"%(p,i,d))
    
    for i in (1,2,3,4):
        canable.sendFrame(canable.buildRPDO(i, 3, int(p*10), int(i*10) ))
        canable.sendFrame(canable.buildRPDO(i, 4, int(d*10), 1         ))
        #canable.sendFrame(canable.buildSetVariable(i, 18, int(p*10) ))
        #canable.sendFrame(canable.buildSetVariable(i, 19, int(i*10) ))
        #canable.sendFrame(canable.buildSetVariable(i, 20, int(d*10) ))
        #canable.sendFrame(canable.buildSetVariable(i, 21, 1)) # Alert the controller to the change
#
def pid_callback(m):
    updatePID(m.data[0], m.data[1], m.data[2])
    return
    
    
    # Highest user variable I can actually write to is 16 (0x10)
    # The same variables the RPDOs and TPDOs write to.
    # Guess I'll need to use RPDOs 3 and 4 for PID editing for now.
    for var in (0x01, 0x10, 0x11):
        pid_test(var, int(m.data[0]))
#
def pid_test(var, val):
    print("Sending PID Update")
    
    frame = canable.buildRawCommand(2, 0x600, 8, [0x20,0x05,0x20,var,val,0x00,0x00,0x00])
    canable.sendFrame(frame)
    print (frame.id, frame.data)
    # To assemble other commands
    # [byte0,index2,index1,sub, data1,data2,data3,data4]
    #  Index |   Sub    |         Entry Name         | Size | Access | Command
    # 0x2003 | 01 to ee | Set Encoder Counter        | S32  |   WO   | C
    # canable.buildRawCommand(2, 0x600, 8, [0x20,0x03,0x20,0x02,0x0A,0x00,0x00,0x00]) # Works
    # 0x2106 | 01 to vv | Read User Integer Variable | S32  |   RO   | VAR
    # 0x2005 | 01 to vv | Set User Integer Variable  | S32  |   WO   | VAR
    
    frame = canable.buildRawCommand(2, 0x600, 8, [0x40,0x06,0x21,var,0x00,0x00,0x00,0x00])
    canable.sendFrame(frame)
    print (frame.id, frame.data)
    
    #setEncFrame = canable.buildRawCommand(2, 0x600, 8, [0x20,0x03,0x20,0x02,0x0B,0x00,0x00,0x00])
    #canable.sendFrame(setEncFrame)
    #print (setEncFrame.id, setEncFrame.data)
    
    #frame = canable.buildRawCommand(2, 0x600, 8, [0x40,0x04,0x21,0x02,0x00,0x00,0x00,0x00])
    #canable.sendFrame(frame)
    #print (frame.id, frame.data)
#

def sendHeartbeat(event):
    # TODO: Change the 400 to read MC_Speed_Scale from ROS params file
    canable.sendFrame(canable.buildRPDO(1, 2, 400, 1))
    canable.sendFrame(canable.buildRPDO(2, 2, 400, 1))
    canable.sendFrame(canable.buildRPDO(3, 2, 400, 1))
    canable.sendFrame(canable.buildRPDO(4, 2, 400, 1))
#

def sendESTOP():
    for i in (1,2,3,4):
        canable.sendFrame(canable.buildRawCommand(i, 0x600, 8, [0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00]))
    # Broadcast frame don't work
    #canable.sendFrame(canable.buildRawCommand(0, 0x600, 8, [0x2C,0x0C,0x20,0x00,0x01,0x00,0x00,0x00]))
#

def clearESTOP(node):
    canable.sendFrame(canable.buildRawCommand(node, 0x600, 8, [0x2C,0x0D,0x20,0x00,0x01,0x00,0x00,0x00]))
#



def new_command_callback(data):
    # print("Recieved command: %.2f mm/s at %.1f deg"%(data.back_left_speed, data.back_left_angle))
    canable.sendFrame(canable.buildRPDO(frontLeftID, 1, round(data.front_left_speed * speedMultiplier), round(data.front_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(frontRightID, 1, round(data.front_right_speed * speedMultiplier), round( data.front_right_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backLeftID, 1, round(data.back_left_speed * speedMultiplier), round(data.back_left_angle * angleMultiplier)))
    canable.sendFrame(canable.buildRPDO(backRightID, 1, round(data.back_right_speed * speedMultiplier), round(data.back_right_angle * angleMultiplier)))
#

# TODO: Verify value scaling
def handleTPDO(index, node, data1, data2):
	if index == 1: # TPDO1: RMP 1, Amps 1
		wheel_statuses[node].drive_rpm = data1 # TODO: convert to rpm or to m/s
		wheel_statuses[node].drive_amps = abs(data2)
		ready_to_send[node] |= 0x1
	elif index == 2: # TPDO2: Count 1, MCU Temp
		wheel_statuses[node].drive_spin_count = data1
		wheel_statuses[node].controller_temp = data2 / 10.0
		ready_to_send[node] |= 0x2
	elif index == 3: # TPDO3: Angle 2, Amps 2
		wheel_statuses[node].steering_angle = data1 / 10.0
		wheel_statuses[node].steering_amps = abs(data2)
		ready_to_send[node] |= 0x4
	elif index == 4: # TPDO4: Temp 1, Temp 2
		wheel_statuses[node].drive_temp = data1 / 10.0
		wheel_statuses[node].steering_temp = data2 / 10.0
		ready_to_send[node] |= 0x8
	else:
		return # TODO: Throw an error
	#
	
	if ready_to_send[node] == 0xF:
		#print("Publishing %d: % 6.1f % 6.1f % 8d % 6.1f % 6.1f % 6.1f % 6.1f % 6.1f"%(node, wheel_statuses[node].drive_rpm, wheel_statuses[node].drive_amps, wheel_statuses[node].drive_spin_count, wheel_statuses[node].controller_temp, wheel_statuses[node].steering_angle, wheel_statuses[node].steering_amps, wheel_statuses[node].drive_temp, wheel_statuses[node].steering_temp))
		statuspub.publish(wheel_statuses[node])
		ready_to_send[node] = 0x0
	#
#


#############
# Main Loop #
#############

canable = TychoCANable(port=port)

frontLeftID  = 1
frontRightID = 2
backLeftID   = 3
backRightID  = 4
speedMultiplier = 1
angleMultiplier = 10

# starts the node
rospy.init_node('CAN_Handler')

print("Subscribing to topic")
# Subscribe to low-level commands topic
rospy.Subscriber("tycho/low_level_motor_values", WheelAnglesSpeeds, new_command_callback, queue_size=1)

# Subscribe to PID update topic
rospy.Subscriber("tycho/pid", Float32MultiArray, pid_callback, queue_size=1)
    
# Publish topic with current wheel sensor statuses
# TODO: Maybe break this up into a couple of topics?
statuspub = rospy.Publisher('tycho/wheel_status', WheelStatus, queue_size=1)


ready_to_send = [0x0,0x0,0x0,0x0,0x0]
wheel_statuses = [0, WheelStatus(), WheelStatus(), WheelStatus(), WheelStatus()]
for i in (1,2,3,4): wheel_statuses[i].wheel_id = i


# Trying broadcast

print("Sending initial frame")
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
print ("Starting main loop")
while not rospy.is_shutdown():
  #rate.sleep()
  #continue
  
  
  f, packet_type = canable.receiveFrameWithType()
  # print(f, packet_type)
  if packet_type == PacketTypes.TPDO:
    index, node, data1, data2 = canable.readTPDO(f)
    handleTPDO(index, node, data1, data2)
  elif packet_type == PacketTypes.heartbeat:
    node = canable.readHeartbeat(f)
    #print("Heartbeat ", node)
    lastHeartbeat[node] = now
  else: # Unknwon packet type
    #pass
    print("%03X"%f.id, f.data)
  #
  
  # Check heartbeats
  for i in (1,2,3,4):
    if now - lastHeartbeat[i] > maxHeartbeatDelay:
      # TODO: Send estop if a controller is lost!
      rospy.logwarn("Lost controller %d!"%i)
      # sendESTOP()
    #
  #
  rate.sleep()
#

# Not needed for socketcan?
#canable.stop()

