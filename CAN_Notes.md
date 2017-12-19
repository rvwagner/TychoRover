# CAN/CANable/Ubuntu Notes

CANable info page:
canable.io

Need to install can-utils:

    sudo apt install can-utils

On Raspberry Pi, also need to install slcan kernel module separately:
https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi

To get CANard installed,

    git clone https://github.com/normaldotcom/CANard.git
    cd CANard
    [sudo] python setup.py install
    [cd ..; rm -rf CANard/]

- - - 

To test:

    sudo modprobe can
    sudo modprobe slcan
    sudo slcand -o -c -s5 /dev/ttyACM0 can0
    sudo ifconfig can0 up

To automatically load the kernel modules on boot, add the lines "can" and "slcan" to `/etc/modules`.

Do not use slcand if you are using CANard's cantact module.

Test commands:

    candump can0 & # Or run without backgrounding in a separate window
    cansend can0 201#000A.002D

Be warned that apparently if there isn't something on the other end recieving messages, the transmit buffer fills up after ~10 messages.


# CAN and RoboTeq Controllers

CAN commands are always executed
MicroBASIC is identical priority to Serial, and are also subject to the watchdog timer.
In non-Serial modes, the controller will still respond to info queries over serial wihtout affecting control. DItto queries over CAN.


Serial section starts page 141
- They advise against using USB connection in the field- RS232 is more resilient to eletrical disruptions.
- Commands are not case-sensitive
- Underscore is an alias of carriage return
- Default watchdog is 1sec


CAN sections are 149 and ... and 169 (CANopen)
- Supports RawCAN, MiniCAN, CANopen (I know that's a ROS node for this), and RoboCAN
- - MiniCAN is a subset of OpenCAN
- - RoboCAN is proprietary, but designed for networking RoboTeq controllers
- Some controllers disable CAN while USB is connected- check datasheet
- A CAN packet/frame is 11-bit header and 0-8 byte LSB payload
- CAN packets can/should/must be interpreted by MicroBASIC scripts :-D
- In MiniCAN mode, incoming packets are automatically written to VAR9-VAR16

- RoboCAN is interesting, but not useful for us without building a custom RoboCAN-interpreting node.  Since a MiniCAN node would also need to be custom (but might be possible to build with just config files using ros_canopen), there will be no particular effort savings.

- Need to refer to general CAN documentation for bit-rate given our cable length.  Can probably hit the 1Mbps level, though default is 125kbps.

- CANopen may be a better way to go, assuming there's a general way to send data- they have a CANopen-compliant dictionary on page 173, which should allow building the ros_canopen config files.
- - As in MiniCAN, Controller will automatically transmit up to four types of packet containing settable values (VAR1-8) at an adjustable frequency (TPDO1-4).  Slight difference from MiniCAN in that only 8 vars are transmitted, as opposed to 4 4-byte, 4 2-byte and 32 bools. (Verify this in practice.)
- - As in MiniCAN, the RPDO1-4 packets write to VAR9-16. (Same difference as above.)
- - New part from CANopen is the ability to request and command specific values, bypassing the MicroBASIC script (such as e-stop)
- https://www.amazon.com/dp/B00ZW8N930/
- https://github.com/b/libusbcan
- This seems to be the go-to CAN adaptor: ~$200: http://www.peak-system.com/PCAN-USB.199.0.html?&L=1
- Here's a $25 version (out of stock) that is likely to do everything we need: http://protofusion.org/wordpress/2015/05/the-canable-a-small-usb-to-can-adapter/

- - -

MicroBASIC
- Integer (32bit) and boolean variables only - limit 1024/4096 ints and 1024 bools
- (I'll send the motor commands in RPM and radians, in order to avoid any need to update the controller code if the system layout changes?  No, it needs to know the tire diameter and steering arm length to function, so I might need to make those writable...)
- All variables set to 0/false on script first load/controller reset
- One of their suggested script actions is the limit the amp range based on heatsink temperature.
- scripts are activated every 1ms, after motion control processing is done.  Motor control takes priority.  They estimate ~50 lines of execution per activation.  Serial print is limited to 32 char/activation.
- Can upload scripts saved in .hex format from PC Utility over serial (page 183)
- NEVER set a script to auto-run unless you've made sure it doesn't crash immediately- if it does, you may need to reflash the firmware.

- - -

NUC-side CAN node:
- probably use ros_canopen for the actual communication
- Have a custom node watching the CAN node's messages
- - Track heartbeats, if any node goes silent for too long, throw an error
- - - Some level of control system should throw an estop (or harsh speed limit) if a controller goes out
- - - Probably the same level that keeps track of what the motor parameters are- call it "self-analysis"
- - When a status message (TPDO) comes in, translate to the appropriate "sensor" topics based on source ID, message type
- - If I do a single "motor status" topic, then send either when heartbeat fails or when last TPDO of the set from a controller comes in
- May have a separate node for transmission (why not?)
- - When a motor_command_final comes in, translate into CAN messages, publish to whatever ros_canopen takes
- - When an e-stop command comes in, send e-stop broadcast, as well as targeted "stop" messages.


## Manual NUC-side notes ##

CANsend documentation:
https://github.com/linux-can/can-utils/blob/master/cansend.c

Packet format:
    [three-hex-id]#[0-8 char hex data, with optional '.' separators]
    [header 0x000]#[data 0x00...]


Outgoing from controller: (not sure what Node ID is, as there isn't an obvious way to set it...  Probably broadcast.)
TPDO1: 0x180 & Node ID
TPDO2: 0x280 & Node ID
TPDO3: 0x380 & Node ID
TPDO4: 0x480 & Node ID

Incoming to controller:
RPDO1: 0x200 & Node ID
RPDO2: 0x300 & Node ID
RPDO3: 0x400 & Node ID
RPDO4: 0x500 & Node ID

So, to send RPDO1 to Node 1 with data 10,45:
201#000A.002D


E-stop Node 1 (see page 175):
Not sure what the "01" in the data is about. Seems unused.
601#8.2C.200C.00.01000000     # E-stop all channels
601#8.2C.200D.00.01000000     # Release e-stop
601#8.2C.200E.00.01000000     # "Stop [channel 1] in all modes", arg is channel


0x200E: Stop in all modes (command name MS)
- "The MS command is similar to the EX emergency stop command except that it is applied to the speci ed motor channel"

setcommand(_MSTOP, channel)

- - -


# MicroBASIC Script #


' See Page 210 for command, page 227 for queries; page 305 for config values ; page 344 for CAN bus

' Input: speed, in tenths of an RPM
'        angle, in tenths of a degree
'        scale factor = steerArmLength/tireRadius * 100
'        Heartbeat: any non-0 value that must be sent more often than HEARTBEAT_RECIEVE_RATE

#define STEER_BRAKE_PIN xx
#define DRIVE_BRAKE_PIN xx
#define COMMANDED_SPEED VAR9
#define COMMANDED_ANGLE VAR10
#define SPEED_SCALE_FACTOR_INPUT VAR11
#define NUC_IS_ALIVE VAR12
#define DATA_SEND_RATE 200
#define HEARTBEAT_RECIEVE_RATE 1000

' Use defines for readability and to allow for easy reversal of whether high or low activates the brakes
' _D0 is set pin low, _D1 is set pin high
#define BRAKES_ON _D0
#define BRAKES_OFF _D1

#define REPORT_TIMER = 1
#define KEEPALIVE_TIMER = 2

' First, make certain the brakes are on.
' They will be on when the controller is off.
SetCommand(BRAKES_ON, DRIVE_BRAKE_PIN)
SetCommand(BRAKES_ON, STEER_BRAKE_PIN)

' Set up status reporting
SetConfig(_CEN, 1)                  ' Start CANopen (comes with a default 100ms heartbeat)
SetConfig(_CTPS, 1, DATA_SEND_RATE) ' Send VAR1,2 every DATA_SEND_RATE ms
SetConfig(_CTPS, 1, DATA_SEND_RATE) ' Send VAR3,4 every DATA_SEND_RATE ms
SetConfig(_CTPS, 1, DATA_SEND_RATE) ' Send VAR5,6 every DATA_SEND_RATE ms
SetConfig(_CTPS, 1, 0)              ' Do not send VAR7,8

' Other initialization stuff
Max_Speed = GetConfig(MXRPM, 1)

' Wait for the controlling computer to check in and set the scaling parameter
GoSub SafeUntilSignal
Speed_Scale_Factor = SPEED_SCALE_FACTOR_INPUT

SetTimerCount(REPORT_TIMER, DATA_SEND_RATE)
SetTimerCount(KEEPALIVE_TIMER, HEARTBEAT_RECIEVE_RATE)

loop:
    ' (adjustments based on temperature?)
    
    Current_Steering_Speed = GetConfig(_S, 2) 'Get actual steering speed in RPM
    Current_Drive_Speed = GetConfig(_S, 1) 'Get actual steering speed in RPM
    
    ' (Current_Steering_Speed * Speed_Scale_Factor / 100) is the angular
    '  speed needed to coordinate the tire with the steering
    Final_Drive_Speed = COMMANDED_SPEED + Current_Steering_Speed * Speed_Scale_Factor / 100
    
    ' Convert angle in tenth-degrees to -1000/+1000 range
    Final_Angle = COMMANDED_ANGLE * 10 / 9

    If Final_Angle = GetValue(_F, 2) Then
        SetCommand(BRAKES_ON, STEER_BRAKE_PIN)
    Else 
        SetCommand(BRAKES_OFF, STEER_BRAKE_PIN)
        SetValue(_G, 2, Final_Angle)
    End If
    
    If Final_Drive_Speed = 0 && Current_Drive_Speed = 0 Then '[or braking commanded?]
        SetCommand(BRAKES_ON, DRIVE_BRAKE_PIN)
    Else
        SetCommand(BRAKES_OFF, DRIVE_BRAKE_PIN)
    End If
    
    If Final_Drive_Speed <> Current_Drive_Speed Then
        ' apply motor speed to drive motor
        SetValue(_G, 1, Final_Drive_Speed*100/Max_Speed)
    End If
    
    ' Copy motor status if it is time
    If GetTimerState(REPORT_TIMER) = 1 Then
        GoSub ReportParameters
        SetTimerCount(REPORT_TIMER, DATA_SEND_RATE)
    End If
    
    ' Check that the controller heartbeat has occurred
    If GetTimerState(KEEPALIVE_TIMER) = 1 Then
        If NUC_IS_ALIVE = 0 Then
            GoSub SafeUntilSignal
        End If
        NUC_IS_ALIVE = 0
        SetTimerCount(KEEPALIVE_TIMER, HEARTBEAT_RECIEVE_RATE)
    End If
    
    ' Wait until next chronon- er, cycle (is there a sync function?)
    wait(1)
GoTo loop

' Parameter-reporting subroutine
ReportParameters:
    VAR1 = 0
    VAR2 = 0
    VAR3 = 0
    VAR4 = 0
    VAR5 = 0
    VAR6 = 0
    VAR7 = 0
    VAR8 = 0
Return


' safe-until-signal subroutine
SafeUntilSignal:
    ' Clear the flag that set by the NUC's "heartbeat" RPDO
    NUC_IS_ALIVE = 0
    
    ' Stop motors
    SetValue(_G, 1, 0)
    setcommand(_MSTOP, 2)
    ' TODO: See if there's an actual "stop" command for position motors other than this psuedo-e-stop
    
    ' Wait until both are stopped before signaling brakes
    While GetConfig(_S, 2) <> 0 ' Steering speed (will probably hit 0 first)
        wait(1)
    End While
    SetCommand(BRAKES_ON, STEER_BRAKE_PIN)
    
    While GetConfig(_S, 1) <> 0 ' Driving speed
        wait(1)
    End While
    SetCommand(BRAKES_ON, DRIVE_BRAKE_PIN)
    
    ' Wait for a signal
    While NUC_IS_ALIVE = 0
        wait(1)
    End While
Return






' Dead simple CAN test
SetConfig(_CEN, 1)       ' Start CANopen
SetConfig(_CTPS, 1, 200) ' Send VAR1,2 every 200 ms

loop:
    VAR1 = VAR9 + 3
    VAR2 = VAR10 * 2
    wait(10)
GoTo loop









' Slightly more complicated CAN test
' Added defines, timer, and subroutine call

#define COMMANDED_SPEED VAR9
#define COMMANDED_ANGLE VAR10
#define DATA_SEND_RATE 200

#define REPORT_TIMER = 1

' Set up status reporting
SetConfig(_CEN, 1)                  ' Start CANopen (comes with a default 100ms heartbeat
SetConfig(_CTPS, 1, DATA_SEND_RATE) ' Send VAR1,2 every DATA_SEND_RATE ms
SetConfig(_CTPS, 1, 0) ' Do not send
SetConfig(_CTPS, 1, 0) ' Do not send
SetConfig(_CTPS, 1, 0) ' Do not send VAR7,8

SetTimerCount(REPORT_TIMER, DATA_SEND_RATE)

loop:
    ' Set report variables
    If GetTimerState(REPORT_TIMER) = 1 Then
        GoSub ReportParameters
        SetTimerCount(REPORT_TIMER, DATA_SEND_RATE)
    End If
    
    ' Wait until next cycle
    wait(1)
GoTo loop

' Parameter-reporting subroutine
ReportParameters:
    VAR1 = COMMANDED_SPEED + 3
    VAR2 = COMMANDED_ANGLE * 2
Return


