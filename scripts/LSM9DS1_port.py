#!/usr/bin/env python

# Port of Arduino code from https://github.com/sparkfun/LSM9DS1_Breakout

from smbus import SMBus
from enum import Enum


#/////////////////////////////////////////
#// LSM9DS1 Accel/Gyro (XL/G) Registers //
#/////////////////////////////////////////
ACT_THS				= 0x04
ACT_DUR				= 0x05
INT_GEN_CFG_XL		= 0x06
INT_GEN_THS_X_XL	= 0x07
INT_GEN_THS_Y_XL	= 0x08
INT_GEN_THS_Z_XL	= 0x09
INT_GEN_DUR_XL		= 0x0A
REFERENCE_G			= 0x0B
INT1_CTRL			= 0x0C
INT2_CTRL			= 0x0D
WHO_AM_I_XG			= 0x0F
CTRL_REG1_G			= 0x10
CTRL_REG2_G			= 0x11
CTRL_REG3_G			= 0x12
ORIENT_CFG_G		= 0x13
INT_GEN_SRC_G		= 0x14
OUT_TEMP_L			= 0x15
OUT_TEMP_H			= 0x16
STATUS_REG_0		= 0x17
OUT_X_L_G			= 0x18
OUT_X_H_G			= 0x19
OUT_Y_L_G			= 0x1A
OUT_Y_H_G			= 0x1B
OUT_Z_L_G			= 0x1C
OUT_Z_H_G			= 0x1D
CTRL_REG4			= 0x1E
CTRL_REG5_XL		= 0x1F
CTRL_REG6_XL		= 0x20
CTRL_REG7_XL		= 0x21
CTRL_REG8			= 0x22
CTRL_REG9			= 0x23
CTRL_REG10			= 0x24
INT_GEN_SRC_XL		= 0x26
STATUS_REG_1		= 0x27
OUT_X_L_XL			= 0x28
OUT_X_H_XL			= 0x29
OUT_Y_L_XL			= 0x2A
OUT_Y_H_XL			= 0x2B
OUT_Z_L_XL			= 0x2C
OUT_Z_H_XL			= 0x2D
FIFO_CTRL			= 0x2E
FIFO_SRC			= 0x2F
INT_GEN_CFG_G		= 0x30
INT_GEN_THS_XH_G	= 0x31
INT_GEN_THS_XL_G	= 0x32
INT_GEN_THS_YH_G	= 0x33
INT_GEN_THS_YL_G	= 0x34
INT_GEN_THS_ZH_G	= 0x35
INT_GEN_THS_ZL_G	= 0x36
INT_GEN_DUR_G		= 0x37

#///////////////////////////////
#// LSM9DS1 Magneto Registers //
#///////////////////////////////
OFFSET_X_REG_L_M	= 0x05
OFFSET_X_REG_H_M	= 0x06
OFFSET_Y_REG_L_M	= 0x07
OFFSET_Y_REG_H_M	= 0x08
OFFSET_Z_REG_L_M	= 0x09
OFFSET_Z_REG_H_M	= 0x0A
WHO_AM_I_M			= 0x0F
CTRL_REG1_M			= 0x20
CTRL_REG2_M			= 0x21
CTRL_REG3_M			= 0x22
CTRL_REG4_M			= 0x23
CTRL_REG5_M			= 0x24
STATUS_REG_M		= 0x27
OUT_X_L_M			= 0x28
OUT_X_H_M			= 0x29
OUT_Y_L_M			= 0x2A
OUT_Y_H_M			= 0x2B
OUT_Z_L_M			= 0x2C
OUT_Z_H_M			= 0x2D
INT_CFG_M			= 0x30
INT_SRC_M			= 0x30
INT_THS_L_M			= 0x32
INT_THS_H_M			= 0x33

#////////////////////////////////
#// LSM9DS1 WHO_AM_I Responses //
#////////////////////////////////
WHO_AM_I_AG_RSP		= 0x68
WHO_AM_I_M_RSP		= 0x3D









# The LSM9DS1 functions over both I2C or SPI. This library supports both.
# But the interface mode used must be sent to the LSM9DS1 constructor. Use
# one of these two as the first parameter of the constructor.
IMU_MODE_SPI = 0
IMU_MODE_I2C = 1


# accel_scale defines all possible FSR's of the accelerometer:
class accel_scale(Enum):
    A_SCALE_2G  = 0  # 00:  2g
    A_SCALE_16G = 1  # 01:  16g
    A_SCALE_4G  = 2  # 10:  4g
    A_SCALE_8G  = 3  # 11:  8g
#

# gyro_scale defines the possible full-scale ranges of the gyroscope:
class gyro_scale(Enum):
    G_SCALE_245DPS  = 0  # 00:  245 degrees per second
    G_SCALE_500DPS  = 1  # 01:  500 dps
    G_SCALE_2000DPS = 2  # 11:  2000 dps # FIXME: Is this 0x2 or 0x3?
#

# mag_scale defines all possible FSR's of the magnetometer:
class mag_scale(Enum):
    M_SCALE_4GS  = 0    # 00:  4Gs
    M_SCALE_8GS  = 1    # 01:  8Gs
    M_SCALE_12GS = 2    # 10:  12Gs
    M_SCALE_16GS = 3    # 11:  16Gs
#

# gyro_odr defines all possible data rate/bandwidth combos of the gyro:
class gyro_odr(Enum):
    #! TODO 
    G_ODR_PD  = 0   # Power down (0)
    G_ODR_149 = 1   # 14.9 Hz (1)
    G_ODR_595 = 2   # 59.5 Hz (2)
    G_ODR_119 = 3   # 119 Hz (3)
    G_ODR_238 = 4   # 238 Hz (4)
    G_ODR_476 = 5   # 476 Hz (5)
    G_ODR_952 = 6   # 952 Hz (6)
#

# accel_oder defines all possible output data rates of the accelerometer:
class accel_odr(Enum):
    XL_POWER_DOWN = 0   # Power-down mode (0x0)
    XL_ODR_10     = 1   # 10 Hz (0x1)
    XL_ODR_50     = 2   # 50 Hz (0x02)
    XL_ODR_119    = 3   # 119 Hz (0x3)
    XL_ODR_238    = 4   # 238 Hz (0x4)
    XL_ODR_476    = 5   # 476 Hz (0x5)
    XL_ODR_952    = 6   # 952 Hz (0x6)
#

# accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
class accel_abw(Enum):
    A_ABW_408 = 0   # 408 Hz (0x0)
    A_ABW_211 = 1   # 211 Hz (0x1)
    A_ABW_105 = 2   # 105 Hz (0x2)
    A_ABW_50  = 3   #  50 Hz (0x3)
#


# mag_odr defines all possible output data rates of the magnetometer:
class mag_odr(Enum):
    M_ODR_0625 = 0  # 0.625 Hz (0)
    M_ODR_125  = 1  # 1.25 Hz (1)
    M_ODR_250  = 2  # 2.5 Hz (2)
    M_ODR_5    = 3  # 5 Hz (3)
    M_ODR_10   = 4  # 10 Hz (4)
    M_ODR_20   = 5  # 20 Hz (5)
    M_ODR_40   = 6  # 40 Hz (6)
    M_ODR_80   = 7  # 80 Hz (7)
#

class interrupt_select(Enum):
    XG_INT1 = INT1_CTRL = 0
    XG_INT2 = INT2_CTRL = 1
#

class interrupt_generators(Enum):
    INT_DRDY_XL    = (1<<0)  # Accelerometer data ready (INT1 & INT2)
    INT_DRDY_G     = (1<<1)  # Gyroscope data ready (INT1 & INT2)
    INT1_BOOT      = (1<<2)  # Boot status (INT1)
    INT2_DRDY_TEMP = (1<<2)  # Temp data ready (INT2)
    INT_FTH        = (1<<3)  # FIFO threshold interrupt (INT1 & INT2)
    INT_OVR        = (1<<4)  # Overrun interrupt (INT1 & INT2)
    INT_FSS5       = (1<<5)  # FSS5 interrupt (INT1 & INT2)
    INT_IG_XL      = (1<<6)  # Accel interrupt generator (INT1)
    INT1_IG_G      = (1<<7)  # Gyro interrupt enable (INT1)
    INT2_INACT     = (1<<7)  # Inactivity interrupt output (INT2)
#

class accel_interrupt_generator(Enum):
    XLIE_XL = (1<<0)
    XHIE_XL = (1<<1)
    YLIE_XL = (1<<2)
    YHIE_XL = (1<<3)
    ZLIE_XL = (1<<4)
    ZHIE_XL = (1<<5)
    GEN_6D = (1<<6)
#

class gyro_interrupt_generator(Enum):
    XLIE_G = (1<<0)
    XHIE_G = (1<<1)
    YLIE_G = (1<<2)
    YHIE_G = (1<<3)
    ZLIE_G = (1<<4)
    ZHIE_G = (1<<5)
#

class mag_interrupt_generator(Enum):
    ZIEN = (1<<5)
    YIEN = (1<<6)
    XIEN = (1<<7)
#

class h_lactive(Enum):
    INT_ACTIVE_HIGH = 0
    INT_ACTIVE_LOW  = 1
#

class pp_od(Enum):
    INT_PUSH_PULL  = 0
    INT_OPEN_DRAIN = 1
#

#class fifoMode_type(Enum):
FIFO_OFF          = 0
FIFO_THS          = 1
FIFO_CONT_TRIGGER = 3
FIFO_OFF_TRIGGER  = 4
FIFO_CONT         = 5
#




















class GyroSettingsStruct:
    def __init__(self):
        # Gyroscope settings:
        self.enabled = 0
        self.scale = 0
        self.sampleRate = 0
        # New gyro stuff:
        self.bandwidth = 0
        self.lowPowerEnable = 0
        self.HPFEnable = 0
        self.HPFCutoff = 0
        self.flipX = 0
        self.flipY = 0
        self.flipZ = 0
        self.orientation = 0
        self.enableX = 0
        self.enableY = 0
        self.enableZ = 0
        self.latchInterrupt = 0
#

class DeviceSettingsStruct:
    def __init__(self):
        self.commInterface = 0  # Can be I2C, SPI 4-wire or SPI 3-wire
        self.agAddress     = 0  # I2C address or SPI CS pin
        self.mAddress      = 0  # I2C address or SPI CS pin
#

class AccelSettingsStruct:
    def __init__(self):
    # Accelerometer settings:
        self.enabled = 0
        self.scale = 0
        self.sampleRate = 0
    # New accel stuff:
        self.enableX = 0
        self.enableY = 0
        self.enableZ = 0
        self.bandwidth = 0
        self.highResEnable = 0
        self.highResBandwidth = 0
#

class MagSettingsStruct:
    def __init__(self):
    # Magnetometer settings:
        self.enabled = 0
        self.scale = 0
        self.sampleRate = 0
    # New mag stuff:
        self.tempCompensationEnable = 0
        self.XYPerformance = 0
        self.ZPerformance = 0
        self.lowPowerEnable = 0
        self.operatingMode = 0
#

class TemperatureSettingsStruct:
    def __init__(self):
        # Temperature settings
        self.enabled = False
#



def LSM9DS1_AG_ADDR(sa0):
    if sa0 == 0:
        return 0x6A 
    else: 
        return 0x6B
def LSM9DS1_M_ADDR(sa1):
    if sa1 == 0:
        return 0x1C 
    else: 
        return 0x1E
#

class lsm9ds1_axis(Enum):
    X_AXIS = 0
    Y_AXIS = 1
    Z_AXIS = 2
    ALL_AXIS = 3
#

X_AXIS = 0
Y_AXIS = 1
Z_AXIS = 2
ALL_AXIS = 3

LSM9DS1_COMMUNICATION_TIMEOUT = 1000
magSensitivity = [0.00014, 0.00029, 0.00043, 0.00058]

class LSM9DS1:
    
    # LSM9DS1 -- LSM9DS1 class constructor
    # The constructor will set up a handful of private variables, and set the
    # communication mode as well.
    # Input:
    #	- interface = Either IMU_MODE_SPI or IMU_MODE_I2C, whichever you're using
    #				to talk to the IC.
    #	- xgAddr = If IMU_MODE_I2C, this is the I2C address of the accel/gyroscope.
    # 				If IMU_MODE_SPI, this is the chip select pin of the gyro (CS_AG)
    #	- mAddr = If IMU_MODE_I2C, this is the I2C address of the magnetometer.
    #				If IMU_MODE_SPI, this is the cs pin of the magnetometer (CS_M)
#    def __init__(self):
#        self.init(IMU_MODE_I2C, LSM9DS1_AG_ADDR(1), LSM9DS1_M_ADDR(1))    
    def __init__(self, interface = IMU_MODE_I2C, xgAddr = 0x6B, mAddr = 0x1E):
        self.init(interface, xgAddr, mAddr)
    #
    
    
    # init() -- Sets up gyro, accel, and mag settings to default.
    # - interface - Sets the interface mode (IMU_MODE_I2C or IMU_MODE_SPI)
    # - xgAddr - Sets either the I2C address of the accel/gyro or SPI chip 
    #   select pin connected to the CS_XG pin.
    # - mAddr - Sets either the I2C address of the magnetometer or SPI chip 
    #   select pin connected to the CS_M pin.
    def init(self, interface, xgAddr, mAddr):
        self.device = DeviceSettingsStruct()
        self.gyro   = GyroSettingsStruct()
        self.accel  = AccelSettingsStruct()
        self.mag    = MagSettingsStruct()
        self.temp   = TemperatureSettingsStruct()
        
        self.device.commInterface = interface;
        self.device.agAddress = xgAddr;
        self.device.mAddress = mAddr;
        
        self.gyro.enabled = True;
        self.gyro.enableX = True;
        self.gyro.enableY = True;
        self.gyro.enableZ = True;
        # gyro scale can be 245, 500, or 2000
        self.gyro.scale = 245;
        # gyro sample rate: value between 1-6
        # 1 = 14.9    4 = 238
        # 2 = 59.5    5 = 476
        # 3 = 119     6 = 952
        self.gyro.sampleRate = 6;
        # gyro cutoff frequency: value between 0-3
        # Actual value of cutoff frequency depends
        # on sample rate.
        self.gyro.bandwidth = 0;
        self.gyro.lowPowerEnable = False;
        self.gyro.HPFEnable = False;
        # Gyro HPF cutoff frequency: value between 0-9
        # Actual value depends on sample rate. Only applies
        # if gyroHPFEnable is true.
        self.gyro.HPFCutoff = 0;
        self.gyro.flipX = False;
        self.gyro.flipY = False;
        self.gyro.flipZ = False;
        self.gyro.orientation = 0;
        self.gyro.latchInterrupt = True;
        
        self.accel.enabled = True;
        self.accel.enableX = True;
        self.accel.enableY = True;
        self.accel.enableZ = True;
        # accel scale can be 2, 4, 8, or 16
        self.accel.scale = 2;
        # accel sample rate can be 1-6
        # 1 = 10 Hz    4 = 238 Hz
        # 2 = 50 Hz    5 = 476 Hz
        # 3 = 119 Hz   6 = 952 Hz
        self.accel.sampleRate = 6;
        # Accel cutoff freqeuncy can be any value between -1 - 3. 
        # -1 = bandwidth determined by sample rate
        # 0 = 408 Hz   2 = 105 Hz
        # 1 = 211 Hz   3 = 50 Hz
        self.accel.bandwidth = -1;
        self.accel.highResEnable = False;
        # accelHighResBandwidth can be any value between 0-3
        # LP cutoff is set to a factor of sample rate
        # 0 = ODR/50    2 = ODR/9
        # 1 = ODR/100   3 = ODR/400
        self.accel.highResBandwidth = 0;
        
        self.mag.enabled = True;
        # mag scale can be 4, 8, 12, or 16
        self.mag.scale = 4;
        # mag data rate can be 0-7
        # 0 = 0.625 Hz  4 = 10 Hz
        # 1 = 1.25 Hz   5 = 20 Hz
        # 2 = 2.5 Hz    6 = 40 Hz
        # 3 = 5 Hz      7 = 80 Hz
        self.mag.sampleRate = 7;
        self.mag.tempCompensationEnable = False;
        # magPerformance can be any value between 0-3
        # 0 = Low power mode      2 = high performance
        # 1 = medium performance  3 = ultra-high performance
        self.mag.XYPerformance = 3;
        self.mag.ZPerformance = 3;
        self.mag.lowPowerEnable = False;
        # magOperatingMode can be 0-2
        # 0 = continuous conversion
        # 1 = single-conversion
        # 2 = power down
        self.mag.operatingMode = 0;
        
        self.temp.enabled = True;
        
        self.gBias = [0,0,0]
        self.aBias = [0,0,0]
        self.mBias = [0,0,0]
        self.gBiasRaw = [0,0,0]
        self.aBiasRaw = [0,0,0]
        self.mBiasRaw = [0,0,0]
        
        # These values are the RAW signed 16-bit readings from the sensors.
        self.gx = 0 # x, y, and z axis readings of the gyroscope
        self.gy = 0
        self.gz = 0
        self.ax = 0 # x, y, and z axis readings of the accelerometer
        self.ay = 0
        self.az = 0
        self.mx = 0 # x, y, and z axis readings of the magnetometer
        self.my = 0
        self.mz = 0
        self.temperature = 0 # Chip temperature
        
        # _autoCalc keeps track of whether we're automatically subtracting off
        # accelerometer and gyroscope bias calculated in calibrate().
        self._autoCalc = False;
        
        # gRes, aRes, and mRes store the current resolution for each sensor. 
        # Units of these values would be DPS (or g's or Gs's) per ADC tick.
        # This value is calculated as (sensor scale) / (2^15).
    #    float gRes, aRes, mRes;
    #
    
    
    
    
    

    	
    # begin() -- Initialize the gyro, accelerometer, and magnetometer.
    # This will set up the scale and output rate of each sensor. The values set
    # in the IMUSettings struct will take effect after calling this function.
    def begin(self):
        
        self.constrainScales()
        # Once we have the scale values, we can calculate the resolution
        # of each sensor. That's what these functions are for. One for each sensor
        self.calcgRes() # Calculate DPS / ADC tick, stored in gRes variable
        self.calcmRes() # Calculate Gs / ADC tick, stored in mRes variable
        self.calcaRes() # Calculate g / ADC tick, stored in aRes variable
    
        # Now, initialize our hardware interface.
        if self.device.commInterface == IMU_MODE_I2C:	# If we're using I2C
            self.initI2C();	# Initialize I2C
        #elif (self.device.commInterface == IMU_MODE_SPI): 	# else, if we're using SPI
        #    self.initSPI();	# Initialize SPI
        
        # To verify communication, we can read from the WHO_AM_I register of
        # each device. Store those in a variable so we can return them.
        mTest = self.mReadByte(WHO_AM_I_M)		# Read the gyro WHO_AM_I
        xgTest = self.xgReadByte(WHO_AM_I_XG)	# Read the accel/mag WHO_AM_I
        whoAmICombined = (xgTest << 8) | mTest
    
        if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP)):
            return 0
    
        # Gyro initialization stuff:
        self.initGyro()     # This will "turn on" the gyro. Setting up interrupts, etc.
        self.initAccel()    # "Turn on" all axes of the accel. Set up interrupts, etc.
        self.initMag()      # "Turn on" all axes of the mag. Set up interrupts, etc.
        
        # Once everything is initialized, return the WHO_AM_I registers we read:
        return whoAmICombined;
    #
    
    # This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
    # them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
    # for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
    # the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
    # subtract the biases ourselves. This results in a more accurate measurement in general and can
    # remove errors due to imprecise or varying initial placement. Calibration of sensor data in this
    # manner is good practice.
    def calibrate(self, autoCalc = True):
        data = [0, 0, 0, 0, 0, 0]
        samples = 0;
        aBiasRawTemp = [0, 0, 0]
        gBiasRawTemp = [0, 0, 0]
    
        # Turn on FIFO and set threshold to 32 samples
        self.enableFIFO(True);
        self.setFIFO(FIFO_THS, 0x1F);
        while (samples < 0x1F):
            samples = (self.xgReadByte(FIFO_SRC) & 0x3F); # Read number of stored samples
        #
        for ii in range(samples): # (ii = 0; ii < samples ; ii++) 
            # Read the gyro data stored in the FIFO
            self.readGyro()
            gBiasRawTemp[0] += self.gx
            gBiasRawTemp[1] += self.gy
            gBiasRawTemp[2] += self.gz
            self.readAccel()
            aBiasRawTemp[0] += self.ax
            aBiasRawTemp[1] += self.ay
            aBiasRawTemp[2] += self.az #  - (1./self.aRes) # Assumes sensor facing up!
        #
        
        for ii in range(3): # (ii = 0; ii < 3; ii++)
            self.gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
            self.gBias[ii] = self.calcGyro(gBiasRaw[ii]);
            self.aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
            self.aBias[ii] = self.calcAccel(aBiasRaw[ii]);
        #
        
        self.enableFIFO(False);
        self.setFIFO(FIFO_OFF, 0x00);
        
        if (autoCalc): self._autoCalc = True;
    #
    
    def calibrateMag(self, loadIn = True):
        magMin = [0, 0, 0]
        magMax = [0, 0, 0] # The road warrior
    
        for i in range(128):
            while not self.magAvailable(): pass
            magTemp = self.readMag()
            for j in range(3):
                if (magTemp[j] > magMax[j]): magMax[j] = magTemp[j];
                if (magTemp[j] < magMin[j]): magMin[j] = magTemp[j];
            #
        #
        for j in range(3):
            self.mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
            self.mBias[j] = self.calcMag(self.mBiasRaw[j]);
            if (loadIn): self.magOffset(j, self.mBiasRaw[j]);
        #
    #
    def magOffset(self, axis, offset):
        if (axis > 2): return;
        msb = (offset & 0xFF00) >> 8;
        lsb = offset & 0x00FF;
        self.mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
        self.mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
    #
    
    # accelAvailable() -- Polls the accelerometer status register to check
    # if new data is available.
    # Output:	1 - New data available
    #			0 - No new data available
    def accelAvailable(self):
        status = self.xgReadByte(STATUS_REG_1);
        return (status & (1<<0));
    #
    
    # gyroAvailable() -- Polls the gyroscope status register to check
    # if new data is available.
    # Output:	1 - New data available
    #			0 - No new data available
    def gyroAvailable(self):
        status = self.xgReadByte(STATUS_REG_1);
        return ((status & (1<<1)) >> 1);
    #
    
    # gyroAvailable() -- Polls the temperature status register to check
    # if new data is available.
    # Output:	1 - New data available
    #			0 - No new data available
    def tempAvailable(self):
        status = self.xgReadByte(STATUS_REG_1);
        return ((status & (1<<2)) >> 2);
    #
    
    # magAvailable() -- Polls the accelerometer status register to check
    # if new data is available.
    # Input:
    #	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
    #	  on one specific axis. Or ALL_AXIS (default) to check for new data
    #	  on all axes.
    # Output:	1 - New data available
    #			0 - No new data available
    def magAvailable(self, axis = lsm9ds1_axis.ALL_AXIS):
        status = self.mReadByte(STATUS_REG_M);
        return ((status & (1<<axis.value)) >> axis.value);
    #
    
    # readGyro() -- Read the gyroscope output registers.
    # This function will read all six gyroscope output registers.
    # The readings are stored in the class' gx, gy, and gz variables. Read
    # those _after_ calling readGyro().
    def readGyro(self):
        temp = self.xgReadBytes(OUT_X_L_G, 6); # Read 6 bytes, beginning at OUT_X_L_G
        self.gx = (temp[1] << 8) | temp[0]; # Store x-axis values into gx
        self.gy = (temp[3] << 8) | temp[2]; # Store y-axis values into gy
        self.gz = (temp[5] << 8) | temp[4]; # Store z-axis values into gz
        if (self._autoCalc):
            self.gx -= self.gBiasRaw[X_AXIS];
            self.gy -= self.gBiasRaw[Y_AXIS];
            self.gz -= self.gBiasRaw[Z_AXIS];
        return (self.gx, self.gy, self.gz)
    #
    
    # int16_t readGyro(axis) -- Read a specific axis of the gyroscope.
    # [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
    # Input:
    #	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
    # Output:
    #	A 16-bit signed integer with sensor data on requested axis.
    def readGyroAxis(self, axis):
        temp = self.xgReadBytes(OUT_X_L_G + (2 * axis.value), 2);
        value = (temp[1] << 8) | temp[0];
        if (self._autoCalc):
            value -= self.gBiasRaw[axis.value]
        return value
    #
    
    # readAccel() -- Read the accelerometer output registers.
    # This function will read all six accelerometer output registers.
    # The readings are stored in the class' ax, ay, and az variables. Read
    # those _after_ calling readAccel().
    def readAccel(self):
        temp = self.xgReadBytes(OUT_X_L_XL, 6); # Read 6 bytes, beginning at OUT_X_L_XL
        self.ax = (temp[1] << 8) | temp[0]; # Store x-axis values into ax
        self.ay = (temp[3] << 8) | temp[2]; # Store y-axis values into ay
        self.az = (temp[5] << 8) | temp[4]; # Store z-axis values into az
        if (self._autoCalc):
            self.ax -= self.aBiasRaw[X_AXIS];
            self.ay -= self.aBiasRaw[Y_AXIS];
            self.az -= self.aBiasRaw[Z_AXIS];
        return (self.ax, self.ay, self.az)
    #
    
    # int16_t readAccel(axis) -- Read a specific axis of the accelerometer.
    # [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
    # Input:
    #	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
    # Output:
    #	A 16-bit signed integer with sensor data on requested axis.
    def readAccelAxis(self, axis):
        temp = self.xgReadBytes(OUT_X_L_XL + (2 * axis.value), 2);
        value = (temp[1] << 8) | temp[0];
        
        if (self._autoCalc): value -= self.aBiasRaw[axis.value];
        return value;
    #
    
    # readMag() -- Read the magnetometer output registers.
    # This function will read all six magnetometer output registers.
    # The readings are stored in the class' mx, my, and mz variables. Read
    # those _after_ calling readMag().
    def readMag(self):
        temp = self.mReadBytes(OUT_X_L_M, 6); # Read 6 bytes, beginning at OUT_X_L_M
        self.mx = (temp[1] << 8) | temp[0]; # Store x-axis values into mx
        self.my = (temp[3] << 8) | temp[2]; # Store y-axis values into my
        self.mz = (temp[5] << 8) | temp[4]; # Store z-axis values into mz
        return (self.mx, self.my, self.mz)
    #
    
    
    # int16_t readMag(axis) -- Read a specific axis of the magnetometer.
    # [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
    # Input:
    #	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
    # Output:
    #	A 16-bit signed integer with sensor data on requested axis.
    def readMagAxis(self, axis):
        temp = self.mReadBytes(OUT_X_L_M + (2 * axis.value), 2);
        return (temp[1] << 8) | temp[0];
    #
    
    # readTemp() -- Read the temperature output register.
    # This function will read two temperature output registers.
    # The combined readings are stored in the class' temperature variables. Read
    # those _after_ calling readTemp().
    def readTemp(self):
        temp = self.xgReadBytes(OUT_TEMP_L, 2); # Read 2 bytes, beginning at OUT_TEMP_L
        self.temperature = (temp[1] << 8) | temp[0]
        return self.temperature
    #
    
    # calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
    # This function reads in a signed 16-bit value and returns the scaled
    # DPS. This function relies on gScale and gRes being correct.
    # Input:
    #	- gyro = A signed 16-bit raw reading from the gyroscope.
    def calcGyro(self, gyro):
        # Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
        return self.gRes * gyro
    
    def readGyroCal(self):
        # Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
        g = self.readGyro()
        return (g[0]*self.gRes, g[1]*self.gRes, g[2]*self.gRes)
    
    # calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
    # This function reads in a signed 16-bit value and returns the scaled
    # g's. This function relies on aScale and aRes being correct.
    # Input:
    #	- accel = A signed 16-bit raw reading from the accelerometer.
    def calcAccel(self, accel):
        # Return the accel raw reading times our pre-calculated g's / (ADC tick):
        return self.aRes * accel
    
    def readAccelCal(self):
        # Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
        a = self.readAccel()
        return (a[0]*self.aRes, a[1]*self.aRes, a[2]*self.aRes)
    #
    
    # calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
    # This function reads in a signed 16-bit value and returns the scaled
    # Gs. This function relies on mScale and mRes being correct.
    # Input:
    #	- mag = A signed 16-bit raw reading from the magnetometer.
    def calcMag(self, mag):
        # Return the mag raw reading times our pre-calculated Gs / (ADC tick):
        return self.mRes * mag
    
    def readMagCal(self):
        # Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
        m = self.readMag()
        return (m[0]*self.mRes, m[1]*self.mRes, m[2]*self.mRes)
    #
    
    # setGyroScale() -- Set the full-scale range of the gyroscope.
    # This function can be called to set the scale of the gyroscope to 
    # 245, 500, or 200 degrees per second.
    # Input:
    # 	- gScl = The desired gyroscope scale. Must be one of three possible
    #		values from the gyro_scale.
    def setGyroScale(self, gScl):
        # Read current value of CTRL_REG1_G:
        ctrl1RegValue = self.xgReadByte(CTRL_REG1_G);
        # Mask out scale bits (3 & 4):
        ctrl1RegValue &= 0xE7;
        if gScl ==  500:
            ctrl1RegValue |= (0x1 << 3);
            self.gyro.scale = 500;
        elif gScl ==  2000:
            ctrl1RegValue |= (0x3 << 3);
            self.gyro.scale = 2000;
        else: # Otherwise we'll set it to 245 dps (0x0 << 4)
            self.gyro.scale = 245;
        #
        self.xgWriteByte(CTRL_REG1_G, ctrl1RegValue);
        self.calcgRes()
    #
    
    # setAccelScale() -- Set the full-scale range of the accelerometer.
    # This function can be called to set the scale of the accelerometer to
    # 2, 4, 6, 8, or 16 g's.
    # Input:
    # 	- aScl = The desired accelerometer scale. Must be one of five possible
    #		values from the accel_scale.
    def setAccelScale(self, aScl):
        # We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
        tempRegValue = self.xgReadByte(CTRL_REG6_XL);
        # Mask out accel scale bits:
        tempRegValue &= 0xE7;
        
        if aScl ==  4:
            tempRegValue |= (0x2 << 3);
            self.accel.scale = 4;
        elif aScl ==  8:
            tempRegValue |= (0x3 << 3);
            self.accel.scale = 8;
        elif aScl ==  16:
            tempRegValue |= (0x1 << 3);
            self.accel.scale = 16;
        else: # Otherwise it'll be set to 2g (0x0 << 3)
            self.accel.scale = 2;
        #
        self.xgWriteByte(CTRL_REG6_XL, tempRegValue);
        
        # Then calculate a new aRes, which relies on aScale being set correctly:
        self.calcaRes();
    #
    
    # setMagScale() -- Set the full-scale range of the magnetometer.
    # This function can be called to set the scale of the magnetometer to
    # 2, 4, 8, or 12 Gs.
    # Input:
    # 	- mScl = The desired magnetometer scale. Must be one of four possible
    #		values from the mag_scale.
    def setMagScale(self, mScl):
        # We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
        temp = self.mReadByte(CTRL_REG2_M)
        # Then mask out the mag scale bits:
        temp &= 0xFF^(0x3 << 5);
        
        if mScl ==  8:
            temp |= (0x1 << 5);
            self.mag.scale = 8;
        elif mScl ==  12:
            temp |= (0x2 << 5);
            self.mag.scale = 12;
        elif mScl ==  16:
            temp |= (0x3 << 5);
            self.mag.scale = 16;
        else: # Otherwise we'll default to 4 gauss (00)
            self.mag.scale = 4;
        #
        
        # And write the new register value back into CTRL_REG6_XM:
        self.mWriteByte(CTRL_REG2_M, temp);
        
        # We've updated the sensor, but we also need to update our class variables
        # First update mScale:
        #mScale = mScl;
        # Then calculate a new mRes, which relies on mScale being set correctly:
        self.calcmRes();
    #
    
    # setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
    # Input:
    #	- gRate = The desired output rate and cutoff frequency of the gyro.
    def setGyroODR(self, gRate):
        # Only do this if gRate is not 0 (which would disable the gyro)
        if ((gRate & 0x07) == 0): return
        
        # We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
        temp = self.xgReadByte(CTRL_REG1_G);
        # Then mask out the gyro ODR bits:
        temp &= 0xFF^(0x7 << 5);
        temp |= (gRate & 0x07) << 5;
        # Update our settings struct
        self.gyro.sampleRate = gRate & 0x07;
        # And write the new register value back into CTRL_REG1_G:
        self.xgWriteByte(CTRL_REG1_G, temp);
    #
    
    # setAccelODR() -- Set the output data rate of the accelerometer
    # Input:
    #	- aRate = The desired output rate of the accel.
    def setAccelODR(self, aRate):
        # Only do this if aRate is not 0 (which would disable the accel)
        if ((aRate & 0x07) == 0): return
        
        # We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
        temp = self.xgReadByte(CTRL_REG6_XL);
        # Then mask out the accel ODR bits:
        temp &= 0x1F;
        # Then shift in our new ODR bits:
        temp |= ((aRate & 0x07) << 5);
        self.accel.sampleRate = aRate & 0x07;
        # And write the new register value back into CTRL_REG1_XM:
        self.xgWriteByte(CTRL_REG6_XL, temp);
    #
    
    # setMagODR() -- Set the output data rate of the magnetometer
    # Input:
    #	- mRate = The desired output rate of the mag.
    def setMagODR(self, mRate):
        # We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
        temp = self.mReadByte(CTRL_REG1_M);
        # Then mask out the mag ODR bits:
        temp &= 0xFF^(0x7 << 2);
        # Then shift in our new ODR bits:
        temp |= ((mRate & 0x07) << 2);
        self.mag.sampleRate = mRate & 0x07;
        # And write the new register value back into CTRL_REG5_XM:
        self.mWriteByte(CTRL_REG1_M, temp);
    #
    
    # configInactivity() -- Configure inactivity interrupt parameters
    # Input:
    #	- duration = Inactivity duration - actual value depends on gyro ODR
    #	- threshold = Activity Threshold
    #	- sleepOn = Gyroscope operating mode during inactivity.
    #	  true: gyroscope in sleep mode
    #	  false: gyroscope in power-down
    def configInactivity(self, duration, threshold, sleepOn):
        temp = threshold & 0x7F;
        if (sleepOn): temp |= (1<<7);
        self.xgWriteByte(ACT_THS, temp);
        self.xgWriteByte(ACT_DUR, duration);
    #
    
    # configAccelInt() -- Configure Accelerometer Interrupt Generator
    # Input:
    #	- generator = Interrupt axis/high-low events
    #	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
    #	- andInterrupts = AND/OR combination of interrupt events
    #	  true: AND combination
    #	  false: OR combination
    def configAccelInt(self, generator, andInterrupts = False):
        # Use variables from accel_interrupt_generator, OR'd together to create
        # the [generator]value.
        if (andInterrupts): generator |= 0x80;
        self.xgWriteByte(INT_GEN_CFG_XL, generator);
    #
    
    # configAccelThs() -- Configure the threshold of an accelereomter axis
    # Input:
    #	- threshold = Interrupt threshold. Possible values: 0-255.
    #	  Multiply by 128 to get the actual raw accel value.
    #	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
    #	- duration = Duration value must be above or below threshold to trigger interrupt
    #	- wait = Wait function on duration counter
    #	  true: Wait for duration samples before exiting interrupt
    #	  false: Wait function off
    def configAccelThs(self, threshold, axis, duration = 0, wait = False):
        # Write threshold value to INT_GEN_THS_?_XL.
        # axis will be 0, 1, or 2 (x, y, z respectively)
        self.xgWriteByte(INT_GEN_THS_X_XL + axis.value, threshold);
        
        # Write duration and wait to INT_GEN_DUR_XL
        temp = (duration & 0x7F);
        if (wait):temp |= 0x80;
        self.xgWriteByte(INT_GEN_DUR_XL, temp);
    #
    
    # configGyroInt() -- Configure Gyroscope Interrupt Generator
    # Input:
    #	- generator = Interrupt axis/high-low events
    #	  Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
    #	- aoi = AND/OR combination of interrupt events
    #	  true: AND combination
    #	  false: OR combination
    #	- latch: latch gyroscope interrupt request.
    def configGyroInt(self, generator, aoi, latch):
        # Use variables from accel_interrupt_generator, OR'd together to create
        # the [generator] value.
        if (aoi): generator |= 0x80;
        if (latch): generator |= 0x40;
        self.xgWriteByte(INT_GEN_CFG_G, generator);
    #
    
    # configGyroThs() -- Configure the threshold of a gyroscope axis
    # Input:
    #	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
    #	  Value is equivalent to raw gyroscope value.
    #	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
    #	- duration = Duration value must be above or below threshold to trigger interrupt
    #	- wait = Wait function on duration counter
    #	  true: Wait for duration samples before exiting interrupt
    #	  false: Wait function off
    def configGyroThs(self, threshold, axis, duration, wait):
        buffer = [(threshold & 0x7F00) >> 8, (threshold & 0x00FF)]
        # Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
        # axis will be 0, 1, or 2 (x, y, z respectively)
        self.xgWriteByte(INT_GEN_THS_XH_G + (axis.value * 2), buffer[0]);
        self.xgWriteByte(INT_GEN_THS_XH_G + 1 + (axis.value * 2), buffer[1]);
    
        # Write duration and wait to INT_GEN_DUR_XL
        temp = (duration & 0x7F);
        if (wait): temp |= 0x80;
        self.xgWriteByte(INT_GEN_DUR_G, temp);
    #
    
    # configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
    # Input:
    #	- interrupt = Select INT1 or INT2
    #	  Possible values: XG_INT1 or XG_INT2
    #	- generator = Or'd combination of interrupt generators.
    #	  Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
    #	  INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
    #	- activeLow = Interrupt active configuration
    #	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
    #	- pushPull =  Push-pull or open drain interrupt configuration
    #	  Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
    def configInt(self, interupt, generator, activeLow = h_lactive.INT_ACTIVE_LOW, pushPull = pp_od.INT_PUSH_PULL):
        # Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
        # those two values.
        # [generator] should be an OR'd list of values from the interrupt_generators enum
        self.xgWriteByte(interrupt.value, generator);
        
        # Configure CTRL_REG8
        temp = self.xgReadByte(CTRL_REG8);
        
        if (activeLow.value): temp |= (1<<5);
        else: temp &= ~(1<<5);
        
        if (pushPull.value): temp &= ~(1<<4);
        else: temp |= (1<<4);
        
        self.xgWriteByte(CTRL_REG8, temp);
    #
    
    
    # configMagInt() -- Configure Magnetometer Interrupt Generator
    # Input:
    #	- generator = Interrupt axis/high-low events
    #	  Any OR'd combination of ZIEN, YIEN, XIEN
    #	- activeLow = Interrupt active configuration
    #	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
    #	- latch: latch gyroscope interrupt request.
    def configMagInt(self, generator, activeLow, latch = True):
        # Mask out non-generator bits (0-4)
        config = (generator & 0xE0)
        # IEA bit is 0 for active-low, 1 for active-high.
        if (activeLow == h_lactive.INT_ACTIVE_HIGH): config |= (1<<2)
        # IEL bit is 0 for latched, 1 for not-latched
        if not latch: config |= (1<<1)
        # As long as we have at least 1 generator, enable the interrupt
        if (generator != 0): config |= (1<<0)
        
        self.mWriteByte(INT_CFG_M, config)
    #
    
    # configMagThs() -- Configure the threshold of a gyroscope axis
    # Input:
    #	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
    #	  Value is equivalent to raw magnetometer value.
    def configMagThs(self, threshold):
        # Write high eight bits of [threshold] to INT_THS_H_M
        self.mWriteByte(INT_THS_H_M, ((threshold & 0x7F00) >> 8));
        # Write low eight bits of [threshold] to INT_THS_L_M
        self.mWriteByte(INT_THS_L_M, (threshold & 0x00FF));
    #
    
    # getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
    def getGyroIntSrc(self):
        intSrc = self.xgReadByte(INT_GEN_SRC_G);
        # Check if the IA_G (interrupt active) bit is set
        if intSrc & (1<<6):
            return (intSrc & 0x3F);
        return 0;
    #
    
    # getGyroIntSrc() -- Get contents of accelerometer interrupt source register
    def getAccelIntSrc(self):
        intSrc = self.xgReadByte(INT_GEN_SRC_XL);
        # Check if the IA_XL (interrupt active) bit is set
        if (intSrc & (1<<6)):
            return (intSrc & 0x3F)
        return 0;
    #
    
    # getGyroIntSrc() -- Get contents of magnetometer interrupt source register
    def getMagIntSrc(self):
        intSrc = self.mReadByte(INT_SRC_M)
        # Check if the INT (interrupt active) bit is set
        if (intSrc & (1<<0)):
            return (intSrc & 0xFE)
        return 0;
    #
    
    # getGyroIntSrc() -- Get status of inactivity interrupt
    def getInactivity(self):
        temp = self.xgReadByte(STATUS_REG_0);
        temp &= (0x10);
        return temp;
    #
    
    # sleepGyro() -- Sleep or wake the gyroscope
    # Input:
    #	- enable: True = sleep gyro. False = wake gyro.
    def sleepGyro(self, enable = True):
        temp = self.xgReadByte(CTRL_REG9);
        if (enable): temp |= (1<<6);
        else: temp &= ~(1<<6);
        self.xgWriteByte(CTRL_REG9, temp);
    #
    
    # enableFIFO() - Enable or disable the FIFO
    # Input:
    #	- enable: true = enable, false = disable.
    def enableFIFO(self, enable = True):
        temp = self.xgReadByte(CTRL_REG9);
        if (enable): temp |= (1<<1);
        else: temp &= ~(1<<1);
        self.xgWriteByte(CTRL_REG9, temp);
    #
    
    # setFIFO() - Configure FIFO mode and Threshold
    # Input:
    #	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
    #	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
    #	- fifoThs: FIFO threshold level setting
    #	  Any value from 0-0x1F is acceptable.
    def setFIFO(self, fifoMode, fifoThs):
        # Limit threshold - 0x1F (31) is the maximum. If more than that was asked
        # limit it to the maximum.
        if fifoThs <= 0x1F:
            threshold = fifoThs 
        else: 
            threshold = 0x1F;
        self.xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
    #
    
    # getFIFOSamples() - Get number of FIFO samples
    def getFIFOSamples(self):
        return (self.xgReadByte(FIFO_SRC) & 0x3F);
    #
    
#protected:	
    
    # initGyro() -- Sets up the gyroscope to begin reading.
    # This function steps through all five gyroscope control registers.
    # Upon exit, the following parameters will be set:
    #	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled. 
    #		95 Hz ODR, 12.5 Hz cutoff frequency.
    #	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
    #		set to 7.2 Hz (depends on ODR).
    #	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
    #		active high). Data-ready output enabled on DRDY_G.
    #	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
    #		address. Scale set to 245 DPS. SPI mode set to 4-wire.
    #	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
    def initGyro(self):
        tempRegValue = 0;
        
        # CTRL_REG1_G (Default value: 0x00)
        # [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
        # ODR_G[2:0] - Output data rate selection
        # FS_G[1:0] - Gyroscope full-scale selection
        # BW_G[1:0] - Gyroscope bandwidth selection
        
        # To disable gyro, set sample rate bits to 0. We'll only set sample
        # rate if the gyro is enabled.
        if (self.gyro.enabled):
            tempRegValue = (self.gyro.sampleRate & 0x07) << 5;
        #
        if self.gyro.scale == 500:
            tempRegValue |= (0x1 << 3);
        elif self.gyro.scale == 2000:
            tempRegValue |= (0x3 << 3);
            # Otherwise we'll set it to 245 dps (0x0 << 4)
        #
        tempRegValue |= (self.gyro.bandwidth & 0x3)
        self.xgWriteByte(CTRL_REG1_G, tempRegValue)
        
        # CTRL_REG2_G (Default value: 0x00)
        # [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
        # INT_SEL[1:0] - INT selection configuration
        # OUT_SEL[1:0] - Out selection configuration
        self.xgWriteByte(CTRL_REG2_G, 0x00);
        
        # CTRL_REG3_G (Default value: 0x00)
        # [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
        # LP_mode - Low-power mode enable (0: disabled, 1: enabled)
        # HP_EN - HPF enable (0:disabled, 1: enabled)
        # HPCF_G[3:0] - HPF cutoff frequency
        if self.gyro.lowPowerEnable:
            tempRegValue = (1<<7) 
        else:
            tempRegValue = 0;
        #
        if (self.gyro.HPFEnable):
            tempRegValue |= (1<<6) | (self.gyro.HPFCutoff & 0x0F);
        #
        self.xgWriteByte(CTRL_REG3_G, tempRegValue)
    
        # CTRL_REG4 (Default value: 0x38)
        # [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
        # Zen_G - Z-axis output enable (0:disable, 1:enable)
        # Yen_G - Y-axis output enable (0:disable, 1:enable)
        # Xen_G - X-axis output enable (0:disable, 1:enable)
        # LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
        # 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
        tempRegValue = 0;
        if (self.gyro.enableZ): tempRegValue |= (1<<5);
        if (self.gyro.enableY): tempRegValue |= (1<<4);
        if (self.gyro.enableX): tempRegValue |= (1<<3);
        if (self.gyro.latchInterrupt): tempRegValue |= (1<<1);
        self.xgWriteByte(CTRL_REG4, tempRegValue);
    
        # ORIENT_CFG_G (Default value: 0x00)
        # [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
        # SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
        # Orient [2:0] - Directional user orientation selection
        tempRegValue = 0;
        if (self.gyro.flipX): tempRegValue |= (1<<5);
        if (self.gyro.flipY): tempRegValue |= (1<<4);
        if (self.gyro.flipZ): tempRegValue |= (1<<3);
        self.xgWriteByte(ORIENT_CFG_G, tempRegValue)
    #
    
    # initAccel() -- Sets up the accelerometer to begin reading.
    # This function steps through all accelerometer related control registers.
    # Upon exit these registers will be set as:
    #	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
    #	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
    #		all axes enabled.
    #	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
    #	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
    def initAccel(self):
        tempRegValue = 0;
        
        #	CTRL_REG5_XL (0x1F) (Default value: 0x38)
        #	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
        #	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
        #		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
        #	Zen_XL - Z-axis output enabled
        #	Yen_XL - Y-axis output enabled
        #	Xen_XL - X-axis output enabled
        if (self.accel.enableZ): tempRegValue |= (1<<5);
        if (self.accel.enableY): tempRegValue |= (1<<4);
        if (self.accel.enableX): tempRegValue |= (1<<3);
        self.xgWriteByte(CTRL_REG5_XL, tempRegValue);
    
        # CTRL_REG6_XL (0x20) (Default value: 0x00)
        # [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
        # ODR_XL[2:0] - Output data rate & power mode selection
        # FS_XL[1:0] - Full-scale selection
        # BW_SCAL_ODR - Bandwidth selection
        # BW_XL[1:0] - Anti-aliasing filter bandwidth selection
        tempRegValue = 0;
        # To disable the accel, set the sampleRate bits to 0.
        if (self.accel.enabled):
            tempRegValue |= (self.accel.sampleRate & 0x07) << 5;
        #
        if self.accel.scale ==  4:
            tempRegValue |= (0x2 << 3);
        elif self.accel.scale ==  8:
            tempRegValue |= (0x3 << 3);
        elif self.accel.scale ==  16:
            tempRegValue |= (0x1 << 3);
        # Otherwise it'll be set to 2g (0x0 << 3)
        
        if (self.accel.bandwidth >= 0):
            tempRegValue |= (1<<2); # Set BW_SCAL_ODR
            tempRegValue |= (self.accel.bandwidth & 0x03);
        self.xgWriteByte(CTRL_REG6_XL, tempRegValue);
        
        # CTRL_REG7_XL (0x21) (Default value: 0x00)
        # [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
        # HR - High resolution mode (0: disable, 1: enable)
        # DCF[1:0] - Digital filter cutoff frequency
        # FDS - Filtered data selection
        # HPIS1 - HPF enabled for interrupt function
        tempRegValue = 0;
        if (self.accel.highResEnable):
            tempRegValue |= (1<<7); # Set HR bit
            tempRegValue |= (self.accel.highResBandwidth & 0x3) << 5;
        self.xgWriteByte(CTRL_REG7_XL, tempRegValue);
    #
    
    # initMag() -- Sets up the magnetometer to begin reading.
    # This function steps through all magnetometer-related control registers.
    # Upon exit these registers will be set as:
    #	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
    #	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
    #		requests don't latch. Temperature sensor disabled.
    #	- CTRL_REG6_XM = 0x00:  2 Gs scale.
    #	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
    #	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
    def initMag(self):
        tempRegValue = 0;
        
        # CTRL_REG1_M (Default value: 0x10)
        # [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
        # TEMP_COMP - Temperature compensation
        # OM[1:0] - X & Y axes op mode selection
        #	00:low-power, 01:medium performance
        #	10: high performance, 11:ultra-high performance
        # DO[2:0] - Output data rate selection
        # ST - Self-test enable
        if (self.mag.tempCompensationEnable): tempRegValue |= (1<<7);
        tempRegValue |= (self.mag.XYPerformance & 0x3) << 5;
        tempRegValue |= (self.mag.sampleRate & 0x7) << 2;
        self.mWriteByte(CTRL_REG1_M, tempRegValue);
    
        # CTRL_REG2_M (Default value 0x00)
        # [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
        # FS[1:0] - Full-scale configuration
        # REBOOT - Reboot memory content (0:normal, 1:reboot)
        # SOFT_RST - Reset config and user registers (0:default, 1:reset)
        tempRegValue = 0;
        if self.mag.scale == 8:
            tempRegValue |= (0x1 << 5);
        elif self.mag.scale == 12:
            tempRegValue |= (0x2 << 5);
        elif self.mag.scale == 16:
            tempRegValue |= (0x3 << 5);
        # Otherwise we'll default to 4 gauss (00)
        
        self.mWriteByte(CTRL_REG2_M, tempRegValue); # +/-4Gauss
        
        # CTRL_REG3_M (Default value: 0x03)
        # [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
        # I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
        # LP - Low-power mode cofiguration (1:enable)
        # SIM - SPI mode selection (0:write-only, 1:read/write enable)
        # MD[1:0] - Operating mode
        #	00:continuous conversion, 01:single-conversion,
        #  10,11: Power-down
        tempRegValue = 0;
        if (self.mag.lowPowerEnable): tempRegValue |= (1<<5);
        tempRegValue |= (self.mag.operatingMode & 0x3);
        self.mWriteByte(CTRL_REG3_M, tempRegValue); # Continuous conversion mode
        
        # CTRL_REG4_M (Default value: 0x00)
        # [0][0][0][0][OMZ1][OMZ0][BLE][0]
        # OMZ[1:0] - Z-axis operative mode selection
        #	00:low-power mode, 01:medium performance
        #	10:high performance, 10:ultra-high performance
        # BLE - Big/little endian data
        tempRegValue = 0;
        tempRegValue = (self.mag.ZPerformance & 0x3) << 2;
        self.mWriteByte(CTRL_REG4_M, tempRegValue);
        
        # CTRL_REG5_M (Default value: 0x00)
        # [0][BDU][0][0][0][0][0][0]
        # BDU - Block data update for magnetic data
        #	0:continuous, 1:not updated until MSB/LSB are read
        tempRegValue = 0;
        self.mWriteByte(CTRL_REG5_M, tempRegValue);
    #
    
    # gReadByte() -- Reads a byte from a specified gyroscope register.
    # Input:
    # 	- subAddress = Register to be read from.
    # Output:
    # 	- An 8-bit value read from the requested address.
    def mReadByte(self, subAddress):
        # Whether we're using I2C or SPI, read a byte using the
        # accelerometer-specific I2C address or SPI CS pin.
        if (self.device.commInterface == IMU_MODE_I2C):
            return self.I2CreadByte(self.device.mAddress, subAddress);
    #    else if (self.device.commInterface == IMU_MODE_SPI)
    #        return self.SPIreadByte(self.device.mAddress, subAddress);
    #
    
    # gReadBytes() -- Reads a number of bytes -- beginning at an address
    # and incrementing from there -- from the gyroscope.
    # Input:
    # 	- subAddress = Register to be read from.
    # 	- * dest = A pointer to an array of uint8_t's. Values read will be
    #		stored in here on return.
    #	- count = The number of bytes to be read.
    # Output: No value is returned, but the `dest` array will store
    # 	the data read upon exit.
    def mReadBytes(self, subAddress, count):
        # Whether we're using I2C or SPI, read multiple bytes using the
        # accelerometer-specific I2C address or SPI CS pin.
        if (self.device.commInterface == IMU_MODE_I2C):
            return self.I2CreadBytes(self.device.mAddress, subAddress, count);
    #    else if (self.device.commInterface == IMU_MODE_SPI)
    #        return self.SPIreadBytes(self.device.mAddress, subAddress, count);
    #
    
    # gWriteByte() -- Write a byte to a register in the gyroscope.
    # Input:
    #	- subAddress = Register to be written to.
    #	- data = data to be written to the register.
    def mWriteByte(self, subAddress, data):
        # Whether we're using I2C or SPI, write a byte using the
        # accelerometer-specific I2C address or SPI CS pin.
        if (self.device.commInterface == IMU_MODE_I2C):
            return self.I2CwriteByte(self.device.mAddress, subAddress, data);
    #    else if (self.device.commInterface == IMU_MODE_SPI)
    #        return SPIwriteByte(self.device.mAddress, subAddress, data);
    #
    
    # xmReadByte() -- Read a byte from a register in the accel/mag sensor
    # Input:
    #	- subAddress = Register to be read from.
    # Output:
    #	- An 8-bit value read from the requested register.
    def xgReadByte(self, subAddress):
        # Whether we're using I2C or SPI, read a byte using the
        # gyro-specific I2C address or SPI CS pin.
        if (self.device.commInterface == IMU_MODE_I2C):
            return self.I2CreadByte(self.device.agAddress, subAddress);
    #    else if (self.device.commInterface == IMU_MODE_SPI)
    #        return self.SPIreadByte(self.device.agAddress, subAddress);
    #
    
    # xmReadBytes() -- Reads a number of bytes -- beginning at an address
    # and incrementing from there -- from the accelerometer/magnetometer.
    # Input:
    # 	- subAddress = Register to be read from.
    # 	- * dest = A pointer to an array of uint8_t's. Values read will be
    #		stored in here on return.
    #	- count = The number of bytes to be read.
    # Output: No value is returned, but the `dest` array will store
    # 	the data read upon exit.
    def xgReadBytes(self, subAddress, count):
        # Whether we're using I2C or SPI, read multiple bytes using the
        # gyro-specific I2C address or SPI CS pin.
        if (self.device.commInterface == IMU_MODE_I2C):
            return self.I2CreadBytes(self.device.agAddress, subAddress, count);
    #    else if (self.device.commInterface == IMU_MODE_SPI)
    #        return SPIreadBytes(self.device.agAddress, subAddress, count);
    #
    
    # xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
    # Input:
    #	- subAddress = Register to be written to.
    #	- data = data to be written to the register.
    def xgWriteByte(self, subAddress, data):
        # Whether we're using I2C or SPI, write a byte using the
        # gyro-specific I2C address or SPI CS pin.
        if (self.device.commInterface == IMU_MODE_I2C):
            self.I2CwriteByte(self.device.agAddress, subAddress, data);
        #else if (self.device.commInterface == IMU_MODE_SPI)
        #    SPIwriteByte(self.device.agAddress, subAddress, data);
    #
    
    # calcgRes() -- Calculate the resolution of the gyroscope.
    # This function will set the value of the gRes variable. gScale must
    # be set prior to calling this function.
    def calcgRes(self):
        self.gRes = self.gyro.scale / 32768.0
    
    # calcmRes() -- Calculate the resolution of the magnetometer.
    # This function will set the value of the mRes variable. mScale must
    # be set prior to calling this function.
    def calcmRes(self):
        #mRes = ((float) self.mag.scale) / 32768.0;
        if self.mag.scale ==  4:
            self.mRes = magSensitivity[0];
        if self.mag.scale ==  8:
            self.mRes = magSensitivity[1];
        if self.mag.scale ==  12:
            self.mRes = magSensitivity[2];
        if self.mag.scale ==  16:
            self.mRes = magSensitivity[3];
    #
    
    # calcaRes() -- Calculate the resolution of the accelerometer.
    # This function will set the value of the aRes variable. aScale must
    # be set prior to calling this function.
    def calcaRes(self):
        self.aRes = self.accel.scale / 32768.0
    
    ###########
    # Helper Functions #
    ###########
    def constrainScales(self):
        if ((self.gyro.scale != 245) and (self.gyro.scale != 500) and (self.gyro.scale != 2000)):
            self.gyro.scale = 245
        #
        if ((self.accel.scale != 2) and (self.accel.scale != 4) and (self.accel.scale != 8) and (self.accel.scale != 16)):
            self.accel.scale = 2
        #
        if ((self.mag.scale != 4) and (self.mag.scale != 8) and (self.mag.scale != 12) and (self.mag.scale != 16)):
            self.mag.scale = 4
        #
    #
    
    #########/
    # SPI Functions #
    #########/
    # initSPI() -- Initialize the SPI hardware.
    # This function will setup all SPI pins and related hardware.
    #def initSPI(self);
    
    # SPIwriteByte() -- Write a byte out of SPI to a register in the device
    # Input:
    #	- csPin = The chip select pin of the slave device.
    #	- subAddress = The register to be written to.
    #	- data = Byte to be written to the register.
    #def SPIwriteByte(self, uint8_t csPin, uint8_t subAddress, uint8_t data);
    
    # SPIreadByte() -- Read a single byte from a register over SPI.
    # Input:
    #	- csPin = The chip select pin of the slave device.
    #	- subAddress = The register to be read from.
    # Output:
    #	- The byte read from the requested address.
    #uint8_t SPIreadByte(uint8_t csPin, uint8_t subAddress);
    
    # SPIreadBytes() -- Read a series of bytes, starting at a register via SPI
    # Input:
    #	- csPin = The chip select pin of a slave device.
    #	- subAddress = The register to begin reading.
    # 	- * dest = Pointer to an array where we'll store the readings.
    #	- count = Number of registers to be read.
    # Output: No value is returned by the function, but the registers read are
    # 		all stored in the *dest array given.
    #def SPIreadBytes(self, uint8_t csPin, uint8_t subAddress, 
    #						uint8_t * dest, uint8_t count);
    
    #########/
    # I2C Functions #
    #########/
    # See http://wiki.erazor-zone.de/wiki:linux:python:smbus:doc
    # https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/plain/Documentation/i2c/smbus-protocol
    
    
    
    # initI2C() -- Initialize the I2C hardware.
    # This function will setup all I2C pins and related hardware.
    def initI2C(self):
        self.bus = SMBus(1)
    #
    
    # I2CwriteByte() -- Write a byte out of I2C to a register in the device
    # Input:
    #	- address = The 7-bit I2C address of the slave device.
    #	- subAddress = The register to be written to.
    #	- data = Byte to be written to the register.
    def I2CwriteByte(self, address, subAddress, data):
        self.bus.write_byte_data(address, subAddress&0xFF, data&0xFF)
    #
    
    # I2CreadByte() -- Read a single byte from a register over I2C.
    # Input:
    #	- address = The 7-bit I2C address of the slave device.
    #	- subAddress = The register to be read from.
    # Output:
    #	- The byte read from the requested address.
    def I2CreadByte(self, address, subAddress):
        return self.bus.read_byte_data(address, subAddress&0xFF)
    #
    
    # I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
    # Input:
    #	- address = The 7-bit I2C address of the slave device.
    #	- subAddress = The register to begin reading.
    # 	- * dest = Pointer to an array where we'll store the readings.
    #	- count = Number of registers to be read.
    # Output: No value is returned by the function, but the registers read are
    # 		all stored in the *dest array given.
    def I2CreadBytes(self, address, subAddress, count):
        arr =  self.bus.read_i2c_block_data(address, subAddress&0xFF, count)
        print(arr)
        return arr[0:count]
    #
#






imu = LSM9DS1()
imu.begin()
#imu.calibrate()
imu.calibrateMag()

print(imu.readAccelCal())
print(imu.readGyroCal())
print(imu.readMagCal())
print(imu.readTemp())



