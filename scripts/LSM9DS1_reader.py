#!/usr/bin/env python
# coding: utf-8

# Based on example and using library from
# https://github.com/akimach/LSM9DS1_RaspberryPi_Library

#import ctypes
from ctypes import *

path = "../lib/liblsm9ds1cwrapper.so"
lib = cdll.LoadLibrary(path)

lib.lsm9ds1_create.argtypes = []
lib.lsm9ds1_create.restype = c_void_p

lib.lsm9ds1_begin.argtypes = [c_void_p]
lib.lsm9ds1_begin.restype = None

lib.lsm9ds1_calibrate.argtypes = [c_void_p]
lib.lsm9ds1_calibrate.restype = None

lib.lsm9ds1_gyroAvailable.argtypes = [c_void_p]
lib.lsm9ds1_gyroAvailable.restype = c_int
lib.lsm9ds1_accelAvailable.argtypes = [c_void_p]
lib.lsm9ds1_accelAvailable.restype = c_int
lib.lsm9ds1_magAvailable.argtypes = [c_void_p]
lib.lsm9ds1_magAvailable.restype = c_int

lib.lsm9ds1_readGyro.argtypes = [c_void_p]
lib.lsm9ds1_readGyro.restype = c_int
lib.lsm9ds1_readAccel.argtypes = [c_void_p]
lib.lsm9ds1_readAccel.restype = c_int
lib.lsm9ds1_readMag.argtypes = [c_void_p]
lib.lsm9ds1_readMag.restype = c_int

lib.lsm9ds1_getGyroX.argtypes = [c_void_p]
lib.lsm9ds1_getGyroX.restype = c_float
lib.lsm9ds1_getGyroY.argtypes = [c_void_p]
lib.lsm9ds1_getGyroY.restype = c_float
lib.lsm9ds1_getGyroZ.argtypes = [c_void_p]
lib.lsm9ds1_getGyroZ.restype = c_float

lib.lsm9ds1_getAccelX.argtypes = [c_void_p]
lib.lsm9ds1_getAccelX.restype = c_float
lib.lsm9ds1_getAccelY.argtypes = [c_void_p]
lib.lsm9ds1_getAccelY.restype = c_float
lib.lsm9ds1_getAccelZ.argtypes = [c_void_p]
lib.lsm9ds1_getAccelZ.restype = c_float

lib.lsm9ds1_getMagX.argtypes = [c_void_p]
lib.lsm9ds1_getMagX.restype = c_float
lib.lsm9ds1_getMagY.argtypes = [c_void_p]
lib.lsm9ds1_getMagY.restype = c_float
lib.lsm9ds1_getMagZ.argtypes = [c_void_p]
lib.lsm9ds1_getMagZ.restype = c_float

lib.lsm9ds1_calcGyro.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcGyro.restype = c_float
lib.lsm9ds1_calcAccel.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcAccel.restype = c_float
lib.lsm9ds1_calcMag.argtypes = [c_void_p, c_float]
lib.lsm9ds1_calcMag.restype = c_float

class LSM9DS1:
    def __init__(self):
        self.imu = lib.lsm9ds1_create()
        if lib.lsm9ds1_begin(imu) == 0:
            print("Failed to communicate with LSM9DS1.")
            quit()
        lib.lsm9ds1_calibrate(imu)
        
        self.ax = 0
        self.ay = 0
        self.az = 0
    #
    
    def pollBlocking(self):
        while lib.lsm9ds1_gyroAvailable(self.imu) == 0:
            pass
        lib.lsm9ds1_readGyro(self.imu)
        while lib.lsm9ds1_accelAvailable(self.imu) == 0:
            pass
        lib.lsm9ds1_readAccel(self.imu)
        while lib.lsm9ds1_magAvailable(self.imu) == 0:
            pass
        lib.lsm9ds1_readMag(self.imu)
    #
    def poll(self):
        hasNewData = False
        if lib.lsm9ds1_gyroAvailable(self.imu):
            lib.lsm9ds1_readGyro(self.imu)
            hasNewData = True
        if lib.lsm9ds1_accelAvailable(self.imu):
            lib.lsm9ds1_readAccel(self.imu)
            hasNewData = True
        while lib.lsm9ds1_magAvailable(self.imu):
            lib.lsm9ds1_readMag(self.imu)
            hasNewData = True
        return hasNewData
    #
    
    def getAccel(self):
        cax = lib.lsm9ds1_calcAccel(imu, ax)
        cay = lib.lsm9ds1_calcAccel(imu, ay)
        caz = lib.lsm9ds1_calcAccel(imu, az)
        return (cax, cay, caz)
    #
    def getGyro(self):
        cgx = lib.lsm9ds1_calcGyro(imu, gx)
        cgy = lib.lsm9ds1_calcGyro(imu, gy)
        cgz = lib.lsm9ds1_calcGyro(imu, gz)
        return (cgx, cgy, cgz)
    #
    def getMag(self):
        cmx = lib.lsm9ds1_calcMag(imu, mx)
        cmy = lib.lsm9ds1_calcMag(imu, my)
        cmz = lib.lsm9ds1_calcMag(imu, mz)
        return (cmx, cmy, cmz)
    #
#


if __name__ == "__main__":
    imu = LSM9DS1()

    while True:
        imu.pollBlocking()

# I think these are the raw 16-bit values?
        gx = lib.lsm9ds1_getGyroX(imu)
        gy = lib.lsm9ds1_getGyroY(imu)
        gz = lib.lsm9ds1_getGyroZ(imu)

        ax = lib.lsm9ds1_getAccelX(imu)
        ay = lib.lsm9ds1_getAccelY(imu)
        az = lib.lsm9ds1_getAccelZ(imu)

        mx = lib.lsm9ds1_getMagX(imu)
        my = lib.lsm9ds1_getMagY(imu)
        mz = lib.lsm9ds1_getMagZ(imu)

        cax, cay, caz = imu.getAccel()
        cgx, cgy, cgz = imu.getGyro()
        cmx, cmy, cmz = imu.getMag()

        print("Gyro: %f, %f, %f [deg/s]" % (cgx, cgy, cgz))
        print("Accel: %f, %f, %f [Gs]" % (cax, cay, caz))
        print("Mag: %f, %f, %f [gauss]" % (cmx, cmy, cmz))
