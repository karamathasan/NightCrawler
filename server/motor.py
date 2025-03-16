import numpy as np
from DXLConfig import DXLConfig, Addresses
from dynamixel_sdk import *

class Motor():
    def __init__(self, id):
        self.id = id
        self.bounds = [-150,150]
        self.getBounds()

        # config = DXLConfig.getInstance()

        # if not config.opened:
        #     raise Exception("Port not opened!")

        # if self.id not in config.availableMotors:
        #     raise Exception(f"Motor ID {id} not found!")

    def enable_torque(self):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")

        result, error = config.write1B(self.id, Addresses.TORQUE_ENABLE.value, 1)
        if result != COMM_SUCCESS:
            print("%s" % config.getResult(result))
        elif error != 0:
            print("%s" % config.getError(error))
        else:
            print(f"Dynamixel AX-12 ID: {self.id} Enabled!")

    def disable_torque(self):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")

        result, error = config.write1B(self.id, Addresses.TORQUE_ENABLE.value, 1)
        if result != COMM_SUCCESS:
            print("%s" % config.getResult(result))
        elif error != 0:
            print("%s" % config.getError(error))
        else:
            print(f"Dynamixel AX-12 ID: {self.id} Disabled!")

    def reset(self, newId = None):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")
        
        self.setSpeed()
        self.setBounds()

        if newId is not None:
            result, error = config.write1B(self.id, Addresses.ID.value, newId)
            self.id = newId

        
    def getBounds(self):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")

        cwByte, result, err = config.read2B(self.id, Addresses.CW_ANGLE_LIMIT.value)
        ccwByte, result, err = config.read2B(self.id, Addresses.CCW_ANGLE_LIMIT.value)
        cwBound = self.byte2angle(cwByte)
        ccwBound = self.byte2angle(ccwByte)
        # print(cwByte,ccwByte)
        self.bounds = [cwBound, ccwBound]

    def setBounds(self, cwAngle=-150, ccwAngle=150):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")

        self.bounds = [cwAngle, ccwAngle]
        cwbyte = self.angle2byte(cwAngle)
        ccwbyte = self.angle2byte(ccwAngle)

        result, err = config.write2B(self.id, Addresses.CW_ANGLE_LIMIT.value, cwbyte)
        result, err = config.write2B(self.id, Addresses.CCW_ANGLE_LIMIT.value, ccwbyte)        

    def setPosition(self, angle):
        angle = self.angle2byte(angle)
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")
        
        result, error = config.write2B(self.id, Addresses.GOAL_POSITION.value, angle)
        if result != COMM_SUCCESS:
            print("%s" % config.getResult(result))
        elif error != 0:
            print("%s" % config.getError(error))

    def getPosition(self):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")
        
        value, result, error = config.read2B(self.id, Addresses.GOAL_POSITION.value)
        print(f"Position bytes: {value}")

    def setSpeed(self, speedbytes=0):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")
        
        result, error = config.write2B(self.id, Addresses.MOVING_SPEED.value, speedbytes)
        if result != COMM_SUCCESS:
            print("%s" % config.getResult(result))
        elif error != 0:
            print("%s" % config.getError(error))

    def getSpeed(self):
        config = DXLConfig.getInstance()
        if not config.opened:
            raise Exception("Port not opened!")
        
        value, result, error = config.read2B(self.id, Addresses.MOVING_SPEED.value)
        print(value)

    def angle2byte(self, angle):
        range = self.bounds[1] - self.bounds[0]
        shifted = angle - self.bounds[0]
        return min(int(1024*(shifted)/range),1023)

    def byte2angle(self, byte):
        range = self.bounds[1] - self.bounds[0]
        return int(range * byte/1023 + self.bounds[0])
