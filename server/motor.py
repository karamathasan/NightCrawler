import numpy as np
from DXLConfig import DXLConfig, Addresses
from dynamixel_sdk import *

class Motor():
    def __init__(self, id):
        self.id = id
        self.bounds = [-150, 150]
        self.config = DXLConfig.getInstance()
        self.getBounds()

        # if not config.opened:
        #     raise Exception("Port not opened!")

        # if self.id not in config.availableMotors:
        #     raise Exception(f"Motor ID {id} not found!")

    def enable_torque(self):
        if not self.config.opened:
            raise Exception("Port not opened!")

        result, error = self.config.write1B(self.id, Addresses.TORQUE_ENABLE.value, 1)
        if result != COMM_SUCCESS:
            print("%s" % self.config.getResult(result))
        elif error != 0:
            print("%s" % self.config.getError(error))
        else:
            print(f"Dynamixel AX-12 ID: {self.id} Torque Enabled!")

    def disable_torque(self):
        if not self.config.opened:
            raise Exception("Port not opened!")

        result, error = self.config.write1B(self.id, Addresses.TORQUE_ENABLE.value, 1)
        if result != COMM_SUCCESS:
            print("%s" % self.config.getResult(result))
        elif error != 0:
            print("%s" % self.config.getError(error))
        else:
            print(f"Dynamixel AX-12 ID: {self.id} Torque Disabled!")

    def reset(self, newId = None):
        if not self.config.opened:
            raise Exception("Port not opened!")
        
        self.setSpeed()
        self.setBounds()

        if newId is not None:
            result, error = self.config.write1B(self.id, Addresses.ID.value, newId)
            self.id = newId

    def getBounds(self):
        if not self.config.opened:
            raise Exception("Port not opened!")

        cwByte, result, err = self.config.read2B(self.id, Addresses.CW_ANGLE_LIMIT.value)
        ccwByte, result, err = self.config.read2B(self.id, Addresses.CCW_ANGLE_LIMIT.value)
        cwBound = self.byte2angle(cwByte)
        ccwBound = self.byte2angle(ccwByte)

        self.bounds = [cwBound, ccwBound]
        return self.bounds

    def setBounds(self, cwAngle=-150, ccwAngle=150):
        if not self.config.opened:
            raise Exception("Port not opened!")

        self.bounds = [cwAngle, ccwAngle]
        cwbyte = self.angle2byte(cwAngle)
        ccwbyte = self.angle2byte(ccwAngle)

        result, err = self.config.write2B(self.id, Addresses.CW_ANGLE_LIMIT.value, cwbyte)
        result, err = self.config.write2B(self.id, Addresses.CCW_ANGLE_LIMIT.value, ccwbyte)        

    def setAngle(self, angle):
        angle = self.angle2byte(angle)
        if not self.config.opened:
            raise Exception("Port not opened!")
        
        result, error = self.config.write2B(self.id, Addresses.GOAL_POSITION.value, angle)
        if result != COMM_SUCCESS:
            print("%s" % self.config.getResult(result))
        elif error != 0:
            print("%s" % self.config.getError(error))

    def getAngle(self):
        if not self.config.opened:
            raise Exception("Port not opened!")
        
        value, result, error = self.config.read2B(self.id, Addresses.GOAL_POSITION.value)
        # print(f"Position bytes: {value}")
        return self.byte2angle(value)

    def setPosition(self, position):
        if not self.config.opened:
            raise Exception("Port not opened!")
        
        result, error = self.config.write2B(self.id, Addresses.GOAL_POSITION.value, position)
        if result != COMM_SUCCESS:
            print("%s" % self.config.getResult(result))
        elif error != 0:
            print("%s" % self.config.getError(error))
    
    def getPosition(self):
        if not self.config.opened:
            raise Exception("Port not opened!")
        
        value, result, error = self.config.read2B(self.id, Addresses.GOAL_POSITION.value)
        # print(f"Position bytes: {value}")
        return value
    

    def setSpeed(self, speedbytes=0):
        if not self.config.opened:
            raise Exception("Port not opened!")
        
        result, error = self.config.write2B(self.id, Addresses.MOVING_SPEED.value, speedbytes)
        if result != COMM_SUCCESS:
            print("%s" % self.config.getResult(result))
        elif error != 0:
            print("%s" % self.config.getError(error))

    def getSpeed(self):
        if not self.config.opened:
            raise Exception("Port not opened!")
        
        value, result, error = self.config.read2B(self.id, Addresses.MOVING_SPEED.value)
        print(value)

    def angle2byte(self, angle):
        assert self.bounds[0] <= angle <= self.bounds[1], f"Angle {angle} does fit in bounds {self.bounds[0],self.bounds[1]}"
        # range = self.bounds[1] - self.bounds[0]
        # shifted = angle - self.bounds[0]
        range = 300
        shifted = angle + 150
        return min(int(1024*(shifted)/range),1023)

    def byte2angle(self, byte):
        # range = self.bounds[1] - self.bounds[0]
        range = 300
        return int(range * byte/1023 + self.bounds[0])
