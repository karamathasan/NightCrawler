import os
from enum import Enum
from dynamixel_sdk import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class DXLConfig():
    def __new__(cls, devicename, baudrate = 1000000, protocol = 1.0):
        if not hasattr(cls,'instance'):
            cls.instance = super(DXLConfig, cls).__new__(cls)
        return cls.instance

    def __init__(self, devicename, baudrate = 1000000, protocol = 1.0):
        self.DEVICENAME = devicename # DEVICENAME WILL BE DIFFERENT FORMAT FOR PI
        self.BAUDRATE = baudrate
        self.PROTOCOL_VERSION = protocol

        self.availableMotors = []

        self.portHandler = PortHandler(self.DEVICENAME)

        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.opened = False

    @staticmethod
    def getInstance():
        return DXLConfig.instance
    
    def open(self):
        if self.portHandler.openPort():
            self.opened = True
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # self.findIDs()

    def close(self):
        print("Closing port")
        self.opened = False
        if not self.portHandler:
            print("porthandler doesnt exists")
        self.portHandler.closePort()

    def read1B(self, id, addr):
        return self.packetHandler.read1ByteTxRx(self.portHandler, id, addr)
    
    def write1B(self, id, addr, data):
        return self.packetHandler.write1ByteTxRx(self.portHandler, id, addr, data)

    def read2B(self, id, addr):
        return self.packetHandler.read2ByteTxRx(self.portHandler, id, addr)
    
    def write2B(self, id, addr, data):    
        return self.packetHandler.write2ByteTxRx(self.portHandler, id, addr, data)
    
    def getResult(self, result):
        return self.packetHandler.getTxRxResult(result)
    
    def getError(self, error):
        return self.packetHandler.getRxPacketError(error)

    def findIDs(self):
        for test_id in range(0, 254):
        # for test_id in range(0, 19):
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, test_id)
            if dxl_comm_result == COMM_SUCCESS:
                self.availableMotors.append(test_id)
                print(f"Found Dynamixel at ID: {test_id}, Model: {dxl_model_number}")   
        print("Pinging complete")

    def findFirstID(self):
        for test_id in range(0, 254):
        # for test_id in range(0, 19):
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, test_id)
            if dxl_comm_result == COMM_SUCCESS:
                self.availableMotors.append(test_id)
                print(f"Found Dynamixel at ID: {test_id}, Model: {dxl_model_number}")   
                return test_id

class Addresses(Enum):
    MODEL_NUMBER = 0 #2b
    FIRMWARE_VERSION = 2
    ID = 3
    BAUD_RATE = 4
    RETURN_DELAY_TIME = 5
    CW_ANGLE_LIMIT = 6 #2b
    CCW_ANGLE_LIMIT = 8 #2b
    TEMPERATURE_LIMIT = 11
    MIN_VOLTAGE_LIMIT = 12
    MAX_VOLTAGE_LIMIT = 13
    MAX_TORQUE = 14 #2b
    STATUS_RETURN_LEVEL = 16
    ALARM_LED = 17
    SHUTDOWN = 18

    TORQUE_ENABLE = 24
    LED = 25
    CW_COMPLIANCE_MARGIN = 26
    CCW_COMPLIANCE_MARGIN = 27 
    CW_COMPLIANCE_SLOPE = 28
    CCW_COMPLIANCE_SLOPE = 29
    GOAL_POSITION = 30 #2b
    MOVING_SPEED = 32 #2b
    TOQUE_LIMIT = 34 #2b
    PRESENT_POSITION = 36 #2b
    PRESENT_SPEED = 38 #2b
    PRESENT_LOAD = 40 #2b
    PRESENT_VOLTAGE = 42
    PRESENT_TEMPRATURE = 43
    REGISTERED = 44
    MOVING = 46
    LOCK = 47
    PUNCH = 48 #2b