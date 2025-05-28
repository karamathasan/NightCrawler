from motor import Motor
import numpy as np
from matplotlib import pyplot as plt

class Leg():
    """
    ### Leg
    A representation of a single leg of the bot. Each joint can be controlled relative to the angle that the servos are facing
    """
    def __init__(self, right, leg_id, motor1:Motor, motor2:Motor, motor3:Motor):
        self.right = right
        self.leg_id = leg_id

        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3
        
        # # in mm
        motor1.setBounds(-90,90)
        motor2.setBounds(-90,90)
        motor3.setBounds(-90,90)
        self.setOrigin()
        
        motor1.enable_torque()
        motor2.enable_torque()
        motor3.enable_torque()

    def setOrigin(self):
        if self.right:
            if self.leg_id == 1:
                self.leg_origin = LegConstants.RIGHT1
                self.leg_origin_angle = 60
            if self.leg_id == 2:
                self.leg_origin = LegConstants.RIGHT2
                self.leg_origin_angle = 0
            if self.leg_id == 3:
                self.leg_origin = LegConstants.RIGHT3
                self.leg_origin_angle = -60
        else:
            if self.leg_id == 1:
                self.leg_origin = LegConstants.LEFT1
                self.leg_origin_angle = 120
            if self.leg_id == 2:
                self.leg_origin = LegConstants.LEFT2
                self.leg_origin_angle = 180
            if self.leg_id == 3:
                self.leg_origin = LegConstants.LEFT3
                self.leg_origin_angle = 240


    def getMotors(self):
        return (self.motor1, self.motor2, self.motor3)
    
    def getJointAngles(self):
        if self.right:
            return (self.motor1.getAngle(), self.motor2.getAngle(), self.motor3.getAngle())
        else:
            return (-self.motor1.getAngle(), -self.motor2.getAngle(), -self.motor3.getAngle())


    def setJoint1(self, angle):
        if self.right:
            self.motor1.setAngle(angle)
        else:
            self.motor1.setAngle(-angle)

    def setJoint2(self, angle):
        if self.right:
            self.motor2.setAngle(angle)
        else:
            self.motor2.setAngle(-angle)
    
    def setJoint3(self, angle):
        if self.right:
            self.motor3.setAngle(angle)
        else:
            self.motor3.setAngle(-angle)

    def setJoints(self, theta1, theta2, theta3):
        def wait():
            while self.isMoving():
                pass
            return 
        
        if self.right:
            self.motor1.setAngle(theta1)
            self.motor2.setAngle(theta2)
            self.motor3.setAngle(theta3)
        else:
            self.motor1.setAngle(-theta1)
            self.motor2.setAngle(-theta2)
            self.motor3.setAngle(-theta3)

        return wait

    def setSpeeds(self, byte1=0, byte2=0, byte3=0):
        self.motor1.setSpeed(byte1)
        self.motor2.setSpeed(byte2)
        self.motor3.setSpeed(byte3)

    def getSpeeds(self):
        return self.motor1.getSpeed(), self.motor2.getSpeed(), self.motor3.getSpeed()

    def isMoving(self):
        return self.motor1.isMoving() or self.motor2.isMoving() or self.motor3.isMoving()

    def solveEndPosition(self, pos):
        def RY(rad):
            return np.array([
                [np.cos(rad), 0, np.sin(rad)],
                [0,1,0],
                [-np.sin(rad),0, np.cos(rad)]
            ])
    
        L0 = LegConstants.origin_disp
        L1 = np.linalg.norm(LegConstants.FEMUR)
        L2 = np.linalg.norm(LegConstants.TIBIA)
        L3 = np.linalg.norm(LegConstants.TARSUS)
        
        p = np.sqrt(pos[0]**2 + pos[2]**2)
        q = np.sqrt((pos[0]-self.leg_origin[0,0])**2 + (pos[2]-self.leg_origin[0,2])**2)
        theta0 = np.deg2rad(self.leg_origin_angle)
        theta1 = np.acos(round((p**2 -L0**2 - q**2)/(2*L0*q),9))

        if pos[2] - self.leg_origin[0,2] > 0:
            theta1 *= -1

        if self.right:
            rvec = pos - (self.leg_origin[0] + L1 * RY(theta0 + theta1) @ np.array([1,0,0]))
        else:
            rvec = pos - (self.leg_origin[0] + L1 * RY(theta0 - theta1) @ np.array([1,0,0]))
        # print(pos)
        rx = rvec[0]
        ry = rvec[1]
        rz = rvec[2]
        r = np.linalg.norm(rvec)
        
        s = -np.acos(round((r**2-L2**2-L3**2)/(2 * L2 * L3),9))
        # print(s)
        theta3 = round(s - LegConstants.tarsus_angle,9)
        theta2 = round(np.atan2(ry, np.sqrt(rx**2 + rz**2)) - np.atan2(L3 * np.sin(s), L2 + L3*np.cos(s)),9)

        theta1 = np.rad2deg(theta1)
        theta2 = np.rad2deg(theta2)
        theta3 = np.rad2deg(theta3)
        
        if 90<np.abs(theta1)<91:
            theta1 = np.clip(theta1,-90,90)

        if 90<np.abs(theta2)<91:
            theta2 = np.clip(theta2,-90,90)       

        if 90<np.abs(theta3)<91:
            theta3 = np.clip(theta3,-90,90)
        # print(theta3)
        return round((theta1)), -round((theta2)), -round((theta3))

        # set angles 


    def findEndPosition(self):
        def RZ(rad):
            return np.array([
                [np.cos(rad),-np.sin(rad),0],
                [np.sin(rad), np.cos(rad),0],
                [0,0,1]
            ])

        def RY(rad):
            return np.array([
                [np.cos(rad), 0, np.sin(rad)],
                [0,1,0],
                [-np.sin(rad),0, np.cos(rad)]
            ])

        def H(rot:np.ndarray, disp:np.ndarray):
            return np.block([
                [rot, disp.T],
                [np.zeros((1,3)),1]
            ])
        if self.right:
            H0 = H(RY(np.deg2rad(self.motor1.getAngle() + self.leg_origin_angle)), self.leg_origin)
            H1 = H(RZ(np.deg2rad(-self.motor2.getAngle())), LegConstants.FEMUR)
            H2 = H(RZ(np.deg2rad(-self.motor3.getAngle())), LegConstants.TIBIA)
            H3 = H(np.eye(3),LegConstants.TARSUS)
        else:
            H0 = H(RY(np.deg2rad(self.motor1.getAngle() + self.leg_origin_angle)), self.leg_origin)
            H1 = H(RZ(np.deg2rad(self.motor2.getAngle())), LegConstants.FEMUR)
            H2 = H(RZ(np.deg2rad(self.motor3.getAngle())), LegConstants.TIBIA)
            H3 = H(np.eye(3),LegConstants.TARSUS)
        return (H0 @ H1 @ H2 @ H3) @ np.array([[0,0,0,1]]).T
    

class LegConstants():
    # in mm
    FEMUR = np.array([[67.5,0,0]])
    TIBIA = np.array([[67.5,0,0]])
    TARSUS = np.array([[188.517,-39.055,0]])
    # tarsus may change due to new part 

    tarsus_angle = np.atan2(-39.055,188.517)
    origin_disp = 123.099
    
    RIGHT1 = np.array([[origin_disp*np.cos(np.pi/3),16,-origin_disp*np.sin(np.pi/3)]])
    RIGHT2 = np.array([[origin_disp,16,0]])
    RIGHT3 = np.array([[origin_disp*np.cos(np.pi/3),16,origin_disp*np.sin(np.pi/3)]])

    LEFT1 = np.array([[-origin_disp*np.cos(np.pi/3),16,-origin_disp*np.sin(np.pi/3)]])
    LEFT2 = np.array([[-origin_disp,16,0]])
    LEFT3 = np.array([[-origin_disp*np.cos(np.pi/3),16,origin_disp*np.sin(np.pi/3)]])