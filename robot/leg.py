from motor import Motor
import numpy as np
class Leg():
    """
    ### Leg
    A representation of a single leg of the bot. Each joint can be controlled relative to the angle that the servos are facing
    """
    def __init__(self, right, motor1:Motor, motor2:Motor, motor3:Motor):
    # def __init__(self, right, leg_id, motor1:int, motor2:int, motor3:int):
        self.right = right

        # self.motor1 = Motor(motor1)
        # self.motor2 = Motor(motor2)
        # self.motor3 = Motor(motor3)

        self.motor1 = motor1
        self.motor2 = motor2
        self.motor3 = motor3

        # # in mm
        # if right:
        #     if leg_id == 1:
        #         self.L0 = np.array([1,0,1])
        #     if leg_id == 2:
        #         self.L0 = np.array([1,0,1])
        #     if leg_id == 3:
        #         self.L0 = np.array([1,0,1])
        # else:
        #     if leg_id == 1:
        #         self.L0 = np.array([1,0,1])
        #     if leg_id == 2:
        #         self.L0 = np.array([1,0,1])
        #     if leg_id == 3:
        #         self.L0 = np.array([1,0,1])

        # self.L1 = np.array([67.5,0,0])
        # self.L2 = np.array([67.5,0,0])
        # self.l3 = np.array([1,-1,0])

        motor1.setBounds(-90,90)
        motor2.setBounds(-90,90)
        motor3.setBounds(-90,90)

    def getMotors(self):
        return (self.motor1, self.motor2, self.motor3)
    
    def getJointAngles(self):
        return (self.motor1.getAngle(), self.motor2.getAngle(), self.motor3.getAngle())

    def setJoint1(self, angle):
        self.motor1.setAngle(angle)

    def setJoint2(self, angle):
        self.motor2.setAngle(angle)
    
    def setJoint3(self, angle):
        self.motor3.setAngle(angle)

    def setJoints(self, theta1, theta2, theta3):
        self.motor1.setAngle(theta1)
        self.motor2.setAngle(theta2)
        self.motor3.setAngle(theta3)

    def setEndPosition(self, pos):
        pass

    def getEndPosition(self):
        pass