from motor import Motor
from trajectory import Trajectory
from enum import Enum

import numpy as np

from matplotlib import pyplot as plt

class Leg():
    """
    ### Leg
    A representation of a single leg of the bot. Each joint can be controlled relative to the angle that the servos are facing
    """
    def __init__(self, right, leg_id, motor1:Motor, motor2:Motor, motor3:Motor):
    # def __init__(self, right, leg_id, motor1:int, motor2:int, motor3:int):
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
        L0 = LegConstants.origin_disp
        L1 = LegConstants.FEMUR
        L2 = LegConstants.TIBIA
        L3 = LegConstants.TARSUS
        if self.right:
            theta1 = np.acos((np.dot(pos,pos)-L0**2 - L1**2)/(2*L0*L1))
            posp = pos - self.leg_origin - L0*np.array([np.cos(theta1),0,np.sin(theta1)])
            theta3 = -np.acos((np.linalg.norm(posp)-L2**2-L3**2)/(2 * L2 * L3))
            theta2 = np.atan2(posp[1], np.sqrt(posp[0]**2 + posp[2]**2)) + np.atan2(L3 * np.sin(theta3), L2 + L3*np.cos(theta3))

        print(np.rad2deg(theta1),np.rad2deg(theta2),np.rad2deg(theta3))

    def getEndPosition(self):
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
        
        H0 = H(RY(np.deg2rad(self.leg_origin_angle)), self.leg_origin)
        H1 = H(RY(np.deg2rad(self.motor1.getAngle())),LegConstants.FEMUR)
        H2 = H(RZ(np.deg2rad(self.motor2.getAngle())),LegConstants.TIBIA)
        H3 = H(RZ(np.deg2rad(self.motor3.getAngle())),LegConstants.TARSUS)
        return (H0 @ H1 @ H2 @ H3) @ np.array([[0,0,0,1]]).T
    
    def followTrajectory(self, traj:Trajectory):
        pass

def positionEval(theta1, theta2, theta3):
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
    vec0 = np.array([[0,0,0,1]]).T
    H0 = H(RY(np.deg2rad(theta1 + 60)), LegConstants.RIGHT1)
    p0 = (H0 @ vec0).T[0]
    H1 = H(RZ(np.deg2rad(-theta2)), LegConstants.FEMUR)
    p1 = (H0 @ H1 @ vec0).T[0]
    H2 = H(RZ(np.deg2rad(-theta3)), LegConstants.TIBIA)
    p2 = (H0 @ H1 @ H2 @ vec0).T[0]
    H3 = H(np.eye(3),LegConstants.TARSUS)
    p3 = ((H0 @ H1 @ H2 @ H3) @ vec0).T[0]
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    print(p0)    
    print(p1)    
    print(p2)    
    print(p3)    
    ax.scatter(0, 0, 0, c='red', marker='o')  
    ax.scatter(100, 0, 0, c='red', marker='o')  
    ax.scatter(0, 100, 0, c='red', marker='o')  
    ax.scatter(0, 0, 100, c='red', marker='o')  
    ax.scatter(-p0[2], -p0[0], p0[1], c='blue', marker='o')  
    ax.scatter(-p1[2], -p1[0], p1[1], c='green', marker='o')  
    ax.scatter(-p2[2], -p2[0], p2[1], c='orange', marker='o')  
    ax.scatter(-p3[2], -p3[0], p3[1], c='purple', marker='o')  

    # Set labels
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.view_init(elev=45, azim=180 )
    # Set title
    ax.set_title('leg points')

    # Show plot
    ax.set_xlim(-500,500)
    ax.set_ylim(-500,500)
    ax.set_zlim(-500,500)
    plt.show()
    return (H0 @ H1 @ H2 @ H3) @ vec0

def angleEval(pos):
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
    q = np.sqrt((pos[0]-LegConstants.RIGHT1[0,0])**2 + (pos[2]-LegConstants.RIGHT1[0,2])**2)
    theta0 = np.pi/3
    theta1 = np.acos(round((p**2 -L0**2 - q**2)/(2*L0*q),5))
    rvec = pos - (LegConstants.RIGHT1[0] + L1 * RY(theta0 + theta1) @ np.array([1,0,0]))
    rx = rvec[0]
    ry = rvec[1]
    rz = rvec[2]
    # ry = 0
    r = np.linalg.norm(rvec)
    # r = np.sqrt(rx**2 + ry**2 + rz**2)
    
    # print(np.linalg.norm(LegConstants.RIGHT1 + L1 * RY(theta0 + theta1) @ np.array([1,0,0])))
    # print((r**2-L2**2-L3**2)/(2 * L2 * L3))
    # s = -np.acos(round((r**2-L2**2-L3**2)/(2 * L2 * L3),5))
    s = -np.acos(round((r**2-L2**2-L3**2)/(2 * L2 * L3),9))
    theta3 = round(s - LegConstants.tarsus_angle,5)
    theta2 = round(np.atan2(ry, np.sqrt(rx**2 + rz**2)) - np.atan2(L3 * np.sin(s), L2 + L3*np.cos(s)),5)

    print(np.rad2deg(theta1),np.rad2deg(theta2),np.rad2deg(theta3))
    # return np.array([np.rad2deg(theta1),np.rad2deg(theta2),np.rad2deg(theta3)])
class LegConstants():
    # in mm
    FEMUR = np.array([[67.5,0,0]])
    TIBIA = np.array([[67.5,0,0]])
    TARSUS = np.array([[188.517,-39.055,0]])
    tarsus_angle = np.atan2(-39.055,188.517)
    origin_disp = 123.099
    RIGHT1 = np.array([[origin_disp*np.cos(np.pi/3),16,-origin_disp*np.sin(np.pi/3)]])
    RIGHT2 = np.array([[origin_disp,16,0]])
    RIGHT3 = np.array([[origin_disp*np.cos(np.pi/3),16,origin_disp*np.sin(np.pi/3)]])

    LEFT1 = np.array([[-origin_disp*np.cos(np.pi/3),16,-origin_disp*np.sin(np.pi/3)]])
    LEFT2 = np.array([[-origin_disp,16,0]])
    LEFT3 = np.array([[-origin_disp*np.cos(np.pi/3),16,origin_disp*np.sin(np.pi/3)]])