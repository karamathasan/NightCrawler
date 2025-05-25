from motor import Motor
from trajectory import Trajectory, LinearTrajectory
from enum import Enum
from actions import ActionBase, ActionGroup
import numpy as np
import time
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
        if self.right:
            self.motor1.setAngle(theta1)
            self.motor2.setAngle(theta2)
            self.motor3.setAngle(theta3)
        else:
            self.motor1.setAngle(-theta1)
            self.motor2.setAngle(-theta2)
            self.motor3.setAngle(-theta3)

    def setSpeeds(self, byte1=0, byte2=0, byte3=0):
        self.motor1.setSpeed(byte1)
        self.motor2.setSpeed(byte2)
        self.motor3.setSpeed(byte3)

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
        theta1 = np.acos(round((p**2 -L0**2 - q**2)/(2*L0*q),5))
        if self.right:
            rvec = pos - (self.leg_origin[0] + L1 * RY(theta0 + theta1) @ np.array([1,0,0]))
            
        else:
            rvec = pos - (self.leg_origin[0] + L1 * RY(theta0 - theta1) @ np.array([1,0,0]))
        # print(pos)
        rx = rvec[0]
        ry = rvec[1]
        rz = rvec[2]
        r = np.linalg.norm(rvec)
        
        # print(rvec)
        s = -np.acos(round((r**2-L2**2-L3**2)/(2 * L2 * L3),9))
        # print(s)
        theta3 = round(s - LegConstants.tarsus_angle,5)
        theta2 = round(np.atan2(ry, np.sqrt(rx**2 + rz**2)) - np.atan2(L3 * np.sin(s), L2 + L3*np.cos(s)),5)
        # should only work for int type
        return round(np.rad2deg(theta1), 5), -round(np.rad2deg(theta2), 5), -round(np.rad2deg(theta3), 5)

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
    
    def followTrajectory(self, traj:Trajectory):
        pass

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

class TranslateLeg(ActionBase):
    def __init__(self, leg:Leg, global_translation: np.ndarray, duration = 1):
        self.translation = global_translation
        assert(self.translation.shape == (3,))
        self.leg = leg

        self.duration = duration
        self.start = time.time()
        self.elapsed = 0

        self.trajectory: LinearTrajectory = None
        self.done = False
        # self.trajectory = LinearTrajectory(leg.findEndPosition().T[0,:3], leg.findEndPosition().T[0,:3] + self.translation, self.duration)
        
    def init(self):
        self.start = time.time()
        self.trajectory = LinearTrajectory(self.leg.findEndPosition().T[0,:3], self.leg.findEndPosition().T[0,:3] + self.translation, self.duration)
        print("start: " + str(self.trajectory.start))
        print("end: " + str(self.trajectory.end))

        # dt = time.time() - self.start
        dt = time.time() - self.start
        self.elapsed = dt
        # self.elapsed += dt

        print(self.elapsed/self.trajectory.duration)
        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)
        print("goal: " + str(goal))
        # phi1 = self.leg.findEndPosition()
        # phi2 = self.leg.solveEndPosition(goal)

        angles = self.leg.solveEndPosition(goal)
        print(angles)
        self.leg.setJoints(*angles)

    def execute(self):
        # print(self.elapsed)

        dt = time.time() - self.elapsed
        self.elapsed = time.time()-self.start

        # print(f"{self.elapsed/self.trajectory.duration}")
        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)

        phi1 = np.asarray(self.leg.getJointAngles())
        phi2 = np.asarray(self.leg.solveEndPosition(goal))

        angles = self.leg.solveEndPosition(goal)
        
        delta = np.abs(phi2 - phi1)

        maxspeed = 300
        relativeSpeeds = (delta/np.max(delta))
        self.leg.setSpeeds(*(np.int32(maxspeed * relativeSpeeds)))
        self.leg.setJoints(*angles)

    def moveToSample(self):
        dt = time.time() - self.start
        self.elapsed += dt

        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)

        # phi1 = np.asarray(self.leg.getJointAngles())
        # phi2 = np.asarray(self.leg.solveEndPosition(goal))

        angles = self.leg.solveEndPosition(goal)

        # delta = phi2 - phi1
        # relativeSpeeds = delta/np.max(delta)
        
        # self.leg.setSpeeds(*(self.max_speed * relativeSpeeds))
        self.leg.setJoints(*angles)

    def isDone(self):
        # return self.done
    
        # return not self.leg.isMoving()
        # print("     Is Done?")
        # print(self.trajectory.end)
        # print(self.leg.findEndPosition().T[0,:3])

        return np.allclose(self.leg.findEndPosition().T[0,:3], self.trajectory.end, atol=0.7)
        
        # return np.all(self.leg.findEndPosition() == self.trajectory.end)

    def end(self):
        print(self.trajectory.end)
        print(self.leg.findEndPosition().T[0,:3])


class FollowTrajectory(ActionBase):
    def __init__(self, leg:Leg, trajectory: Trajectory):
        self.trajectory = trajectory
        self.elapsed = 0
        self.max_speed = 1024
        self.leg = leg
        # startAngles = leg.setEndPosition(trajectory.start)
        # endAngles = leg.setEndPosition(trajectory.end)
        # angleDiff = endAngles-startAngles

        # maxAngle = np.max(angleDiff)
        # relativeSpeeds = angleDiff/maxAngle

    def init(self):
        self.start = time.time()
        return 
    
    def execute(self):
        dt = time.time() - self.start
        self.elapsed += dt

        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)

        phi1 = self.leg.findEndPosition()
        phi2 = self.leg.solveEndPosition(goal)

        delta = phi2 - phi1
        relativeSpeeds = delta/np.max(delta)
        
        self.leg.setSpeeds(*(self.max_speed * relativeSpeeds))
        # return super().execute(dt)

    def isDone(self):
        # return self.leg.isMoving()
        return self.leg.findEndPosition() == self.trajectory.end