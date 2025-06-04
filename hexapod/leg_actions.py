import numpy as np
import time
from actions import ActionBase, ActionGroup
from leg import Leg
from trajectory import Trajectory, LinearTrajectory
from motion_profile import MotionProfile, LinearProfile, TrapezoidalProfile

class TranslateLeg(ActionBase):
    def __init__(self, leg:Leg, global_translation: np.ndarray, duration = 1, profile = LinearProfile(), maxspeed=1024):
        self.translation = global_translation
        assert(self.translation.shape == (3,))
        self.leg = leg

        self.duration = duration
        self.start = time.time()
        self.elapsed = 0

        self.profile = profile
        self.trajectory: LinearTrajectory = None
        self.maxspeed=maxspeed
        
    def init(self):
        self.trajectory = LinearTrajectory(self.leg.findEndPosition().T[0,:3], self.leg.findEndPosition().T[0,:3] + self.translation, self.duration, self.profile)
        self.start = time.time()

        dt = time.time() - self.start
        self.elapsed = dt

        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)

        phi1 = np.asarray(self.leg.getJointAngles())
        phi2 = np.asarray(self.leg.solveEndPosition(goal))

        angles = self.leg.solveEndPosition(goal)
        
        delta = np.abs(phi2 - phi1)

        relativeSpeeds = (delta/np.max(delta))
        self.leg.setSpeeds(*(np.int32(self.maxspeed * relativeSpeeds)))
        self.leg.setJoints(*angles)

    def execute(self):
        # print(f"Error: {self.leg.findEndPosition().T[0,:3] - self.trajectory.end }")
        dt = time.time() - self.elapsed
        self.elapsed = time.time() - self.start

        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)

        phi1 = np.asarray(self.leg.getJointAngles())
        phi2 = np.asarray(self.leg.solveEndPosition(goal))

        angles = self.leg.solveEndPosition(goal)
        
        delta = np.abs(phi2 - phi1)

        relativeSpeeds = (delta/np.max(delta))
        self.leg.setSpeeds(*(np.int32(self.maxspeed * relativeSpeeds)))
        self.leg.setJoints(*angles)

    def isDone(self):
        return self.elapsed/self.trajectory.duration >= 1
        # return np.allclose(self.leg.findEndPosition().T[0,:3], self.trajectory.end, atol=2)

    def getTrajectory(self):
        return self.trajectory


class FollowTrajectory(ActionBase):
    def __init__(self, leg:Leg, trajectory: Trajectory, maxspeed):
        self.leg = leg
        self.trajectory = trajectory

        self.elapsed = 0

        self.start = time.time()
        self.elapsed = 0
        self.maxspeed = maxspeed

    def init(self):
        # angles = [*self.leg.getJointAngles()]
        # goal = self.leg.solveEndPosition(self.trajectory.start)
        # print(angles)
        # print(goal)

        assert np.allclose(self.leg.findEndPosition().T[0,:3], self.trajectory.start, atol=2.3), f"Invalid start, error: {self.leg.findEndPosition().T[0,:3] - self.trajectory.start} for Leg {self.leg.leg_id}. Right?: {self.leg.right}"
        self.start = time.time()
        dt = time.time() - self.start
        self.elapsed = dt

        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)

        phi1 = np.asarray(self.leg.getJointAngles())
        phi2 = np.asarray(self.leg.solveEndPosition(goal))

        angles = self.leg.solveEndPosition(goal)
        
        delta = np.abs(phi2 - phi1)

        relativeSpeeds = (delta/np.max(delta))
        self.leg.setSpeeds(*(np.int32(self.maxspeed * relativeSpeeds)))
        self.leg.setJoints(*angles)
    
    def execute(self):
        dt = time.time() - self.start
        self.elapsed = time.time() - self.start

        goal = self.trajectory.sample(self.elapsed/self.trajectory.duration)

        phi1 = np.asarray(self.leg.getJointAngles())
        phi2 = np.asarray(self.leg.solveEndPosition(goal))

        angles = self.leg.solveEndPosition(goal)
        
        delta = np.abs(phi2 - phi1)

        relativeSpeeds = (delta/np.max(delta))
        self.leg.setSpeeds(*(np.int32(self.maxspeed * relativeSpeeds)))
        self.leg.setJoints(*angles)

    def isDone(self):
        return self.elapsed/self.trajectory.duration >= 1
        # return np.allclose(self.leg.findEndPosition().T[0,:3], self.trajectory.end, atol=2.5)