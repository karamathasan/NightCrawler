from abc import ABC
import numpy as np
from motion_profile import LinearProfile, TrapezoidalProfile
'''
to follow a trajectory, we need to mathematically calculate a path, preferrably 
through combinations of bezier curves and lines or whatnot
one way the robot can follow these trajectories is by sampling points along the 
functions
to do this, we need the robot to sample a specific value that is a specific distance 
away from the previous value, or at least samples it consistently in time
if we know the total duration, we can then use our robot's clock speed to choose how
many times we should sample
we might also be able to calculate a minimum duration given some leg parameters
for now, ill just try to set it a few points in some time intervals at max speed
'''

# TODO: some trajectories with a certain profile may cause sample rushing, 
# where the leg can never reach its intended goal before a new goal is set
class Trajectory(ABC):
    def __init__(self, start:np.ndarray, end:np.ndarray, duration, profile=LinearProfile()):
        self.start = start
        self.end = end
        self.duration = duration
        self.profile = profile
        # self.speed = speed

    def sample(self,t) -> float:
        return 0.0
    
    def setProfile(self, profile):
        self.profile = profile

class LinearTrajectory(Trajectory):
    def __init__(self, start, end, duration, profile = LinearProfile()):
        super().__init__(start, end, duration, profile)

    def sample(self,t) -> np.ndarray:
        t = np.clip(t,0,1)
        t = self.profile.modSample(t)
        return (1-t) * self.start + t * self.end
    
class BezierTrajectory(Trajectory):
    def lerp(self,a,b,t):
        return t * b + (1-t) * a

class QuadraticTrajectory(BezierTrajectory):
    def __init__(self, start, end, c1:np.ndarray, duration, profile = LinearProfile()):
        super().__init__(start, end, duration, profile)
        self.c1 = c1

    def sample(self,t):
        t = np.clip(t,0,1)
        return self.lerp(
            self.lerp(self.start,self.c1,t),
            self.lerp(self.c1,self.end,t),
            t
        )
    
    def __str__(self):
        indent = " " * 4
        return f"Quadtric trajectory \n{indent}Start:{self.start}\n{indent}End:{self.end}\n{indent}Control:{self.c1}"

class CubicTrajectory(BezierTrajectory):
    def __init__(self,start, end, c1:np.ndarray, c2:np.ndarray, duration, profile = LinearProfile()):
        super().__init__(start, end, duration, profile)
        self.c1 = c1
        self.c2 = c2

    def sample(self,t):
        t = np.clip(t,0,1)
        return self.lerp(
            self.lerp(
                self.lerp(self.start,self.c1,t),
                self.lerp(self.c1,self.c2,t),
                t
            ),
            self.lerp(
                self.lerp(self.c1,self.c2,t),
                self.lerp(self.c2,self.end,t),
                t
            ),
            t
        )
