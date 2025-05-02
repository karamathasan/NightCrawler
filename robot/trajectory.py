from abc import ABC
import numpy as np

class Trajectory(ABC):
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def sample(t):
        pass

class LinearTrajectory(Trajectory):
    def __init__(self, start, end):
        super().__init__(start, end)


class CubicTrajectory(Trajectory):
    def __init__(self,start, end, c1):
        super().__init__(start, end)
        self.c1 = c1