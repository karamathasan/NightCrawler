import types
import numpy as np
class MotionProfile():
    def __init__(self):
        self.profiling_function: types.FunctionType

    def modSample(self,t):
        return self.profiling_function(t)
    
class LinearProfile(MotionProfile):
    def __init__(self):
        self.profiling_function = lambda x:x

class TrapezoidalProfile(MotionProfile):
    def __init__(self):
        self.profiling_function = lambda x: np.cbrt(x-0.5)/np.cbrt(4)+0.5

    