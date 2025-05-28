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
    def __init__(self, ramp_up = 0.5):#ramp up is the time it spends accelerating
        #the ramp up time is the time spent in the leading edge of the trapezoid
        assert 0 <= ramp_up <= 0.5, "INVALID RAMP UP TIME"
        self.ramp_up = ramp_up
        self.profiling_function = lambda x:self.trapezoidalMod(x)

    def trapezoidalMod(self,t):
        amplitude = 1/(1-self.ramp_up)
        if t < self.ramp_up:
            return (amplitude/self.ramp_up) * (t**2/2)
        elif self.ramp_up <= t < 1-self.ramp_up:
            return amplitude*(t-0.5)+0.5
        elif 1-self.ramp_up <= t <= 1:
            return  -(amplitude/self.ramp_up) * (t-1)**2/2+1
        else:
            return t

class CuberootProfile(MotionProfile):
    def __init__(self):
        self.profiling_function = lambda x: np.cbrt(x-0.5)/np.cbrt(4)+0.5

    