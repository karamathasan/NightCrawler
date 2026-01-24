import numpy as np
# from imu import IMUSensor
# from hexapod.robot_drive import GaitDrive
from actions import ActionQueue
from leg import Leg
from vision import Vision

from auto.fsm import FiniteStateMachine
from auto.state import State

import asyncio

'''
class for highest level robot control
'''
class Robot():
    def __init__(self, gait_drive, operated=False):
        self.operated = operated
        if not operated:
            self.fsm = FiniteStateMachine()

        self.running = True


        self.queue = ActionQueue()
        self.driver = gait_drive
        self.vision = Vision()

    def activate(self):
        self.running = True
        if not self.operated:
            self.fsm.current.init()

        while(self.running):
            if self.operated:
                pass
            else:
                self.fsm.update()
            pass

    def deactivate(self):
        self.running = False

    # create asynchronous logic loop
    # async def run(self):
    #     while self.running:
    #         pass

    def run(self):
        if not self.queue.isEmpty() or self.queue.active is not None:
            if self.queue.active is None:
                action = self.queue.pop()
                self.queue.active = action
                action.init()
            else:
                if self.queue.active.isDone():
                    self.queue.active.end()
                    self.queue.active = None
                else:
                    self.queue.active.execute()

class RobotConstants:
    LEGANGLES = (0,-10,70)
    RIGHT1START = np.array([ 158.59078135, -154.74423213, -275.68508234])
    RIGHT2START = np.array([ 318.04567543, -154.74423213, -0.49889572])
    RIGHT3START = np.array([ 159.45489408, -154.74423213, 275.18618663])
    LEFT1START = np.array([-159.45489408, -154.74423213, -275.18618663])
    LEFT2START = np.array([-318.04567543, -154.74423213,    0.49889572])
    LEFT3START = np.array([-158.59078135, -154.74423213,  275.68508234])
