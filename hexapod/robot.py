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

            
        