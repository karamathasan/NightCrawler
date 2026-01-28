from DXLConfig import DXLConfig
from motor import Motor
# from robot_Drive import RobotDrive
from robot import Robot, RobotConstants

import time
import numpy as np
from leg import Leg

from trajectory import LinearTrajectory, QuadraticTrajectory
from leg_actions import FollowTrajectory, TranslateLeg
from motion_profile import LinearProfile, TrapezoidalProfile, CuberootProfile
from actions import ActionQueue, ActionGroup, RepeatingGroup, Wait, SequentialGroup

conf = DXLConfig("/dev/ttyACM0")
conf.open()
right1 = Leg(True, 1, Motor(1), Motor(2), Motor(3))
right2 = Leg(True, 2, Motor(4), Motor(5), Motor(6))
right3 = Leg(True, 3, Motor(7), Motor(8), Motor(9))

left1 = Leg(False, 1, Motor(10), Motor(11), Motor(12))
left2 = Leg(False, 2, Motor(13), Motor(14), Motor(15))
left3 = Leg(False, 3, Motor(16), Motor(17), Motor(18))

right1.setSpeeds(100,100,100)
right2.setSpeeds(100,100,100)
right3.setSpeeds(100,100,100)
left1.setSpeeds(100,100,100)
left2.setSpeeds(100,100,100)
left3.setSpeeds(100,100,100)

#   ZERO POSITION
def zero_legs():
    right1.setJoints(0,-0,0)
    right2.setJoints(0,-0,0)
    right3.setJoints(0,-0,0)

    left1.setJoints(0,-0,0)
    left2.setJoints(0,-0,0)
    left3.setJoints(0,-0,0)

#   REST POSITION
def lotus_rest():
    right1.setJoints(0,-80,-90)
    event = right2.setJoints(0,-80,-90)
    right3.setJoints(0,-80,-90)

    left1.setJoints(0,-80,-90)
    left2.setJoints(0,-80,-90)
    left3.setJoints(0,-80,-90)
    return event

#   STAND 
def stand(a1=0, a2=-10, a3=70):
    right1.setJoints(a1,a2,a3)
    event = right2.setJoints(a1,a2,a3)
    right3.setJoints(a1,a2,a3)

    left1.setJoints(a1,a2,a3)
    left2.setJoints(a1,a2,a3)
    left3.setJoints(a1,a2,a3)
    return event

# zero_legs()
# stand(0,-45,90)()
stand()()

queue = ActionQueue()

def RY(rad):
    return np.array([
        [np.cos(rad), 0, np.sin(rad)],
        [0,1,0],
        [-np.sin(rad),0, np.cos(rad)]
    ])

disp = 40
theta = np.deg2rad(45)


dur = 1
speed = 100

height = 40
out = 95
k = 13/7

right1toInit = FollowTrajectory(
    right1,
    LinearTrajectory(
        RobotConstants.RIGHT1START,
        RobotConstants.RIGHT1START + np.array([0,0,disp]),
        dur
    ),
    speed
)

right1return = FollowTrajectory(
    right1,
    QuadraticTrajectory(
        RobotConstants.RIGHT1START + np.array([0,0,-disp]),
        RobotConstants.RIGHT1START + np.array([0,0,disp]),
        RobotConstants.RIGHT1START + np.array([0,height,0]),
        dur
    ),
    speed
    # ignore_start=True
)

right1stance = FollowTrajectory(
    right1,
    LinearTrajectory(
        RobotConstants.RIGHT1START + np.array([0,0,-disp]),
        RobotConstants.RIGHT1START + np.array([0,0,disp]),
        dur
    ),
    speed
)

right1swing = FollowTrajectory(
    right1,
    QuadraticTrajectory(
        RobotConstants.RIGHT1START + np.array([0,0,disp]),
        RobotConstants.RIGHT1START + np.array([0,0,-disp]),
        RobotConstants.RIGHT1START + np.array([0,height,0]),
        dur
    ),
    speed
    # ignore_start=True
)


single_cycle = SequentialGroup(
    right1swing,
    right1stance
)

queue.push(Wait(1))
queue.push(right1toInit)
# queue.push(Wait(1))
queue.push(single_cycle)

# queue.clear()
while not queue.isEmpty() or queue.active is not None:
    if queue.active is None:
        action = queue.pop()
        queue.active = action
        action.init()
    else:
        if queue.active.isDone():
            queue.active.end()
            queue.active = None
        else:
            queue.active.execute()

conf.close()