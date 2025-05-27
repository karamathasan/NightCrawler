from DXLConfig import DXLConfig
from motor import Motor
# from robot_Drive import RobotDrive
# from robot import Robot

import time
import numpy as np
from leg import Leg

from trajectory import LinearTrajectory, QuadraticTrajectory
from leg import FollowTrajectory, TranslateLeg
from motion_profile import LinearProfile, TrapezoidalProfile, CuberootProfile
from actions import ActionQueue, ActionGroup, Wait

conf = DXLConfig("COM9")
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
# time.sleep(2)

#   REST POSITION
def lotus_rest():
    right1.setJoints(0,-70,-90)
    right2.setJoints(0,-70,-90)
    right3.setJoints(0,-70,-90)

    left1.setJoints(0,-70,-90)
    left2.setJoints(0,-70,-90)
    left3.setJoints(0,-70,-90)

#   STAND 
def stand():
    right1.setJoints(0,-35,90)
    event = right2.setJoints(0,-35,90)
    right3.setJoints(0,-35,90)

    left1.setJoints(0,-35,90)
    left2.setJoints(0,-35,90)
    left3.setJoints(0,-35,90)
    return event

def half_stance():
    event = right1.setJoints(0.14662756598241344, -35.92375366568915, 90.1759530791789)
    right2.setJoints(20.087976539589448, -30.93841642228739, 83.13782991202345)
    right3.setJoints(-8.064516129032256, -6.011730205278582, 42.96187683284458)

    left1.setJoints(8.064516129032256, -6.011730205278582, 42.96187683284458)
    left2.setJoints(-20.087976539589448, -30.93841642228739, 83.13782991202346)
    left3.setJoints(-0.14662756598241344, -35.92375366568916, 90.1759530791789)
    return event
# zero_legs()
lotus_rest()
# stand()()

# right2.setJoints(30,-30,30)()
# print(right2.solveEndPosition(right2.findEndPosition().T[0,:3]))
# right2.setJoints(-30,-30,30)()
# print(right2.solveEndPosition(right2.findEndPosition().T[0,:3]))

# right2.setJoints(30,-20,80)

# time.sleep(1.5)

queue = ActionQueue()
# start = right2.findEndPosition().T[0,:3]
# print(start)
# c1 = start + np.array([60,30,0])
# end = start + np.array([70,220,0])
# qt = QuadraticTrajectory(start,end,c1,1)
# queue.push(FollowTrajectory(right2, qt))

disp = 70
dur = 0.3

# queue.push(
#     ActionGroup(
#         TranslateLeg(right1, np.array([0,0,-disp]), dur),
#         TranslateLeg(left3, np.array([0,0,disp]), dur)
#     )
# )
# queue.push(Wait(1))
# queue.push(
#     ActionGroup(
#         ActionGroup(
#             TranslateLeg(right1, np.array([0,0,disp]), dur),
#             TranslateLeg(left2, np.array([0,0,disp]), dur),
#             TranslateLeg(right3, np.array([0,0,disp]), dur)
#         ),
#         ActionGroup(
#             TranslateLeg(left1, np.array([0,0,-disp]), dur),
#             TranslateLeg(right2, np.array([0,0,-disp]), dur),
#             TranslateLeg(left3, np.array([0,0,-disp]), dur)
#         )
#     )
# )

# queue.push(
#     ActionGroup(
#         ActionGroup(
#             TranslateLeg(right1, np.array([0,0,-disp]), 0.5),
#             TranslateLeg(left2, np.array([0,0,-disp]), 0.5),
#             TranslateLeg(right3, np.array([0,0,-disp]), 0.5)
#         ),
#         ActionGroup(
#             TranslateLeg(left1, np.array([0,0,disp]), 0.5),
#             TranslateLeg(right2, np.array([0,0,disp]), 0.5),
#             TranslateLeg(left3, np.array([0,0,disp]), 0.5)
#         )
#     )
# )
# queue.push()
# queue.push()
# queue.push(TranslateLeg(right2, np.array([0,-70,0]), 1, TrapezoidalProfile()))


print(f"""
    event = right1.setJoints({right1.getJointAngles()})
    right2.setJoints({right2.getJointAngles()})
    right3.setJoints({right3.getJointAngles()})

    left1.setJoints({left1.getJointAngles()})
    left2.setJoints({left2.getJointAngles()})
    left3.setJoints({left3.getJointAngles()})
    return event
""")
queue.clear()
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

# print(right2.findEndPosition().T[0,:3])
# print(right2.solveEndPosition(right2.findEndPosition().T[0,:3]))
# time.sleep(1)

# right1.setJoints(0,0,0)
# right2.setJoints(0,0,0)
# right3.setJoints(0,0,0)

# left1.setJoints(0,0,0)
# left2.setJoints(0,0,0)
# left3.setJoints(0,0,0)

conf.close()