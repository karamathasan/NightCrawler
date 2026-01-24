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
from actions import ActionQueue, ActionGroup, RepeatingGroup, Wait

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
def stand():
    right1.setJoints(0,-10,70)
    event = right2.setJoints(0,-10,70)
    right3.setJoints(0,-10,70)

    left1.setJoints(0,-10,70)
    left2.setJoints(0,-10,70)
    left3.setJoints(0,-10,70)
    return event

# zero_legs()
# lotus_rest()()
stand()()
# left1.setJoint2(-55)

# time.sleep(1)

queue = ActionQueue()

disp = 70
dur = 0.75
speed = 100

height = 120
out = 95
k = 13/7
# RIGHT1 STEP
# queue.push(
#     RepeatingGroup(
#         FollowTrajectory(
#             right1,
#             QuadraticTrajectory(
#                 right1.findEndPosition().T[0,:3],
#                 right1.findEndPosition().T[0,:3] + np.array([0,0,-disp]),
#                 right1.findEndPosition().T[0,:3] + np.array([0,height,-k*disp]),
#                 dur
#             ),
#             speed
#         ),
#         TranslateLeg(right1, np.array([0,0,disp]), dur, maxspeed=speed),
#     )
# )

# LEFT1 STEP
# queue.push(
#     RepeatingGroup(
#         FollowTrajectory(
#             left1,
#             QuadraticTrajectory(
#                 left1.findEndPosition().T[0,:3],
#                 left1.findEndPosition().T[0,:3] + np.array([0,0,-disp]),
#                 left1.findEndPosition().T[0,:3] + np.array([0,35,-k*disp/2]),
#                 dur
#             ),
#             speed
#         ),
#         TranslateLeg(left1, np.array([0,0,disp]), dur, maxspeed=speed),
#     )
# )

# queue.push(
#             TranslateLeg(right2, np.array([0,0,-disp/2]), dur),
# )

# HARDWARE BUG FOR LEG 1 
# LEFT FORWARD CONFIG
# INIT CONFIG
right1toInit = FollowTrajectory(
    right1,
    QuadraticTrajectory(
        right1.findEndPosition().T[0,:3],
        right1.findEndPosition().T[0,:3] + np.array([0,0,disp]),
        right1.findEndPosition().T[0,:3] + np.array([0,height,k*disp]),
        dur
    ),
    speed
)


left3toInit = FollowTrajectory(
    left3,
    QuadraticTrajectory(
        left3.findEndPosition().T[0,:3],
        left3.findEndPosition().T[0,:3] + np.array([0,0,-disp]),
        left3.findEndPosition().T[0,:3] + np.array([0,height,-k*disp]),
        dur
    ),
    speed
)

queue.push(
    ActionGroup(
        ActionGroup(
            # TranslateLeg(right1, np.array([0,0,disp]), dur),
            # right1toInit,
            TranslateLeg(left2, np.array([0,0,disp/2]), dur),
            TranslateLeg(right3, np.array([0,0,disp]), dur)
        ),
        ActionGroup(
            TranslateLeg(left1, np.array([0,0,-disp]), dur),
            TranslateLeg(right2, np.array([0,0,-disp/2]), dur),
            # left3toInit
            # TranslateLeg(left3, np.array([0,0,-disp]), dur)
        )
    )
)
# queue.push(TranslateLeg(right1, np.array([0,0,-disp]), dur))
queue.push(Wait(1))

# IGNORING LEFT 1
leftReturn = ActionGroup(
    # FollowTrajectory(
    #     left1,
    #     QuadraticTrajectory(
    #         left1.findEndPosition().T[0,:3],
    #         left1.findEndPosition().T[0,:3] + np.array([0,0,-disp]),
    #         left1.findEndPosition().T[0,:3] + np.array([0,height,-disp/2]),
    #         dur
    #     ),
    #     speed
    # ),
    FollowTrajectory(
        right2,
        QuadraticTrajectory(
            right2.findEndPosition().T[0,:3] + np.array([0,0,disp/2]),
            right2.findEndPosition().T[0,:3] + np.array([0,0,-disp/2]),
            right2.findEndPosition().T[0,:3] + np.array([out,height,0]),
            dur
        ),
        speed
    ),
    FollowTrajectory(
        left3,
        QuadraticTrajectory(
            left3.findEndPosition().T[0,:3] + np.array([0,0,disp]),
            left3.findEndPosition().T[0,:3],
            left3.findEndPosition().T[0,:3] + np.array([0,height, k * disp]),
            dur
        ),
        speed
    )
)

rightReturn = ActionGroup(
    FollowTrajectory(
        right1,
        QuadraticTrajectory(
            right1.findEndPosition().T[0,:3],
            right1.findEndPosition().T[0,:3] + np.array([0,0,-disp]),
            right1.findEndPosition().T[0,:3] + np.array([0,height,-k * disp]),
            dur
        ),
        speed
    ),
    FollowTrajectory(
        left2,
        QuadraticTrajectory(
            left2.findEndPosition().T[0,:3] + np.array([0,0,disp/2]),
            left2.findEndPosition().T[0,:3] + np.array([0,0,-disp/2]),
            left2.findEndPosition().T[0,:3] + np.array([-out,height,0]),
            dur
        ),
        speed
    ),
    FollowTrajectory(
        right3,
        QuadraticTrajectory(
            right3.findEndPosition().T[0,:3] + np.array([0,0,disp]),
            right3.findEndPosition().T[0,:3],
            right3.findEndPosition().T[0,:3] + np.array([0,height,k * disp]),
            dur
        ),
        speed
    )
)

leftPower = ActionGroup(
    # TranslateLeg(left1, np.array([0,0,disp]),dur),
    TranslateLeg(right2, np.array([0,0,disp]),dur),
    TranslateLeg(left3, np.array([0,0,disp]),dur),
)

rightPower = ActionGroup(
    TranslateLeg(right1, np.array([0,0,disp]),dur),
    TranslateLeg(left2, np.array([0,0,disp]),dur),
    TranslateLeg(right3, np.array([0,0,disp]),dur)
)

# queue.push(
#     RepeatingGroup(
#         TranslateLeg(left1, np.array([0,0,disp]),dur),
#         FollowTrajectory(
#             left1,
#             QuadraticTrajectory(
#                 left1.findEndPosition().T[0,:3],
#                 left1.findEndPosition().T[0,:3] + np.array([0,0,-disp]),
#                 left1.findEndPosition().T[0,:3] + np.array([0,height,-disp/2]),
#                 dur
#             ),
#             speed
#         ),
#     )
# )

# walk cycle
queue.push(Wait(1))
queue.push(
    RepeatingGroup(
        ActionGroup(
            rightReturn,
            leftPower
        ),
        ActionGroup(
            rightPower,
            leftReturn
        )
    )
)

# print(f"{right1.getJointAngles()}")
# print(f"{left1.getJointAngles()}")
# print(f"""
# right:
#     {right1.findEndPosition().T[0,:3]}
#     {right2.findEndPosition().T[0,:3]}
#     {right3.findEndPosition().T[0,:3]}
# left:
#     {left1.findEndPosition().T[0,:3]}
#     {left2.findEndPosition().T[0,:3]}
#     {left3.findEndPosition().T[0,:3]} 
#       """)
# print(f"""
# right:
#     {right1.findEndPosition((0,-10,70)).T[0,:3]}
#     {right2.findEndPosition((0,-10,70)).T[0,:3]}
#     {right3.findEndPosition((0,-10,70)).T[0,:3]}
# left:
#     {left1.findEndPosition((0,-10,70)).T[0,:3]}
#     {left2.findEndPosition((0,-10,70)).T[0,:3]}
#     {left3.findEndPosition((0,-10,70)).T[0,:3]} 
#       """)
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

conf.close()