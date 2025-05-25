from DXLConfig import DXLConfig
from motor import Motor
# from robot_Drive import RobotDrive
# from robot import Robot

import time
import numpy as np
from leg import Leg

from trajectory import LinearTrajectory
from leg import FollowTrajectory, TranslateLeg
from actions import ActionQueue

conf = DXLConfig("COM9")
conf.open()
right1 = Leg(True, 1, Motor(1), Motor(2), Motor(3))
right2 = Leg(True, 2, Motor(4), Motor(5), Motor(6))
right3 = Leg(True, 3, Motor(7), Motor(8), Motor(9))

left1 = Leg(False, 1, Motor(10), Motor(11), Motor(12))
left2 = Leg(False, 2, Motor(13), Motor(14), Motor(15))
left3 = Leg(False, 3, Motor(16), Motor(17), Motor(18))

# right1.setSpeeds(100,100,100)
# right2.setSpeeds(100,100,100)
# right3.setSpeeds(100,100,100)
# left1.setSpeeds(100,100,100)
# left2.setSpeeds(100,100,100)
# left3.setSpeeds(100,100,100)

right1.setSpeeds(300,300,300)
right2.setSpeeds(300,300,300)
right3.setSpeeds(300,300,300)
left1.setSpeeds(300,300,300)
left2.setSpeeds(300,300,300)
left3.setSpeeds(300,300,300)

# robot = Robot(GaitDrive([left1,left2,left3],[right1,right2,right3]), operated=True)

# queue = ActionQueue()
# queue.push(TranslateLeg(right2, np.array([0,-90,0]), 1))
# queue.push(TranslateLeg(right1, np.array([0,70,0])))

#   ZERO POSITION
# right1.setJoints(0,-0,0)
# right2.setJoints(0,-0,0)
# right3.setJoints(0,-0,0)

# left1.setJoints(0,-0,0)
# left2.setJoints(0,-0,0)
# left3.setJoints(0,-0,0)

# time.sleep(2)

#   REST POSITION
# right1.setJoints(0,-70,-90)
# right2.setJoints(0,-70,-90)
# right3.setJoints(0,-70,-90)

# left1.setJoints(0,-70,-90)
# left2.setJoints(0,-70,-90)
# left3.setJoints(0,-70,-90)


# print(right2.solveEndPosition(right2.findEndPosition().T[0,:3]))
time.sleep(1)

#   STAND 
# right1.setJoints(0,-90,-45)
# right2.setJoints(0,-90,-45)
# right3.setJoints(0,-90,-45)

# left1.setJoints(0,-90,-45)
# left2.setJoints(0,-90,-45)
# left3.setJoints(0,-90,-45)

queue=ActionQueue()
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