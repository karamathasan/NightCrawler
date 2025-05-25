from DXLConfig import DXLConfig
from motor import Motor
import time
import numpy as np
from leg import Leg
conf = DXLConfig("COM9")
conf.open()

mr1 = Motor(1)
mr2 = Motor(2)
mr3 = Motor(3)

mr1.reset()
mr2.reset()
mr3.reset()
right1 = Leg(True, 1, mr1, mr2, mr3)

# mr4 = Motor(4)
# mr5 = Motor(5)
# mr6 = Motor(6)
# right2 = Leg(True, 1, mr4, mr5, mr6)

# mr7 = Motor(7)
# mr8 = Motor(8)
# mr9 = Motor(9)
# right3 = Leg(True, 1, mr7, mr8, mr9)

# mr1.setSpeed(100)
# mr2.setSpeed(100)
# mr3.setSpeed(100)

# mr1.setAngle(45)
# mr2.setAngle(-45)
# mr3.setAngle(90)
right1.setJoints(0,0,0)
# init = right1.getEndPosition()

# time.sleep(2)

# mr1.setAngle(0)
# mr2.setAngle(0)
# mr3.setAngle(0)
# time.sleep(2)
# fin = right1.getEndPosition()

mr10 = Motor(10)
mr11 = Motor(11)
mr12 = Motor(12)
left1 = Leg(False, 1, mr10, mr11, mr12)
# w
# -1.3665164782359873 left
# -1.7750761753538058 right
# pos
# 
# [   4.37241263  -97.18799668 -319.99465628] right

# mr10.setAngle(45)
# mr11.setAngle(0)
# mr12.setAngle(0)

left1.setJoints(0,0,0)
end = left1.findEndPosition()
print(left1.solveEndPosition(end.T[0,0:3]))

# print(init)
# print(fin)
# [ 223.308       -23.055      -386.78080174]
# pos = np.array([fin[0,0], fin[1,0], fin[2,0]])
# print(right1.getEndPosition())
# angles = right1.setEndPosition(np.array([ 223.308, -23.055, -386.78080174]))
# angles = right1.setEndPosition(init.T[0,0:3])
# print(angles)
# right1.setJoints(*angles)
# pos = positionEval(0,-45,90)
# coord = pos.T[0,0:3]
# print(coord)
# print(-coord[2], -coord[0], coord[1])
# angleEval(pos.T[0,0:3])

conf.close()