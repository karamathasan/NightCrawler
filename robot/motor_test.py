from DXLConfig import DXLConfig
from motor import Motor
import time
from leg import Leg
conf = DXLConfig("COM9")
conf.open()

mr1 = Motor(1)
mr2 = Motor(2)
mr3 = Motor(3)
right1 = Leg(True, mr1,mr2,mr3)

mr4 = Motor(4)
mr5 = Motor(5)
mr6 = Motor(6)
right2 = Leg(True,mr4,mr5,mr6)

mr7 = Motor(7)
mr8 = Motor(8)
mr9 = Motor(9)
right3 = Leg(True,mr7,mr8,mr9)

mr1.reset()
mr2.reset()
mr3.reset()

# print(m.getBounds())
# print(m2.getBounds())
# print(m3.getBounds())

# m.enable_torque()
# m2.enable_torque()
# m3.enable_torque()

# print(right1.getJointAngles())
time.sleep(2)



# print(right1.getJointAngles())

# print(m.getAngle())
# if m.getAngle() == 90:
#     m.setAngle(-90)
#     m2.setAngle(-90)
#     m3.setAngle(-90)
# else:
#     m.setAngle(90)
#     m2.setAngle(90)
#     m3.setAngle(90)

# m.setAngle(0)
# m2.setAngle(0)
# m3.setAngle(0)

m.disable_torque()
m2.disable_torque()
# m3.disable_torque()
conf.close()


