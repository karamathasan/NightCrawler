from DXLConfig import DXLConfig
from motor import Motor
import time
from leg import Leg
conf = DXLConfig("COM9")
conf.open()
m = Motor(1)
m2 = Motor(2)
m3 = Motor(3)
# m.enable_torque()
# m2.enable_torque()
# m3.enable_torque()

m.reset()
m2.reset()
m3.reset()

right1 = Leg(True, m,m2,m3)
right1.setJ1(45)
right1.setJ2(45)
right1.setJ3(-90)

time.sleep(1)

right1.setJ1(0)
right1.setJ2(0)
right1.setJ3(0)

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

# m.disable_torque()
# m2.disable_torque()
# m3.disable_torque()
conf.close()


