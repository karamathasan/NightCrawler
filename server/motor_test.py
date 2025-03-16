from DXLConfig import DXLConfig
from motor import Motor
import time
conf = DXLConfig("COM8")
conf.open()
# m = Motor(1)
# m2 = Motor(2)
m3 = Motor(3)
# m.enable_torque()
# m2.enable_torque()

m3.setBounds(-90,90)
# time.sleep(0.1)
# print(m3.bounds)
# print(m3.getBounds())

m3.setPosition(0)
m3.getPosition()
# m3.reset()
# m3.getPosition()

# m.disable_torque()
# m2.disable_torque()
conf.close()


