from DXLConfig import DXLConfig
from motor import Motor, MotorAction
import time
from leg import Leg

from actions import ActionQueue, ActionGroup
conf = DXLConfig("COM9")
conf.open()
m = Motor(1)
m2 = Motor(2)
m3 = Motor(3)
right1 = Leg(True, m,m2,m3)

m.reset()
m2.reset()
m3.reset()

m.setAngle(0)
m2.setAngle(0)
m3.setAngle(0)

time.sleep(0.5)

m.setSpeed(100)
m2.setSpeed(100)
m3.setSpeed(100)

m.enable_torque()
m2.enable_torque()
m3.enable_torque()

queue = ActionQueue()
# queue.push(MotorAction(m,45))
# queue.push(MotorAction(m2,-45))
# queue.push(MotorAction(m3,90))

queue.push(ActionGroup(
    MotorAction(m,45),
    MotorAction(m2,-45),
    MotorAction(m3,90)   
))

while not queue.isEmpty():
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

m.disable_torque()
m2.disable_torque()
m3.disable_torque()
conf.close()


