from DXLConfig import DXLConfig
from motor import Motor
import time
from leg import Leg
conf = DXLConfig("/dev/ttyACM0")
conf.open()
print(conf.findIDs())

current = conf.findFirstID()
print(current)
new = 10

m = Motor(current)
answer = input(f"Reassigning motor {current} to {new}. Continue?")
if answer == "y":
    print("reassiging.")
    m.reset(new)
conf.close()


