from DXLConfig import DXLConfig, Addresses
from motor import Motor
import time
import numpy as np
from leg import Leg
conf = DXLConfig("COM9")
conf.open()

conf.findIDs()

Motor(1).setAngle(0)

conf.close()