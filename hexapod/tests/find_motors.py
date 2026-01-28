from DXLConfig import DXLConfig, Addresses
from motor import Motor
import time
import numpy as np
from leg import Leg
conf = DXLConfig("/dev/ttyACM0")
conf.open()
conf.findIDs()
conf.close()