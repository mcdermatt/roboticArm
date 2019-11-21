import odrive
from odrive.enums import*
from __future__ import print_function
import time
import math
import numpy as np

od = odrive.find_any()

#calibration
print("calibrating odrive0")
od.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od.axis1.current_state != AXIS_STATE_IDLE
	time.sleep(0.1)

