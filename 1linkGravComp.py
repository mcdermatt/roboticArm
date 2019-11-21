import odrive
from odrive.enums import*
from __future__ import print_function
import time
import math
import numpy as np

l2m = 6.5 #kg


#calibration
od = odrive.find_any()
print("calibrating odrive0")
od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od.axis0.current_state != AXIS_STATE_IDLE
	time.sleep(0.1)

od.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL

od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

while True:
	pos = od.axis0.encoder.pos_estimate
