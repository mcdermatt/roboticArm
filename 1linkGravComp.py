from __future__ import print_function	

import odrive
from odrive.enums import*
import time
import math
import numpy as np

l2m = 1.85 #kg
l2com = 0.1893 #m
l2cpr = 8192 
l2reduction = 6
l2kv = 100


#calibration
od = odrive.find_any()
print("calibrating odrive0")
od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL

od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

while True:
	#get joint angle
	pos = od.axis0.encoder.pos_estimate

	#zero position is straight up
	theta = (2 * np.pi * pos) / (l2cpr * l2reduction)
	force = l2m*9.81*l2com*np.sin(theta)

	currentSetpoint = (force * l2kv) / 8.27
	od.axis0.controller.current_setpoint = currentSetpoint

	measuredCurrent = od.axis0.motor.current_control.Iq_measured

	print("Theta: ",theta,"   Current Setpoint: ", currentSetpoint, " Actual Current: ", measuredCurrent)
	time.sleep(0.01)


