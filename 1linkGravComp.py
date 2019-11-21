from __future__ import print_function	

import odrive
from odrive.enums import*
import time
import math
import numpy as np

l2m = 2.6 #kg was 1.85, moved up to compensate?
l2com = 0.1893 #m
l2cpr = 8192 
l2reduction = 6
l2kv = 100
beta = -0.000005


#calibration
od = odrive.find_any()

#print("rebooting")

print("calibrating odrive0")
od.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od.axis0.controller.config.control_mode = 1

od.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("move arm to position")
time.sleep(2)

while True:
	#get joint angle and velocity
	pos = od.axis0.encoder.pos_estimate

	vel = od.axis0.encoder.vel_estimate


	#zero position is straight up
	theta = (2 * np.pi * pos) / (l2cpr * l2reduction)
	force = (l2m*9.81*l2com*np.sin(theta) / l2reduction) + beta*vel

	currentSetpoint = (-1 * force * l2kv) / 8.27
	od.axis0.controller.current_setpoint = currentSetpoint

	measuredCurrent = od.axis0.motor.current_meas_phB

	print("Theta: ",theta,"   Current Setpoint: ", currentSetpoint, " Actual Current: ", measuredCurrent)
	time.sleep(0.05)


