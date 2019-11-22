from __future__ import print_function	

import odrive
from odrive.enums import*
import time
import math
import numpy as np

#system parameters

l1m = 3.07 #kg
l1com = 0.0729 #m from center of rotation
l1cpr = 910
l1reduction = 9
l1kv = 16
beta1 = 0.00001
serial1 = ""

l2m = 2.6 #kg was 1.85, moved up to compensate for wrist?
l2com = 0.1893 #m
l2cpr = 8192 
l2reduction = 6
l2kv = 100
beta2 = -0.000005
serial2 = ""


#calibration
od2 = odrive.find_any(serial_number=serial2)
print("calibrating odrive2")
od2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od2.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od2.axis0.controller.config.control_mode = 1
od2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("move arm to position")
time.sleep(2)

while True:
	#get joint angle and velocity
	pos2 = od2.axis0.encoder.pos_estimate

	vel2 = od2.axis0.encoder.vel_estimate


	#zero position is straight up
	theta2 = (2 * np.pi * pos2) / (l2cpr * l2reduction)
	force2 = (l2m*9.81*l2com*np.sin(theta) / l2reduction) + beta2*vel2

	currentSetpoint2 = (-1 * force2 * l2kv) / 8.27
	od2.axis0.controller.current_setpoint = currentSetpoint2

	measuredCurrent2 = od2.axis0.motor.current_meas_phB

	print("Theta2: ",theta2,"   Current Setpoint J2: ", currentSetpoint2, " Actual Current J2: ", measuredCurrent2)
	time.sleep(0.05)


