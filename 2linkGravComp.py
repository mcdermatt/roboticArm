from __future__ import print_function	

import odrive
from odrive.enums import*
import time
import math
import numpy as np

#system parameters

#l1- shoulder up/down
l1m = 100 #kg was 3.07
l1com = 0.0729 #m from center of rotation
l1 = 0.1651
l1cpr = 910
l1reduction = 9
l1kv = 16
#beta1 = 0
beta1 = -0.00005
serial1 = "2084377D3548"

#l2- elbow
l2m = 2.6 #kg was 1.85, moved up to compensate for wrist?
l2com = 0.1893 #m
l2cpr = 8192 
l2reduction = 6
l2kv = 100
beta2 = -0.000005
serial2 = "205737993548"


#calibration
od2 = odrive.find_any(serial_number=serial2)
print("calibrating odrive2")
od2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od2.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od2.axis0.controller.config.control_mode = 1
od2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("move elbow to position")
time.sleep(2)

od1 = odrive.find_any(serial_number=serial1)
print("calibrating odrive1")
od1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od1.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od1.axis0.controller.config.control_mode = 1
od1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("move shoulder to position")
time.sleep(2)

theta2 = 0
theta1 = 0
theta2eff = 0

while True:
	#get joint angle and velocity
	pos2 = od2.axis0.encoder.pos_estimate
	vel2 = od2.axis0.encoder.vel_estimate
	pos1 = (-1)*od1.axis0.encoder.pos_estimate
	vel1 = od1.axis0.encoder.vel_estimate

	#zero position is straight up
	theta2 = (2 * np.pi * pos2) / (l2cpr * l2reduction)
	theta1 = (2 * np.pi * pos1) / (l1cpr * l1reduction)
	theta2eff = theta2 + theta1

	force2 = (l2m*9.81*l2com*np.sin(theta2eff) / l2reduction) + beta2*vel2

	currentSetpoint2 = (-1 * force2 * l2kv) / 8.27
	od2.axis0.controller.current_setpoint = currentSetpoint2

	time.sleep(0.05)

	measuredCurrent2 = od2.axis0.motor.current_meas_phB

	
	force1 = (l1m*9.81*l1com*np.sin(theta1)) + (l2m*9.81*(l1*np.sin(theta1)+l2com*np.sin(theta2eff))) + beta1*vel1
	currentSetpoint1 = (1 * l1kv*force1)/(8.27*l1reduction)
	measuredCurrent1 = od1.axis0.motor.current_meas_phB

	od1.axis0.controller.current_setpoint = currentSetpoint1


	error1 = od1.axis0.error


	print("Theta2eff: ",theta2eff,"   Theta1: ", theta1,"   CSJ2: ", currentSetpoint2, " CSJ1:  ", currentSetpoint1, " CMJ1: ", measuredCurrent1, error1)
 	


	time.sleep(0.05)

od1.reboot()
od2.reboot()

