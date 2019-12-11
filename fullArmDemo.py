from __future__ import print_function	

import odrive
from odrive.enums import*
import time
import math
import numpy as np

serial0 = "206A337B304B"
serial1 = "2084377D3548"
serial2 = "205737993548"

#calibration
od0 = odrive.find_any(serial_number=serial0)
print("calibrating j0")
od0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od0.axis0.controller.config.control_mode = 3
od0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("j0 calibrated")
time.sleep(1)

od1 = odrive.find_any(serial_number=serial1)
print("calibrating j1")
od1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od1.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od1.axis0.controller.config.control_mode = 3
od1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("j1 calibrated")
time.sleep(1)

od2 = odrive.find_any(serial_number=serial2)
print("calibrating j2")
od2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od2.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od2.axis0.controller.config.control_mode = 3
od2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("j2 calibrated")
time.sleep(1)

print("calibrating j3")
od2.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od2.axis1.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od2.axis1.controller.config.control_mode = 3
od2.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("j3 calibrated")
time.sleep(1)

j0set = 0
j1set = 0
j2set = 0
j3set = 0

j0max = 30
j1max = 120
j2max = 7000
j3max = -10000

count = 0

while True:

	if count < 3:
		j0set = j0set + j0max/3
		j1set = j1set + j1max/3
		j2set = j2set + j2max/3
		j3set = j3set + j3max/3
	if count >= 3:
		j0set = j0set - j0max/3
		j1set = j1set - j1max/3
		j2set = j2set - j2max/3
		j3set = j3set - j3max/3
	
	count = count + 1

	od0.axis0.controller.pos_setpoint = j0set
	od1.axis0.controller.pos_setpoint = j1set
	od2.axis0.controller.pos_setpoint = j2set
	od2.axis1.controller.pos_setpoint = j3set

	if count == 6:
		count = 0
	time.sleep(2)
	
