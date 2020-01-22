from __future__ import print_function
import odrive
from odrive.enums import*
import time
import math
import numpy as np
import keyboard
import matplotlib.pyplot as plt

#system parameters

#l0- shoulder side/side
serial0 = "206A337B304B"
l0cpr = 90
l0reduction = 6

#l1- shoulder up/down
l1m = 3.07 #kg was 3.07
l1com = 0.0729 #m from center of rotation
l1 = 0.1651
l1cpr = 90
l1reduction = 6 #was 9, switched to og opentorque for less friction
l1kv = 16
#beta1 = 0
beta1 = -0.025 #to do- make beta a function of motor velocity to cancel out inertia? was -0.025 with 9:1
serial1 = "2084377D3548"

#l2- elbow
l2m = 3.1 #kg was 1.85, moved up to compensate for wrist? was 2.6 before end effector
l2com = 0.30 #m was 0.1893 moved up to compensate for wrist and end effector
l2cpr = 8192 
l2reduction = 6
l2kv = 100
beta2 = -0.000005
serial2 = "205737993548"


#calibration
od0 = odrive.find_any(serial_number=serial0)
print("calibrating j0")
od0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od0.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od0.axis0.controller.config.control_mode = 1
od0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("rotate shoulder away from work station")
time.sleep(1)
j0offset = od0.axis0.encoder.pos_estimate

od2 = odrive.find_any(serial_number=serial2)
print("calibrating odrive2")
od2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od2.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od2.axis0.controller.config.control_mode = 1
od2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("move elbow to vertical position")
time.sleep(2)
j2offset = od2.axis0.encoder.pos_estimate


od1 = odrive.find_any(serial_number=serial1)
print("calibrating odrive1")
od1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while od1.axis0.current_state != AXIS_STATE_IDLE:
	time.sleep(0.1)

od1.axis0.controller.config.control_mode = 1
od1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("move shoulder to vertical position")
time.sleep(2)
j1offset = od1.axis0.encoder.pos_estimate

theta2 = 0
theta1 = 0
theta2eff = 0

recording = 0

print("press q to teach sequence. press w to stop teaching sequence. press f to quit")
recording = 0
simulating = 0

while True:
	#get joint angle and velocity
	pos2 = od2.axis0.encoder.pos_estimate - j2offset
	vel2 = od2.axis0.encoder.vel_estimate
	pos1 = od1.axis0.encoder.pos_estimate - j1offset
	vel1 = od1.axis0.encoder.vel_estimate
	pos0 = od0.axis0.encoder.pos_estimate - j0offset
	vel0 = od0.axis0.encoder.vel_estimate
	
	#zero position is straight up
	theta2 = (2 * np.pi * pos2) / (l2cpr * l2reduction)
	theta1 = (2 * np.pi * pos1) / (l1cpr * l1reduction)
	theta0 = (2 * np.pi * pos0) / (l0cpr * l0reduction)
	theta2eff = theta2 + theta1

	force2 = (l2m*9.81*l2com*np.sin(theta2eff) / l2reduction) + beta2*vel2
	currentSetpoint2 = (-1 * force2 * l2kv) / 8.27
	od2.axis0.controller.current_setpoint = currentSetpoint2
	time.sleep(0.01)
	measuredCurrent2 = od2.axis0.motor.current_meas_phB
	
	force1 = (l1m*9.81*l1com*np.sin(theta1)) + (l2m*9.81*(l1*np.sin(theta1)+l2com*np.sin(theta2eff))) + beta1*vel1
	currentSetpoint1 = (-1 * l1kv*force1)/(8.27*l1reduction)
	measuredCurrent1 = od1.axis0.motor.current_control.Iq_measured
	od1.axis0.controller.current_setpoint = currentSetpoint1
	error1 = od1.axis0.error
	#print("Theta2eff: ",theta2eff,"   Theta1: ", theta1," CSJ2: ", currentSetpoint2, " CSJ1: ", currentSetpoint1, " CMJ1: ", measuredCurrent1, error1)
	
	#manually moving arm through range of motion to be recorded
	if recording == 1:
		#arr = [[theta0, theta1, theta2]]
		loadedArray = np.genfromtxt('armPath.txt',delimiter=" ")
		currPos = [[theta0,theta1,theta2]]
		print(currPos)
		loadedArray = np.append(loadedArray,currPos,axis=0)
		np.savetxt('armPath.txt',loadedArray)

	if simulating == 1:

		#calculate x y and z elbow and wrist
		l1sim = 0.40625*25.4 # upper arm
		l2sim = 0.59375*25.4 # lower arm l1+l2=1, easiest if upper and lower arm are same length
		#l3sim = 0.375*25.4 #wrist to end effector
		xElbow = ( l1sim * np.cos(theta0)*np.sin(theta1))
		yElbow = ( l1sim * np.sin(theta0)*np.sin(theta1))
		zElbow = ( l1sim * np.cos(theta1))

		xWrist = xElbow + l2sim*np.cos(theta1-theta2)*np.cos(theta0)
		yWrist = yElbow + l2sim*np.cos(theta1-theta2)*np.sin(theta0)
		zWrist = zElbow + l2sim*np.sin(theta1-theta2)

		#draw link1 and link2
		xpts = [0,xElbow,xWrist]
		ypts = [0,yElbow,yWrist]
		zpts = [0,zElbow,zWrist]

		lineOfArm = ax.plot(xpts,ypts,zpts, 'o-', mec = [abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], mew = 5, lw = 10)
		plt.draw()
		plt.pause(0.05)
		lineOfArm.remove()




	#keyboard interrupt to record position 
	try:
		if keyboard.is_pressed('q'):
			print("recording path sequence")
			recording = 1
			arr = [[theta0,theta1,theta2],[theta0,theta1,theta2]]
			np.savetxt('armPath.txt',arr)
	except:
		pass

	try:
		if keyboard.is_pressed('w'):
			recording = 0
			print("stopping recording")
			print("Press q to teach sequence. Press s to simulate. Press f to quit")
	except:
		pass
	#	break

	try: 
		if keyboard.is_pressed('s'):
			simulating = 1
			print("starting simulation")
			fig = plt.figure()
			ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)
			ax.grid(False)

			plt.xlabel("x",fontdict=None,labelpad=None)
			plt.ylabel("y",fontdict=None,labelpad=None)
			#plt.zlabel("z",fontdict=None,labelpad=None)
			ax.set_xlabel('x')
			ax.set_ylabel('y')
			ax.set_zlabel('z')
			plt.draw()
			plt.pause(0.5)
			plt.cla()
			ax.grid(False)
			ax.set_xlim(0,20)
			ax.set_ylim(0,20)
			ax.set_zlim(0,20)
	except:
		pass


	#alternate keyboard interrupt to end hand guided sequence
	try:
		if keyboard.is_pressed('f'):
			print("powering down")
			#slowly power down each joint
			while currentSetpoint1 > 3:
				currentSetpoint1 = 0.99 * currentSetpoint1
				currentSetpoint2 = 0.99 * currentSetpoint2
				time.sleep(0.1)
			currentSetpoint1 = 0
			currentSetpoint2 = 0
			od0.reboot()
			od1.reboot()
			od2.reboot()
			break
	except:
		pass
		#break
	time.sleep(0.01)


