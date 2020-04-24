from __future__ import print_function
import odrive
from odrive.enums import*
import time
import math
import numpy as np
import keyboard
import ctypes
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront

theta2 = 0
theta1 = 0
theta0 = 0
theta2eff = 0
recording = 0
simulating = 0

filename = 'armPath3.txt'

#init openGL stuff
window = pyglet.window.Window(width=1280, height=720)
keys = key.KeyStateHandler()
window.push_handlers(keys)

base = Wavefront('base.obj')
link0 = Wavefront('l0.obj')
link1 = Wavefront('l1.obj')
link2 = Wavefront('l2.obj')
link3 = Wavefront('l3.obj')
link4 = Wavefront('l4.obj')

l1sim = 6.5
l2sim = 6.5
l3sim = 2.65
rotation = 0.0 #count variable for spinning


i = 0
#pathArr = np.genfromtxt('armPath.txt',delimiter=" ")

#system parameters
#l0- shoulder side/side
serial0 = "206A337B304B"
l0cpr = 90
l0reduction = 6

#l1- shoulder up/down
l1m = 1.81 #kg was 3.07
l1com = 0.08 #m from center of rotation
l1 = 0.1651
l1cpr = 90
l1reduction = 6 #was 9, switched to og opentorque for less friction
l1kv = 16
#beta1 = 0
beta1 = -0.025 #to do- make beta a function of motor velocity to cancel out inertia? was -0.025 with 9:1
serial1 = "2084377D3548"

#l2- elbow
l2m = 2.259 #kg was 1.85, moved up to compensate for wrist? was 2.6 before end effector
l2com = 0.158 #m was 0.1893 moved up to compensate for wrist and end effector
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
print("j0 offset: ",j0offset)

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
print("j2 offset: ",j2offset)

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
print("j1 offset: ",j1offset)

#clears previous recording, starts off with current arm position
#get joint angle and velocity
pos2 = od2.axis0.encoder.pos_estimate - j2offset
pos1 = od1.axis0.encoder.pos_estimate - j1offset
pos0 = od0.axis0.encoder.pos_estimate - j0offset

#zero position is straight up
theta2 = (2 * np.pi * pos2) / (l2cpr * l2reduction)
theta1 = (2 * np.pi * pos1) / (l1cpr * l1reduction)
theta0 = (2 * np.pi * pos0) / (l0cpr * l0reduction)

initArr = [[theta0,theta1,theta2],[theta0,theta1,theta2]]
np.savetxt(filename,initArr)

print("press q to teach sequence. press w to stop teaching sequence. press f to quit")


@window.event
def on_resize(width, height):
	glMatrixMode(GL_PROJECTION)
	glLoadIdentity()
	gluPerspective(50., float(width)/height, 1., 100.) #change first argument for fov
	glTranslatef(0,-5,-40) #fits arm into camera view
	glMatrixMode(GL_MODELVIEW)
	return True


@window.event
def on_draw():
	window.clear()
	glClearColor(1,1,1,0.5) #sets background color
	glViewport(0,0,1280,720)
	glLoadIdentity()
	glMatrixMode(GL_PROJECTION)
	glRotatef(0,0,1,0)
	glRotatef(rotation*0.0035,0,1,0)
	glMatrixMode(GL_MODELVIEW)

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
	# if recording == 1:
	# 	#arr = [[theta0, theta1, theta2]]
	loadedArray = np.genfromtxt(filename,delimiter=" ")
	currPos = [[theta0,theta1,theta2]]
	print(currPos)
	loadedArray = np.append(loadedArray,currPos,axis=0)
	np.savetxt(filename,loadedArray)

	# if simulating == 1:

	# 	#calculate x y and z elbow and wrist
	# 	#l1sim = 0.40625*25.4 # upper arm
	# 	#l2sim = 0.59375*25.4 # lower arm l1+l2=1, easiest if upper and lower arm are same length
	# 	#l3sim = 0.375*25.4 #wrist to end effector
	# 	xElbow = ( l1sim * np.sin(theta0)*np.sin(theta1))
	# 	yElbow = ( l1sim * np.cos(theta0)*np.sin(theta1))
	# 	zElbow = ( l1sim * np.cos(theta1))

	# 	xWrist = xElbow + l2sim*np.cos(np.pi/2-(theta1+theta2))*np.sin(theta0)
	# 	yWrist = yElbow + l2sim*np.cos(np.pi/2-(theta1+theta2))*np.cos(theta0)
	# 	zWrist = zElbow + l2sim*np.sin(np.pi/2-(theta1+theta2))

	#keyboard interrupt to record position 
	# try:
	# 	if keyboard.is_pressed('q'):
	# 		print("recording path sequence press w to stop")
	# 		recording = 1
	# 		arr = [[theta0,theta1,theta2],[theta0,theta1,theta2]]
	# 		np.savetxt('armPath.txt',arr)
	# except:
	# 	pass

	# try:
	# 	if keyboard.is_pressed('w'):
	# 		recording = 0
	# 		print("stopping recording press q to start again")
	# 		print("Press q to teach sequence. Press s to simulate. Press f to quit")
	# except:
	# 	pass
	# #	break

	# try: 
	# 	if keyboard.is_pressed('s'):
	# 		simulating = 1
	# 		print("starting simulation")
	# except:
	# 	pass


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
			od0.axis0.requested_state = AXIS_STATE_IDLE
			od1.axis0.requested_state = AXIS_STATE_IDLE
			od2.axis0.requested_state = AXIS_STATE_IDLE
			#break
	except:
		pass
		#break

   #from pyglet script
	link0Rot = (-180/np.pi)*theta0 #flipped sign to fix display, may require further troubleshooting
	link1Rot = (180/np.pi)*theta1
	link2Rot = (180/np.pi)*theta2
	link3Rot = 0
	link4Rot = 45

 #  rotation = 0.0 #count variable for spinning
	lightfv = ctypes.c_float * 4

	link2RotEff = link1Rot + link2Rot

	xElb = ( l1sim * np.sin(link0Rot*(np.pi/180))*np.sin(link1Rot*(np.pi/180)))
	yElb = ( l1sim * np.cos((link1Rot*(np.pi/180)))) 
	zElb =  ( l1sim * np.cos(link0Rot*(np.pi/180))*np.sin(link1Rot*(np.pi/180)))

	xl3 = xElb + ( l2sim * np.sin(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))
	yl3 = yElb + ( l2sim * np.cos((link2RotEff*(np.pi/180)))) 
	zl3 = zElb + ( l2sim * np.cos(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))

	xl4 = xElb + ( (l2sim+l3sim) * np.sin(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))
	yl4 = yElb + ( (l2sim+l3sim) * np.cos((link2RotEff*(np.pi/180)))) 
	zl4 = zElb + ( (l2sim+l3sim) * np.cos(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))

	#glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0*np.sin(rotation*0.1), 1.0, 0.0))
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 0.9))
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightfv(0.0,0.0,0.0,0.1))

	draw_base(base)
	draw_link0(link0, 0, 0, 0, link0Rot)
	draw_link1(link1, 0, 0, 0,link0Rot, link1Rot)
	draw_link2(link2, xElb, yElb, zElb, link0Rot, link1Rot, link2Rot)
	draw_link3(link3, xl3, yl3, zl3, link0Rot, link1Rot, link2Rot,link3Rot)
	draw_link4(link4, xl4, yl4, zl4, link0Rot, link1Rot, link2Rot,link3Rot,link4Rot)
	time.sleep(0.01)

def draw_base(link):
	glLoadIdentity()
	glMatrixMode(GL_MODELVIEW)
	glTranslatef(0,-3.4,0)
	visualization.draw(link)

def draw_link0(link, x, y, z, link0Rot):
	glLoadIdentity()
	glMatrixMode(GL_MODELVIEW)
	glRotatef(link0Rot, 0.0, 1.0 , 0.0)
	glTranslatef(x, y, z)
#    glRotatef(-25.0, 1.0, 0.0, 0.0)
#    glRotatef(45.0, 0.0, 0.0, 1.0)

	visualization.draw(link)

def draw_link1(link, x, y, z,link0Rot, link1Rot):
	glLoadIdentity()
	glMatrixMode(GL_MODELVIEW)
	#glRotatef(180,0,1,0) #flips l1 around so it isnt bending backwards
	glRotatef(link0Rot, 0.0, 1.0 , 0.0)
	glRotatef(link1Rot, 1.0, 0.0 , 0.0)
	#glTranslated(x, y, z) # link 1 does not translate about workspace, only pivots on shoulder base
#   glRotatef(-25.0, 1.0, 0.0, 0.0)
#   glRotatef(45.0, 0.0, 0.0, 1.0)

	visualization.draw(link)
	
def draw_link2(link, x, y, z, link0Rot, link1Rot, link2Rot):
	glLoadIdentity()
	glMatrixMode(GL_MODELVIEW)
	glTranslatef(x, y, z)
	#print("link0Rot: ", link0Rot, " Link1Rot: ", link1Rot, " Link2Rot: ", link2Rot)
	glRotatef(link0Rot, 0.0, 1.0 , 0.0)
	glRotatef(link1Rot, 1.0, 0.0 , 0.0)
	glRotatef(link2Rot, 1.0, 0.0, 0.0)
	#glRotatef(180,0,1,0) #flipped around to match l1

   #glTranslatef(-x, -y, -z)
   
#   glRotatef(45.0, 0.0, 0.0, 1.0)

	visualization.draw(link)
   
def draw_link3(link, x, y, z, link0Rot, link1Rot, link2Rot,rotation):
	glLoadIdentity()
	glMatrixMode(GL_MODELVIEW)
	glTranslatef(x, y, z)
	glRotatef(link0Rot, 0.0, 1.0 , 0.0)
	glRotatef(link1Rot, 1.0, 0.0 , 0.0)
	glRotatef(link2Rot, 1.0, 0.0, 0.0)
	glRotatef(rotation,0.0,1.0,0.0)
	 
#   glRotatef(45.0, 0.0, 0.0, 1.0)

	visualization.draw(link)

def draw_link4(link, x, y, z, link0Rot, link1Rot, link2Rot,rotation,link4Rot):
	glLoadIdentity()
	glMatrixMode(GL_MODELVIEW)
	glTranslatef(x, y, z)
	glRotatef(link0Rot, 0.0, 1.0 , 0.0)
	glRotatef(link1Rot, 1.0, 0.0 , 0.0)
	glRotatef(link2Rot, 1.0, 0.0, 0.0)
	glRotatef(rotation,0.0,1.0,0.0)
	glRotatef(link4Rot,1.0,0.0,0.0)
	visualization.draw(link)

def update(dt):
	global rotation
	global i
 #  rotation += 10.0 * dt
	if keys[key.A]:
	   rotation += 10
	if keys[key.D]:
		rotation -= 10
	if keys[key.S]:
		cameraZ -= 5
	i += 1

#   if rotation > 720.0:
#	 rotation = 0.0


pyglet.clock.schedule(update)
pyglet.app.run()
