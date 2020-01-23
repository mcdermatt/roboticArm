from __future__ import print_function
import odrive
from odrive.enums import*
import time
import math
import numpy as np
import keyboard
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#get simulation plot running in background
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
ax.set_xlim(-20,20)
ax.set_ylim(-20,20)
ax.set_zlim(0,20)

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

l1sim = 0.40625*25.4 # upper arm
l2sim = 0.59375*25.4 # lower arm

# od0 = odrive.find_any(serial_number=serial0)
# od2 = odrive.find_any(serial_number=serial2)
# od1 = odrive.find_any(serial_number=serial1)

#arm moves through pre-recorded sequence
print("WARNING PROGRAM ASSUMES ARM IS CALIBRATED TO VERTICAL POSITION")
print("move arm to downward resting position")
time.sleep(3)

#sets j0 to reference tracking position control (PID)
od0.axis0.controller.config.control_mode = 3


pathArr = np.genfromtxt('armPath.txt',delimiter=" ")

#raise arm from horizontal position to starting position

i = 0
while i < len(pathArr):

	theta0Goal = pathArr[i,0]
	theta1Goal = pathArr[i,1]
	theta2Goal = pathArr[i,2]
	xElbowGoal = ( l1sim * np.sin(theta0Goal)*np.sin(theta1Goal))
	yElbowGoal = ( l1sim * np.cos(theta0Goal)*np.sin(theta1Goal))
	zElbowGoal = ( l1sim * np.cos(theta1Goal))
	xWristGoal = xElbowGoal + l2sim*np.cos(np.pi/2-(theta1Goal+theta2Goal))*np.sin(theta0Goal)
	yWristGoal = yElbowGoal + l2sim*np.cos(np.pi/2-(theta1Goal+theta2Goal))*np.cos(theta0Goal)
	zWristGoal = zElbowGoal + l2sim*np.sin(np.pi/2-(theta1Goal+theta2Goal))

	#draw link1 and link2
	xptsGoal = [0,xElbowGoal,xWristGoal]
	yptsGoal = [0,yElbowGoal,yWristGoal]
	zptsGoal = [0,zElbowGoal,zWristGoal]

	lineOfArmGoal, = ax.plot(xptsGoal,yptsGoal,zptsGoal, 'o-', color = [0.5,0.5,1], mec = [0.5,0.5,0.5], mew = 5, lw = 10)
	plt.draw()
	plt.pause(0.01)
	lineOfArmGoal.remove()

	time.sleep(0.01)
	i += 1

