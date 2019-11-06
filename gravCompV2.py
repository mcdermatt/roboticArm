#rom IKModelGeneral.py import cartesian_to_spherical

import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
from inputs import devices
from inputs import get_gamepad

#Inverse Kinematic Model for shoulder to writ for Compliant Arm mk7


def cartesian_to_spherical(x,y,z):
	
	r = numpy.sqrt((x*x)+(y*y)+(z*z))
	phi = numpy.arctan2((numpy.sqrt((x  * x) + (y * y))), z )
	theta = numpy.arctan2(y, x)
	return (r,phi,theta)

def get_joint_angles(x,y,z):
	
	(r,phi,theta) = cartesian_to_spherical(x,y,z)
	
	#elbow
	a2 = -numpy.arcsin(((l1*l1)+(l2*l2)-(r*r))/(-2*l1*l2))
	
	#shoulder side to side
	a0 = theta
	
	#shoulder up down
	a1 = numpy.pi + phi + numpy.arccos(((l1*l1)-(l2*l2)+(r*r))/(-2*l1*r))
	
	return(a0,a1,a2)

def get_elbow_pos(x,y,z):

	(r, phi, theta) = cartesian_to_spherical(x,y,z)
	(a0,a1,a2) = get_joint_angles(x,y,z)
	
	xElbow = ( l1 * numpy.cos(a0)*numpy.sin(a1))
	yElbow = ( l1 * numpy.sin(a0)*numpy.sin(a1))
	zElbow = ( l1 * numpy.cos(a1))
	
	return (xElbow, yElbow, zElbow)

def get_l3_pos(x,y,z):
	
	r,phi,theta = cartesian_to_spherical(x,y,z)
	a0,a1,a2 = get_joint_angles(x,y,z)
	xElbow,yElbow,zElbow = get_elbow_pos(a0,a1,a2)
	
	
	xl3 = r*numpy.cos(theta)*numpy.sin(phi)
	
		
	yl3 = r*numpy.sin(theta)*numpy.sin(phi)
	
	
	zl3 = r*numpy.cos(phi)

	
	return (xl3,yl3,zl3)

def get_l1COM_pos(x,y,z): #gets position of the center of mass of link 1 for gravity compensation

	(r, phi, theta) = cartesian_to_spherical(x,y,z)
	(a0,a1,a2) = get_joint_angles(x,y,z)
	
	xl1COM = ( l1COM * numpy.cos(a0)*numpy.sin(a1))
	yl1COM = ( l1COM * numpy.sin(a0)*numpy.sin(a1))
	zl1COM = ( l1COM * numpy.cos(a1))
	
	return (xl1COM, yl1COM, zl1COM)

def get_l2COM_pos(x,y,z): #get position of center of mass of link 2
	(r, phi, theta) = cartesian_to_spherical(x,y,z)
	(a0,a1,a2) = get_joint_angles(x,y,z)

	xElbow = ( l1 * numpy.cos(a0)*numpy.sin(a1))
	yElbow = ( l1 * numpy.sin(a0)*numpy.sin(a1))
	zElbow = ( l1 * numpy.cos(a1))
	xl3 = r*numpy.cos(theta)*numpy.sin(phi)
	yl3 = r*numpy.sin(theta)*numpy.sin(phi)	
	zl3 = r*numpy.cos(phi)


	xl2COM = xElbow + (l2COM/l2)*(xl3 - xElbow)
	yl2COM = yElbow + (l2COM/l2)*(yl3 - yElbow)
	zl2COM = zElbow + (l2COM/l2)*(zl3 - zElbow)

	return(xl2COM,yl2COM,zl2COM)



#init   
#l0 = 0.25
l1 = 0.40625 # upper arm
l1COM = 0.1794   #2.87in from shoulder pivot
l1M = 3.07 # weight = 6.78lb = 3.07kg

l2 = 0.59375 # lower arm l1+l2=1, easiest if upper and lower arm are same length irl l1+l2 = 6.5 + 9.5 = 16in
l2COM = 0.46575 # 7.452 in from axis of rotation
l2M = 1.85 #kg or 4.075 lbs

l3 = 0.375 #wrist to end effector

#------------------Enter desired x y and z values here: ---------------------
x = 0.5
y = 0.125
z = 0.5
wTheta = (numpy.pi)/2
wPhi =  (numpy.pi)/2
#----------------------------------------------------------------------------

#adjusts inputs in IK function to account for length of l3 (wrist) component
xIn = x - l3*numpy.cos(wTheta)*numpy.sin(wPhi)
yIn = y - l3*numpy.sin(wTheta)*numpy.sin(wPhi)
zIn = z - l3*numpy.cos(wPhi)
#count = 0

fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)

plt.xlabel("x",fontdict=None,labelpad=None)
plt.ylabel("y",fontdict=None,labelpad=None)
#plt.zlabel("z",fontdict=None,labelpad=None)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(0,1)
ax.grid(False)
ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
ax.set_facecolor('xkcd:sky blue')
count = 0

while 1:

	events = get_gamepad()
	for event in events:
		if event.code == 'ABS_X':
			if event.state > 5000:
				print("right",event.state)
				x = x + 0.05
			if event.state < -5000:
				print("left",event.state)
				x = x - 0.05
		if event.code == 'ABS_Y':
			if event.state > 5000:
				print("up",event.state)
				y = y + 0.05
			if event.state < -5000:
				print("down",event.state)
				y = y - 0.05
		if event.code == 'RightTrigger':
			if event.state == 1:
				print("up")
	#adjusts inputs in IK function to account for length of l3 (wrist) component
	xIn = x - l3*numpy.cos(wTheta)*numpy.sin(wPhi)
	yIn = y - l3*numpy.sin(wTheta)*numpy.sin(wPhi)
	zIn = z - l3*numpy.cos(wPhi)
	#count = 0

	

	#print(xIn,yIn,zIn)
	r,phi,theta = cartesian_to_spherical(xIn,yIn,zIn)
	#print(r)
	a0,a1,a2 = get_joint_angles(xIn,yIn,zIn)
	xElbow,yElbow,zElbow = get_elbow_pos(xIn,yIn,zIn)
	#print (numpy.rad2deg(a0), numpy.rad2deg(a1),  numpy.rad2deg(a2))
	#print (xElbow,yElbow,zElbow)
	#get l3 pos
	uwX, uwY, uwZ = get_l3_pos(xIn,yIn,zIn)
	#print(uwX,uwY,uwZ)
	#print(' ')


	xl2COM,yl2COM,zl2COM = get_l2COM_pos(xIn,yIn,zIn)
	l2Torque = l2M*numpy.sqrt((xl2COM-xElbow)*(xl2COM-xElbow)+ (yl2COM-yElbow)*(yl2COM-yElbow))*9.81*100/39.37
	l2CurrentSetpoint = l2Torque/6

	xl1COM,yl1COM,zl1COM = get_l1COM_pos(xIn,yIn,zIn)
	l1Torque = (l2M*numpy.sqrt(xl2COM*xl2COM + yl2COM*yl2COM) + l1M*numpy.sqrt(xl1COM*xl1COM + yl1COM*yl1COM))*9.81*16/39.37 
	#print("l1 Torque= ") 
	#print(l1Torque)
	l1CurrentSetpoint = l1Torque*8.27/9
	#print("l1 Current Setpoint= ")
	#print(l1CurrentSetpoint)

	print(numpy.rad2deg(a1)-360)
	print(numpy.rad2deg(a2))

	xs = [0,xElbow,uwX,x]
	ys = [0,yElbow,uwY,y]
	zs = [0,zElbow,uwZ,z]

	line = ax.plot(xs,ys,zs, 'o-', mec = [abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], mew = 5, lw = 10)

	shoulder = ax.plot([0],[0],[0], 'o-', color = [0.5*abs(numpy.sin(theta)),0.25*abs(numpy.cos(theta)),0.75], lw = 10, ms = 15) 
	elbow = ax.plot([xElbow],[yElbow],[zElbow], 'o-', color=[abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], lw = 10, ms = 15)
	upperWrist = ax.plot([uwX],[uwY],[uwZ], 'o-', color = [abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], lw = 10, ms = 15)

	l1Force = ax.plot([xl1COM],[yl1COM],[zl1COM], 'o-', color = [0.5,0.5,0.5], lw = 5, ms = 5) #does not account for links 2 3 or 4 atm
	l1Data = str(l1CurrentSetpoint)[0:5]
	l1Data = "     J1 Current Setpoint: " + l1Data + "A"
	ax.text(0,0,0,'%s' % (l1Data), size=8, zorder=1, color='k')


	l2Force = ax.plot([xl2COM],[yl2COM],[zl2COM], 'o-', color = [0.5,0.5,0.5], lw = 5, ms = 5)
	l2Data = str(l2CurrentSetpoint)[0:5]
	l2Data = "     J2 Current Setpoint: " + l2Data + "A"
	ax.text(xElbow,yElbow,zElbow,'%s' % (l2Data), size=8, zorder=1, color='k')

	plt.draw()
	plt.pause(0.01)
	ax.cla()
	ax.set_xlim(-1,1)
	ax.set_ylim(-1,1)
	ax.set_zlim(0,1)
	ax.grid(False)
	ax.set_xticks([])
	ax.set_yticks([])
	ax.set_zticks([])
	ax.set_facecolor('xkcd:sky blue')
	

plt.pause(10)
#ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)