import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep

def IKModel(path,ax,xIn=0,yIn=0,zIn=0,wTheta=numpy.pi,wPhi=numpy.pi):

	l1 = 0.40625*25.4 # upper arm
	l2 = 0.59375*25.4 # lower arm l1+l2=1, easiest if upper and lower arm are same length
	l3 = 0.375*25.4 #wrist to end effector

	#make path go from start to finish
	path = numpy.flip(path,axis=1)

	#test start vales
	x = path[0][0]
	y = path[1][0]
	z = path[2][0]

	count = 1
	while count < path.shape[1]:

		##adjusts inputs in IK function to account for length of l3 (wrist) component
		xIn = x - l3*numpy.cos(wTheta)*numpy.sin(wPhi)
		yIn = y - l3*numpy.sin(wTheta)*numpy.sin(wPhi)
		zIn = z - l3*numpy.cos(wPhi)

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

		xs = [0,xElbow,uwX,x]
		ys = [0,yElbow,uwY,y]
		zs = [0,zElbow,uwZ,z]

		lineOfArm, = ax.plot(xs,ys,zs, 'o-', mec = [abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], mew = 5, lw = 10, color=[0.8,0.2,0.5])

		shoulder, = ax.plot([0],[0],[0], 'o-', color = [0.5*abs(numpy.sin(theta)),0.25*abs(numpy.cos(theta)),0.75], lw = 10, ms = 15) 
		elbow, = ax.plot([xElbow],[yElbow],[zElbow], 'o-', color=[abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], lw = 10, ms = 15)
		upperWrist, = ax.plot([uwX],[uwY],[uwZ], 'o-', color = [abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], lw = 10, ms = 15)

		plt.draw()
		plt.pause(0.05)
		lineOfArm.remove()
		shoulder.remove()
		elbow.remove()
		upperWrist.remove()

		x = path[0][count]
		y = path[1][count]
		z = path[2][count]

		count = count + 1

		#ax.cla()
		#ax.set_xlim(-1,1)
		#ax.set_ylim(-1,1)
		#ax.set_zlim(0,1)
		#ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)



def cartesian_to_spherical(x,y,z):
	r = numpy.sqrt((x*x)+(y*y)+(z*z))
	phi = numpy.arctan2((numpy.sqrt((x  * x) + (y * y))), z )
	theta = numpy.arctan2(y, x)
	return (r,phi,theta)

def get_joint_angles(x,y,z):
	l1 = 0.40625*25.4 # upper arm
	l2 = 0.59375*25.4 # lower arm l1+l2=1, easiest if upper and lower arm are same length
	l3 = 0.375*25.4 #wrist to end effector
	
	(r,phi,theta) = cartesian_to_spherical(x,y,z)
	
	#elbow
	a2 = numpy.arccos(((l1*l1)+(l2*l2)-(r*r))/(-2*l1*l2))
	#shoulder side to side
	a0 = theta
	#shoulder up down
	a1 = numpy.pi + phi + numpy.arccos(((l1*l1)-(l2*l2)+(r*r))/(-2*l1*r))
	
	return(a0,a1,a2)

def get_elbow_pos(x,y,z):

	l1 = 0.40625*25.4 # upper arm
	l2 = 0.59375*25.4 # lower arm l1+l2=1, easiest if upper and lower arm are same length
	l3 = 0.375*25.4 #wrist to end effector

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