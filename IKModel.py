import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep




def cartesian_to_spherical(x,y,z):
	
	r = numpy.sqrt((x*x)+(y*y)+(z*z))
	phi = numpy.arctan2((numpy.sqrt((x  * x) + (y * y))), z )
	theta = numpy.arctan2(y, x)
	return (r,phi,theta)

def get_joint_angles(x,y,z):
	
	(r,phi,theta) = cartesian_to_spherical(x,y,z)
	
	#elbow
	a2 = 2*numpy.arcsin(r/(2*l1))
	
	#shoulder side to side
	a0 = theta
	
	#shoulder up down
	a1 = phi - (numpy.pi/2 - (a2 / 2))
	
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


#init   
#l0 = 0.25
l1 = 0.5
l2 = l1 #easiest if upper and lower arm are same length

#Enter desired x y and z values here: ---------------------
xIn = 0.0
yIn = -0.25
zIn =  0.75
count = 0

fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)

plt.xlabel("x",fontdict=None,labelpad=None)
plt.ylabel("y",fontdict=None,labelpad=None)
#plt.zlabel("z",fontdict=None,labelpad=None)

plt.ion()

while count <= 150:
	# ax.xlim=(-1,1)
	# ax.ylim=(-1,1)
	# ax.zlim=(0,1)
	print(xIn,yIn,zIn)
	r,phi,theta = cartesian_to_spherical(xIn,yIn,zIn)
	print(r)
	a0,a1,a2 = get_joint_angles(xIn,yIn,zIn)
	xElbow,yElbow,zElbow = get_elbow_pos(xIn,yIn,zIn)
	print (numpy.rad2deg(a0), numpy.rad2deg(a1),  numpy.rad2deg(a2))
	print (xElbow,yElbow,zElbow)
	#get l3 pos
	uwX, uwY, uwZ = get_l3_pos(xIn,yIn,zIn)
	print(uwX,uwY,uwZ)
	print(' ')



	xs = [0,xElbow,uwX]
	ys = [0,yElbow,uwY]
	zs = [0,zElbow,uwZ]

	line = ax.plot(xs,ys,zs, 'o-', mec = [abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], mew = 5, lw = 10)

	shoulder = ax.plot([0],[0],[0], 'o-', color = [0.5*abs(numpy.sin(theta)),0.25*abs(numpy.cos(theta)),0.75], lw = 10, ms = 15) 
	elbow = ax.plot([xElbow],[yElbow],[zElbow], 'o-', color=[abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], lw = 10, ms = 15)
	upperWrist = ax.plot([uwX],[uwY],[uwZ], 'o-', color = [abs(numpy.cos(phi)),abs(numpy.sin(phi)),.25], lw = 10, ms = 15)

	plt.draw()
	plt.pause(0.05)
	ax.cla()
	ax.set_xlim(-1,1)
	ax.set_ylim(-1,1)
	ax.set_zlim(0,1)
	#ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)
	count = count + 1
	
	if count <= 50:
		xIn = xIn + 0.01
		yIn = yIn + 0.005
		zIn = zIn - 0.01

	if count > 50 and count <= 100:
		xIn = xIn - 0.01
		yIn = yIn - 0.01

	if count > 100:
		zIn = zIn - 0.005

plt.pause(5)