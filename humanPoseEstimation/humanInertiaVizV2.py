import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from humanInertiaEstimator import inertiaEstimator


#IMPORTANT NOTES
#   Distance from origin is going to be flipped for the human shoulder- this is plotting as 0,0,0 as the shoulder location where it is actually a nonzero point in the robot's frame (what we are holding to be ground truth)
# 	Be careful not to switch up y and z
#	Make sure forces are DIFFERENTIALLY SMALL- too big and kinematic constraints will become more apparent

# def drawEllipsoid(x,y,z,Ix,Iy,Iz):
# 	''' x y and z projections of inertia into cartesian space, ultimately not that useful '''
# 	scalingFactor = 0.00025

# 	ellipsoidDetail = 12 #number of vertices in each ellipsoid
# 	phi = np.linspace(0,2*np.pi, ellipsoidDetail).reshape(ellipsoidDetail, 1) # the angle of the projection in the xy-plane
# 	theta = np.linspace(0, np.pi, ellipsoidDetail).reshape(-1, ellipsoidDetail) # the angle from the polar axis, ie the polar angle

# 	elX = Ix * scalingFactor * np.sin(theta)*np.cos(phi) + x
# 	elY = Iy * scalingFactor * np.sin(theta)*np.sin(phi) + y
# 	elZ = Iz * scalingFactor * np.cos(theta) + z

# 	inertiaTotal = Ix+Iy+Iz
# 	surf = ax.plot_surface(elX,elY,elZ,color = [Ix/inertiaTotal,Iy/inertiaTotal,Iz/inertiaTotal])

# 	return

def drawCross(x,z,Ix,Iz):
	'''x and z projections of inertia'''

	sf = 0.00005 #scaling factor

	#horizontal line
	ax.plot([x+Ix*sf,x-Ix*sf],[z,z],'b-')

	#vertical line
	ax.plot([x,x],[z+Iz*sf,z-Iz*sf],'b-')

	return

def drawRotatedEllipse(x,z,Ix,Iz):

	#loop through different angles and find volume(?) of each ellipse
	#	constrained by: two points (from cross) and angle (specified in param sweep)

	theta = np.linspace(0,pi,20)
	bestVolume = 0
	bestAng = 0
	for ang in theta:

		#x and z coords of cross rotated by theta
		xp = np.array([x*np.cos(theta),x*np.sin(theta)])
		zp = np.array([-z*np.sin(theta),z*np.cos(theta)])

		#whatever angle produes ell of largest volume will be the answer (or pi - that since there will always be 2 solutions?)
		if vol > bestVolume:
			bestVolume = vol
			bestAng = ang

def areaOfEllipse(l,w):


ie = inertiaEstimator()

fig = plt.figure()
ax = fig.add_subplot(xlim=(0,0.5),ylim=(0,0.5))
# ax = fig.add_subplot(111, xlim=(-0.5,0.5), ylim=(0,0.5), zlim=(-0.5,0.5), projection='3d', autoscale_on=False)
# ax = plt.axes(projection='3d',xlim=(0,0.5),ylim=(0,0.5), zlim=(0,0.5))
# ax.grid(False)
ax.set_xlabel('x')
ax.set_ylabel('z')
# ax.set_zlabel('y')

fidelity = 0.05 #how far apart each point should be
cielI = 1000 #cap on inertia for viz

x = np.arange(0,0.6 + fidelity,fidelity)
y = 0
z = np.arange(0,0.6 + fidelity,fidelity)

#plot origin
ax.plot([0],[0],[0],'ro')

for xstep in x:
	#for ystep in y:
	for zstep in z:
		#set joint angles according to IK model
		ie.x0[0:3] = ie.cartesian2Joint(xstep,y,zstep)

		inertias = ie.getInertia()

		if np.isnan(inertias).all() == 0:
			print('inertias = ', inertias)
			print(xstep,y,zstep)
			

			#set ceiling on how large inertia can be for plotting
			# inertias[inertias > cielI] = cielI

			#draw cross
			cross = drawCross(xstep,zstep,inertias[0],inertias[2])

			#draw ellipsoids
			# el = drawEllipsoid(xstep,zstep,0,inertias[0],inertias[2],inertias[1])

			#dots
			#ax.plot([xstep],[ystep],[zstep],'.' ,color = [inertias[0]/np.sum(inertias),inertias[2]/np.sum(inertias),inertias[1]/np.sum(inertias)])
		#color = [inertias[0]/np.sum(inertias),inertias[1]/np.sum(inertias),inertias[2]/np.sum(inertias)] 
			

plt.draw()
plt.pause(20)



