import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from humanInertiaEstimator import inertiaEstimator


#IMPORTANT NOTES
#   Distance from origin is going to be flipped for the human shoulder- this is plotting as 0,0,0 as the shoulder location where it is actually a nonzero point in the robot's frame (what we are holding to be ground truth)

def drawEllipsoid(x,y,z,Ix,Iy,Iz):
	
	scalingFactor = 0.005

	ellipsoidDetail = 12 #number of vertices in each ellipsoid
	phi = np.linspace(0,2*np.pi, ellipsoidDetail).reshape(ellipsoidDetail, 1) # the angle of the projection in the xy-plane
	theta = np.linspace(0, np.pi, ellipsoidDetail).reshape(-1, ellipsoidDetail) # the angle from the polar axis, ie the polar angle

	elX = Ix * scalingFactor * np.sin(theta)*np.cos(phi) + x
	elZ = Iz * scalingFactor * np.sin(theta)*np.sin(phi) + z
	elY = Iy * scalingFactor * np.cos(theta) + y

	inertiaTotal = Ix+Iy+Iz
	surf = ax.plot_surface(elX,elY,elZ, color = [Ix/inertiaTotal,Iy/inertiaTotal,Iz/inertiaTotal])

	return


ie = inertiaEstimator()

fig = plt.figure()
# ax = fig.add_subplot(111, xlim=(-0.5,0.5), ylim=(0,0.5), zlim=(-0.5,0.5), projection='3d', autoscale_on=False)
ax = plt.axes(projection='3d',xlim=(-0.5,0.5),ylim=(0,0.5), zlim=(-0.5,0.5))
# ax.grid(False)
ax.set_xlabel('x')
ax.set_ylabel('z')
ax.set_zlabel('y')

fidelity = 0.1 #how far apart each point should be
cielI = 15 #cap on inertia for viz

x = np.arange(-0.3,0.5 + fidelity,fidelity)
y = np.arange(0,0.5 + fidelity,fidelity)
z = np.arange(0,0.5 + fidelity,fidelity)

#plot origin
ax.plot([0],[0],[0],'ro')

for xstep in x:
	for ystep in y:
		for zstep in z:
			#set joint angles according to IK model
			ie.x0[0:3] = ie.cartesian2Joint(xstep,ystep,zstep)

			inertias = ie.getInertia()
			print(inertias)
			print(xstep,ystep,zstep)
			if np.isnan(inertias).all() == 0:
				
				#set ceiling on how large inertia can be for plotting
				inertias[inertias > cielI] = cielI

				#draw ellipsoids
				el = drawEllipsoid(xstep,ystep,zstep,inertias[0],inertias[1],inertias[2])

				#dots
				#ax.plot([xstep],[ystep],[zstep],'.' ,color = [inertias[0]/np.sum(inertias),inertias[2]/np.sum(inertias),inertias[1]/np.sum(inertias)])
			#color = [inertias[0]/np.sum(inertias),inertias[1]/np.sum(inertias),inertias[2]/np.sum(inertias)] 
			

plt.draw()
plt.pause(15)



