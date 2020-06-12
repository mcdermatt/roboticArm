import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from humanInertiaEstimator import inertiaEstimator
from ellipse import *


ie = inertiaEstimator()

fig = plt.figure()
ax = fig.add_subplot(xlim=(-0.1,0.5),ylim=(0,0.5))
ax.set_xlabel('x')
ax.set_ylabel('z')

fidelity = 0.05 #how far apart each point should be
cielI = 1000 #cap on inertia for viz
scalingFactor = 0.25

x = np.arange(0,0.3 + fidelity,fidelity)
y = 0
z = np.arange(0,0.3 + fidelity,fidelity)

#plot origin
ax.plot([0],[0],[0],'ro')

for xstep in x:
	#for ystep in y:
	for zstep in z:
		#set joint angles according to IK model
		ie.x0[0:3] = ie.cartesian2Joint(xstep,y,zstep)

		inertias = ie.getInertia()

		inertias = inertias*scalingFactor

		if np.isnan(inertias).all() == 0:
			print('inertias = ', inertias)
			print(xstep,y,zstep)
			

			#set ceiling on how large inertia can be for plotting
			inertias[inertias > cielI] = cielI

			#draw cross
			cross = drawCross(ax,xstep,zstep,inertias[0]/2,inertias[2]/2)

			#draw ellipses
			ell = drawEllipse(ax,xstep,zstep,inertias[0],inertias[2])


			#dots
			#ax.plot([xstep],[ystep],[zstep],'.' ,color = [inertias[0]/np.sum(inertias),inertias[2]/np.sum(inertias),inertias[1]/np.sum(inertias)])
		#color = [inertias[0]/np.sum(inertias),inertias[1]/np.sum(inertias),inertias[2]/np.sum(inertias)] 
			

plt.draw()
plt.pause(100)