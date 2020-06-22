import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from humanInertiaEstimator import inertiaEstimator
from ellipse import *
from matplotlib.patches import Ellipse

ie = inertiaEstimator()

fig = plt.figure()
ax = fig.add_subplot(xlim=(-0.5,0.5),ylim=(0,0.5))
ax.set_xlabel('x')
ax.set_ylabel('z')

fidelity = 0.1 #how far apart each point should be
linelen = fidelity
randForces = 20

x = np.arange(-0.5,0.5,fidelity)
y = 0
z = np.arange(0,0.5,fidelity)

#plot origin
ax.plot([0],[0],[0],'ro')

for xstep in x:
	#for ystep in y:
	for zstep in z:
		#set joint angles according to IK model
		ie.x0[0:3] = ie.cartesian2Joint(xstep,y,zstep)

		
		theta = ie.predictXZGauss()
		
		print('theta = ', theta)	

		#add first ellipse
		patches = []
		# ellipse1 = Ellipse([xstep,zstep],linelen*np.cos(theta),linelen*np.sin(theta),angle = 0)
		ellipse1 = Ellipse([xstep,zstep],linelen,linelen/2,angle = np.rad2deg(theta))
		patches.append(ellipse1)
		ax.add_patch(ellipse1)

		#not what we're looking for
		# drawEllipse(ax,xstep,zstep,linelen*np.cos(theta),linelen*np.sin(theta))

		#draw first line
		# ax.plot([xstep,xstep+linelen*np.sin(theta)],[zstep,zstep + linelen*np.cos(theta)],'b-')

		#draw second line(?)
		# ax.plot([xstep,xstep + linelen*np.sin(np.pi - theta)],[zstep,zstep + linelen*np.cos(np.pi - theta)],'r-')

		print(xstep,zstep)

plt.draw()
plt.pause(100)

#Notes
# want smallest possible change in position- need to keep things linear
# polyfit a slipe field?
