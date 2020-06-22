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
ax = fig.add_subplot(xlim=(-0.5,0.5),ylim=(-0.5,0.5))
ax.set_xlabel('x')
ax.set_ylabel('z')

fidelity = 0.025 #how far apart each point should be

x = np.arange(-0.5,0.5,fidelity)
y = 0
z = np.arange(-0.5,0.5,fidelity)

#plot origin
ax.plot([0],[0],[0],'ro')

for xstep in x:
	#for ystep in y:
	for zstep in z:
		#set joint angles according to IK model
		ie.x0[0:3] = ie.cartesian2Joint(xstep,y,zstep)

		
		X,Z = ie.predictXZGauss(numTrials = 20)
		
		data = np.array([X,Z])
		cov = np.cov(data)

		# ax.plot(x,z,'b.')

		lam1,lam2,theta = cov2Ell(cov)

		drawEllipse(ax,xstep,zstep,lam1,lam2,theta + np.pi/2) #inertia is perp. to movement

		print(xstep,zstep)

plt.draw()
plt.pause(100)

