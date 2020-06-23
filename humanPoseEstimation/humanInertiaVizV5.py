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
ax = fig.add_subplot(xlim=(-0.7,0.7),ylim=(-0.7,0.7))
ax.set_xlabel('x')
ax.set_ylabel('y')

fidelity = 0.1 #how far apart each point should be
# linelen = fidelity

x = np.arange(-0.7,0.7,fidelity)
y = np.arange(-0.7,0.7,fidelity)
# y = 0
# z = np.arange(0,0.7,fidelity)
z = 0.0

ax.set_title('x vs y at z = 0.0')

#plot origin
ax.plot([0],[0],[0],'ro')

for xstep in x:
	for ystep in y:
	#for zstep in z:
		#set joint angles according to IK model
		ie.x0[0:3] = ie.cartesian2Joint(xstep,ystep,z)
		
		X,Y,Z = ie.predictXZGauss(numTrials = 20)
		
		data = np.array([X,Y])
		cov = np.cov(data)

		# ax.plot(x,z,'b.')

		lam1,lam2,theta = cov2Ell(cov)

		drawEllipse(ax,xstep,ystep,lam1,lam2,theta + np.pi/2) #inertia is perp. to movement

		# ax.plot([xstep,xstep + fidelity * np.cos(theta)],[zstep, zstep + fidelity * np.sin(theta)],'b-')

		print(xstep,ystep)

plt.draw()
plt.pause(100)

