import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from humanInertiaEstimator4DOF import inertiaEstimator4DOF
from ellipse import *
from matplotlib.patches import Ellipse

#displays inertia ellipses about workspace and saves resulting slopes to numpy file

ie = inertiaEstimator4DOF()

fig = plt.figure()
ax = fig.add_subplot(xlim=(-0.7,0.7),ylim=(-0.7,0.7))
ax.set_xlabel('x')
ax.set_ylabel('z')

fidelity = 0.1 #how far apart each point should be
# linelen = fidelity

x = np.arange(-0.7,0.7,fidelity)
# y = np.arange(-0.7,0.7,fidelity)
y = 0
z = np.arange(0,0.7,fidelity)
# z = 0.0

ax.set_title('x vs z at y = 0.0')

#plot origin
ax.plot([0],[0],[0],'ro')

outfile = '4DOFxz.npy'
thetaArr = np.zeros(len(x)*len(z))
i = 0

for xstep in x:
	# for ystep in y:
	for zstep in z:
		#set joint angles according to 3dof IK model ignore bonus shoulder rotation when looking at z
		ie.x0[0], ie.x0[2], ie.x0[3] = ie.cartesian2Joint(xstep,y,zstep)
		
		X,Y,Z = ie.predictXZGauss(numTrials = 20)
		
		data = np.array([X,Z])
		cov = np.cov(data)

		# ax.plot(x,z,'b.')

		lam1,lam2,theta = cov2Ell(cov)
		theta = theta + np.pi/2 #stiffness ell perpindicular to motion
		thetaArr[i] = theta

		drawEllipse(ax,xstep,zstep,lam1,lam2,theta)

		# ax.plot([xstep,xstep + fidelity * np.cos(theta)],[zstep, zstep + fidelity * np.sin(theta)],'b-')

		print(xstep,zstep)
		i += 1

np.save(outfile,thetaArr)
plt.draw()
plt.pause(100)

