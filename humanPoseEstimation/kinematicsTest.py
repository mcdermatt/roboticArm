from humanInertiaEstimator import inertiaEstimator
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = plt.axes(projection='3d',xlim=(0,0.5),ylim=(0,0.5), zlim=(0,0.5))
ax.set_xlabel('x')
ax.set_ylabel('z')
ax.set_zlabel('y')

#plot origin
ax.plot([0],[0],[0],'ro')

ie = inertiaEstimator()

stepsize = 0.2
x = np.arange(0,0.4+stepsize,stepsize)
z = np.arange(0,0.4+stepsize,stepsize)
# y = np.zeros(len(z)) #start with simple case of flat shoulder
y = np.arange(0,0.4+stepsize,stepsize)

for xstep in x:
	for ystep in y:
		for zstep in z:
			#where we should be
			ax.plot([xstep],[zstep],[ystep],'g.')
			print('actual cart = ',xstep,' ', ystep,' ', zstep)

			joint = ie.cartesian2Joint(xstep,ystep,zstep)
			print('joint from inverse kinematics = ', joint)
			cart = ie.joint2Cartesian(joint[0],joint[1],joint[2])
			print('cart from forward kinematics= ', cart)

			ax.plot([cart[0]],[cart[2]],[cart[1]],'ro')



plt.draw()
plt.pause(15)
