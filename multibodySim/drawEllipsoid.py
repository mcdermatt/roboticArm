import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#be careful- z and y axis switched between SM and MPL

fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)
ax.grid(False)

plt.xlabel("x",fontdict=None,labelpad=None)
plt.ylabel("y",fontdict=None,labelpad=None)
#plt.zlabel("z",fontdict=None,labelpad=None)
ax.set_xlabel('x')
ax.set_ylabel('z')
ax.set_zlabel('y')
ax.grid(False)

count = 1
forceArr = np.loadtxt('ellAxis.txt',delimiter=',',usecols=range(96))
scalingFactor = 0.01
fidelity = 0.1
xmin = 0.2
ymin = 0.25
zmin = 0.2
xmax = 0.5
ymax = 0.75
zmax = 0.5
ellipsoidDetail = 16

while count < forceArr.shape[1]:

	phi = np.linspace(0,2*np.pi, ellipsoidDetail).reshape(ellipsoidDetail, 1) # the angle of the projection in the xy-plane
	theta = np.linspace(0, np.pi, ellipsoidDetail).reshape(-1, ellipsoidDetail) # the angle from the polar axis, ie the polar angle
	Ax = forceArr[0,count]*scalingFactor
	Ay = forceArr[1,count]*scalingFactor
	Az = forceArr[2,count]*scalingFactor

	x = Ax*np.sin(theta)*np.cos(phi) + np.floor(count/(6*4))%4*fidelity + xmin
	z = Ay*np.sin(theta)*np.sin(phi) + np.floor(count/4)%6*fidelity + zmin
	y = Az*np.cos(theta) + count%4*fidelity + ymin

	surf = ax.plot_surface(x, y, z, color=[((np.floor(count/(6*4))%4*fidelity + xmin)*0.5+0.5),np.floor(count/4)%6*fidelity + zmin,(count%4*fidelity + ymin)*0.5+0.5])
	# ax.set_xlim(0.2,0.5)
	# ax.set_ylim(0.2,0.5)
	# ax.set_zlim(0.2,0.8)
	ax.set_xlim(-1,1)
	ax.set_ylim(-1,1)
	ax.set_zlim(0,1)


	#plt.draw()
	#plt.pause(0.05)
	#plt.cla()
	count += 1

plt.show()