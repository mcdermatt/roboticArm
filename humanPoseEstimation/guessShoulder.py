import numpy as np
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D

forces = np.loadtxt('forces.txt')

#draw xpos-force relationship
fig1 = plt.figure(1)
ax = fig1.add_subplot()
ax.set_xlabel('x')
ax.set_ylabel('force magnitude')
xForces = forces[forces[:,0].argsort()]
print(xForces.shape)
count = 0
while count < forces.shape[1]:
	xpt = xForces[0,count]
	ypt = xForces[3,count]
	
	plt.plot(xpt,ypt,'bo')
	count += 1
plt.draw()

#draw ypos-force relationship
fig2 = plt.figure(2)
ax = fig2.add_subplot()
ax.set_xlabel('y')
ax.set_ylabel('force magnitude')

xForces = forces[forces[:,1].argsort()]
print(xForces.shape)

count = 0
while count < forces.shape[1]:
	xpt = xForces[1,count]
	ypt = xForces[4,count]
	
	plt.plot(xpt,ypt,'bo')
	count += 1

#draw zpos-force relationship
fig3 = plt.figure(3)
ax = fig3.add_subplot()
ax.set_xlabel('z')
ax.set_ylabel('force magnitude')

xForces = forces[forces[:,2].argsort()]
print(xForces.shape)

count = 0
while count < forces.shape[1]:
	xpt = xForces[2,count]
	ypt = xForces[5,count]
	
	plt.plot(xpt,ypt,'bo')
	count += 1


plt.draw()

plt.pause(10)