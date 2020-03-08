import numpy as np
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D

forces = np.loadtxt('forces.txt')

#draw X/Z pos-force relationship
fig1 = plt.figure(1)
ax = fig1.add_subplot()
ax.set_xlabel('x')
ax.set_ylabel('force magnitude ratio x/z')
xForces = forces[forces[:,0].argsort()]
print(xForces.shape)
count = 0
while count < forces.shape[1]:
	xpt = xForces[0,count]
	ypt = xForces[3,count]/xForces[5,count]
	
	plt.plot(xpt,ypt,'bo')
	count += 1
plt.autoscale(enable=True, axis='y')
plt.draw()

#draw X/Y pos-force relationship
fig2 = plt.figure(2)
ax = fig2.add_subplot()
ax.set_xlabel('y')
ax.set_ylabel('force magnitude ratio y/x')
xForces = forces[forces[:,1].argsort()]
print(xForces.shape)
count = 0
while count < forces.shape[1]:
	xpt = xForces[0,count]
	ypt = xForces[4,count]/xForces[3,count]
	
	plt.plot(xpt,ypt,'bo')
	count += 1
plt.autoscale(enable=True, axis='y')
plt.draw()

#draw Y/Z pos-force relationship
fig3 = plt.figure(3)
ax = fig3.add_subplot()
ax.set_xlabel('z')
ax.set_ylabel('force magnitude ratio z/y')
xForces = forces[forces[:,2].argsort()]
print(xForces.shape)
count = 0
while count < forces.shape[1]:
	xpt = xForces[1,count]
	ypt = xForces[5,count]/xForces[4,count]
	
	plt.plot(xpt,ypt,'bo')
	count += 1
plt.autoscale(enable=True, axis='y')
plt.draw()

plt.pause(30)