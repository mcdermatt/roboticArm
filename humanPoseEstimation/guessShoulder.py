import numpy as np
import matplotlib.pyplot as plt

#from mpl_toolkits.mplot3d import Axes3D

#forces = np.loadtxt('forces.txt')
forces = np.load('forces2.npy')

#draw X/Z pos-force relationship
fig1 = plt.figure(1)
plt.axis([-1, 1, 0, 10])
ax = fig1.add_subplot()
ax.set_xlabel('x')
ax.set_ylabel('force magnitude ratio x/z')
xForces = forces[forces[:,0].argsort()]
print(xForces.shape)
count = 0

XZarr = np.zeros([2,forces.shape[1]])

while count < forces.shape[1]:
	xpt = xForces[0,count]
	ypt = xForces[3,count]/xForces[5,count]
	XZarr[:,count] = [xpt, ypt]
	count += 1

#clip out points more than 3 standard deviations from mean
#XZarr[1,:] = np.clip(XZarr[1,:],(np.mean(XZarr[1,:])-3*np.std(XZarr[1,:])),(np.mean(XZarr[1,:])+3*np.std(XZarr[1,:])))
plt.plot(XZarr[0,:],XZarr[1,:],'bo')

polyOrder = 2

bestFit = np.polyfit(XZarr[0,:],XZarr[1,:],polyOrder)
print(bestFit)
p = np.poly1d(bestFit)
xp = np.linspace(-1,1,100)

plt.plot(xp,p(xp),'--')

plt.autoscale(enable=False, axis='y')
plt.draw()

#draw Y/X pos-force relationship
fig2 = plt.figure(2)
plt.axis([0, 1, 0, 10])
ax = fig2.add_subplot()
ax.set_xlabel('y')
ax.set_ylabel('force magnitude ratio y/x')
yForces = forces[forces[:,1].argsort()]
count = 0

YXarr = np.zeros([2,forces.shape[1]])

while count < forces.shape[1]:
	xpt = yForces[1,count]
	ypt = yForces[3,count]/yForces[4,count]
	YXarr[:,count] = [xpt, ypt]
	count += 1

#YXarr[1,:] = np.clip(YXarr[1,:],(0-.5*np.std(YXarr[1,:])),(0+.5*np.std(YXarr[1,:])))
plt.plot(YXarr[0,:],YXarr[1,:],'bo')
plt.autoscale(enable=False, axis='y')
plt.draw()

#draw Z/Y pos-force relationship
fig3 = plt.figure(3)
plt.axis([-1, 1, 0, 10])
ax = fig3.add_subplot()
ax.set_xlabel('z')
ax.set_ylabel('force magnitude ratio z/y')
zForces = forces[forces[:,1].argsort()]
count = 0

ZYarr = np.zeros([2,forces.shape[1]])

while count < forces.shape[1]:
	xpt = zForces[2,count]
	ypt = zForces[5,count]/zForces[4,count]
	ZYarr[:,count] = [xpt, ypt]
	count += 1

#YXarr[1,:] = np.clip(YXarr[1,:],(0-.5*np.std(YXarr[1,:])),(0+.5*np.std(YXarr[1,:])))
plt.plot(ZYarr[0,:],ZYarr[1,:],'bo')
plt.autoscale(enable=False, axis='y')
plt.draw()

plt.pause(30)