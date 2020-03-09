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

XZarr = np.zeros([2,forces.shape[1]])

while count < forces.shape[1]:
	xpt = xForces[0,count]
	ypt = xForces[3,count]/xForces[5,count]
	XZarr[:,count] = [xpt, ypt]
	count += 1

XZarr[1,:] = np.clip(XZarr[1,:],(np.mean(XZarr[1,:])-3*np.std(XZarr[1,:])),(np.mean(XZarr[1,:])+3*np.std(XZarr[1,:])))
plt.plot(XZarr[0,:],XZarr[1,:],'bo')
plt.autoscale(enable=True, axis='y')
plt.draw()

#draw Y/X pos-force relationship
fig2 = plt.figure(2)
ax = fig2.add_subplot()
ax.set_xlabel('y')
ax.set_ylabel('force magnitude ratio y/x')
yForces = forces[forces[:,1].argsort()]
count = 0

YXarr = np.zeros([2,forces.shape[1]])

while count < forces.shape[1]:
	xpt = yForces[1,count]
	ypt = yForces[4,count]/yForces[3,count]
	YXarr[:,count] = [xpt, ypt]
	count += 1

#YXarr[1,:] = np.clip(YXarr[1,:],(np.mean(YXarr[1,:])-.5*np.std(YXarr[1,:])),(np.mean(YXarr[1,:])+.5*np.std(YXarr[1,:])))
plt.plot(YXarr[0,:],YXarr[1,:],'bo')
plt.autoscale(enable=True, axis='y')
plt.draw()















# #draw X/Y pos-force relationship
# fig2 = plt.figure(2)
# ax = fig2.add_subplot()
# ax.set_xlabel('y')
# ax.set_ylabel('force magnitude ratio y/x')
# xForces = forces[forces[:,1].argsort()]
# print(xForces.shape)
# count = 0
# YXarr = np.zeros([2,forces.shape[1]])

# while count < forces.shape[1]:
# 	xpt = xForces[1,count]
# 	ypt = xForces[4,count]/xForces[3,count]
# 	YXarr[:,count] = [xpt, ypt] 
# 	count += 1

# YXarr = np.clip(YXarr,(np.mean(YXarr)-3*np.std(YXarr)),(np.mean(YXarr)+3*np.std(YXarr)))



# plt.plot(YXarr[0,:],YXarr[1,:],'bo')
# plt.autoscale(enable=True, axis='y')
# plt.draw()






# #draw Y/Z pos-force relationship
# fig3 = plt.figure(3)
# ax = fig3.add_subplot()
# ax.set_xlabel('z')
# ax.set_ylabel('force magnitude ratio z/y')
# xForces = forces[forces[:,2].argsort()]
# print(xForces.shape)
# count = 0
# while count < forces.shape[1]:
# 	xpt = xForces[1,count]
# 	ypt = xForces[5,count]/xForces[4,count]
	
# 	plt.plot(xpt,ypt,'bo')
# 	count += 1
# plt.autoscale(enable=True, axis='y')
# plt.draw()

# print(XZarr)

plt.pause(30)