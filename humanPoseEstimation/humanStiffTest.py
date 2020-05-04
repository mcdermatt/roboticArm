import numpy as np 
from inertiaEstimator import inertiaEstimator
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from shoulderGuesser import shoulderGuesser


sg = shoulderGuesser()
#path 4 - standing far to the posive x
#path 3 - standing ~= x=0
path = np.loadtxt('armPath4.txt')
pathCart = sg.getCartPath(path=path)

forcesCart = sg.getCartForces(pathCart)

#make forces discrete
forcesCart[:,:] = (np.floor(forcesCart[:,:]*100))*0.01

pathCart[:,:] =  (np.floor(pathCart[:,:]*100))*0.01

# forcesCart[:,0] = np.power(forcesCart[:,0],0.1) 
# print(forcesCart)

#force vs X
figX = plt.figure(1)
axX = figX.add_subplot()
plt.axis([-1,1,0,5])
axX.set_xlabel('x')
axX.set_ylabel('force ratio Fx/Fx vs x position')

# xForces = forcesCart[forcesCart[:,0].argsort()]
# print(forcesCart[:,1])
# yForces = forcesCart[forcesCart[:,2].argsort()]
# zForces = forcesCart[forcesCart[:,1].argsort()]

z = np.argwhere(forcesCart[:,2] < 0.1)
forcesCart[z,2] =  1
estRadius = 1 #the radius of the circle on wich the human is most likely standing
# ang = np.arccos(pathCart[:,0]/estRadius)

numPts = 100
conf = np.zeros(numPts)
for x in np.linspace(0,1,numPts):
	#TODO - 
	ang = np.arccos(pathCart[:,0]-x/estRadius)
	FxFz = abs((forcesCart[:,0]/(np.sin(ang)))/(forcesCart[:,2]/(np.cos(ang))))
	# FxFz = ((forcesCart[:,0])/(forcesCart[:,2]))/np.sin(ang)
	# color = [abs(x),abs(x),0.5]

	points, = axX.plot(pathCart[:,0],FxFz,'b.')
	polyOrder = 4 #start with 2nd order, try again and again until there is a negative coeffieienct on largest term
	bestFitX = np.polyfit(pathCart[:,0],FxFz,polyOrder)
	print(bestFitX)
	pX = np.poly1d(bestFitX)
	xpX= np.linspace(-1,1,100)
	axX.plot(xpX,pX(xpX),'--')

	plt.pause(0.1)
	plt.draw()
	points.remove()
plt.pause(5)
# critX = pX.deriv().r
# r_critX = critX[critX.imag==0].real
# testX = pX.deriv(2)(r_critX)
# x_maxX = r_critX[testX<0]
# y_min = pX(x_maxX)

# print("shoulder x is = ", max(x_maxX, key=abs))

# #Y vs Force
# figY = plt.figure(3)
# axY = figY.add_subplot()
# # plt.axis([-0.5,0.5,0,1])
# axY.set_xlabel('Y')
# axY.set_ylabel('force magnitude in Y direction')

# forcesCart[forcesCart[:,0] < 0.1] =1

# axY.plot(pathCart[:,2],forcesCart[:,2]/forcesCart[:,0],'b.')
# polyOrder = 2
# bestFitY = np.polyfit(pathCart[:,2],forcesCart[:,2]/forcesCart[:,0],polyOrder)
# print(bestFitY)
# pY = np.poly1d(bestFitY)
# xpY = np.linspace(-1,1,100)

# axY.plot(xpY,pY(xpY),'--')
# plt.savefig('test.png')

# #Z vs Force
# figZ = plt.figure(2)
# axZ = figZ.add_subplot()
# # plt.axis([-0.5,0.5,0,1])
# axZ.set_xlabel('z')
# axZ.set_ylabel('force magnitude in z direction')


# axZ.plot(pathCart[:,1],forcesCart[:,1]/forcesCart[:,2],'b.')
# polyOrder = 2
# bestFitZ = np.polyfit(pathCart[:,1],forcesCart[:,1]/forcesCart[:,2],polyOrder)
# print(bestFitZ)
# pZ = np.poly1d(bestFitZ)
# xpZ = np.linspace(-1,1,100)

# axZ.plot(xpZ,pZ(xpZ),'--')


# plt.pause(30)
# plt.draw()