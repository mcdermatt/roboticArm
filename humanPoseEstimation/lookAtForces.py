import numpy as np
from numpy import convolve 
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shoulderGuesser import shoulderGuesser
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d

def movingAverage(values,window):
	weights = np.repeat(1.0,window)/window
	sma = np.convolve(values,weights,'valid')
	return sma

sg = shoulderGuesser()

path = np.loadtxt('armPath3.txt')
pathCart = sg.getCartPath(path)
forces = sg.getCartForces(pathCart)
# print(forces)

t = np.arange(np.shape(forces)[0])

xForces = forces[:,0]
yForces = forces[:,2]
zForces = forces[:,1]


#Savgol Filter
# window_size = 101
# poly_order = 3
# sg = savgol_filter(forces,window_size,poly_order)

# print(sg)

plt.figure()
#Moving Average Filter - looks like this one's the winner
window_size = 100

xForcesMA = movingAverage(xForces,window_size)
yForcesMA = movingAverage(yForces,window_size)
zForcesMA = movingAverage(zForces,window_size)
# plt.plot(xForces[len(xForces)-len(yMA):],yMA)
plt.plot(t[len(t)-len(xForcesMA):],xForcesMA,label="Moving Average Fx")
# plt.plot(t[len(t)-len(yForcesMA):],yForcesMA,label="Moving Average Fy")
# plt.plot(t[len(t)-len(zForcesMA):],zForcesMA,label="Moving Average Fz")


print("shape = ",np.shape(xForcesMA))

#Cubic Interpolation of moving average data
f2 = interp1d(t[len(t)-len(xForcesMA):],xForcesMA,kind='cubic')
tNew = np.linspace(len(t)-len(xForcesMA),len(xForces),num=100,endpoint=True)
# cubicInterp, = plt.plot(tNew, f2(tNew), '--')

# plt.axis([0,np.shape(forces)[0],0,5])

#RAW DATA IS NOISY AF
ptsX, = plt.plot(t,forces[:,0],'r.', label='Raw Force Data')

#TODO look into spline
plt.legend(loc='best')

plt.figure()
plt.axis([-0.5,0.5,0,3])
plt.plot(pathCart[window_size-1:,0],xForcesMA/zForcesMA,'.')

polyOrder = 4
bestFitX = np.polyfit(pathCart[window_size-1:,0],xForcesMA/zForcesMA,polyOrder)
pX = np.poly1d(bestFitX)
xpX= np.linspace(-1,1,100)
plt.plot(xpX,pX(xpX),'--')

#OLD POLYFIT STRATEGY
# polyOrder = 30
# bestFitX = np.polyfit(pathCart[:,0],t,polyOrder)
# pX = np.poly1d(bestFitX)
# xpX= np.linspace(-1,1,100)
# plt.plot(t,pX(t),'--')
plt.pause(10)
plt.draw()