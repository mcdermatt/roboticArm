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

#Savgol Filter
# window_size = 101
# poly_order = 3
# sg = savgol_filter(forces,window_size,poly_order)

# print(sg)

plt.figure()
#Moving Average Filter
window_size = 30
xForcesMA = movingAverage(xForces,window_size)
# plt.plot(xForces[len(xForces)-len(yMA):],yMA)
plt.plot(t[len(t)-len(xForcesMA):],xForcesMA,label="Moving Average Filter")

#Cubic Interpolation of moving average data
f2 = interp1d(t[len(t)-len(xForcesMA):],xForcesMA,kind='cubic')
tNew = np.linspace(len(t)-len(xForcesMA),len(xForces)-1,num=41,endpoint=True)
cubicInterp, = plt.plot(tNew, f2(tNew), '--')

# plt.axis([0,np.shape(forces)[0],0,5])
#RAW DATA IS NOISY AF
ptsX, = plt.plot(t,forces[:,0],'r.', label='Raw Force Data')




#OLD POLYFIT STRATEGY
# polyOrder = 30
# bestFitX = np.polyfit(pathCart[:,0],t,polyOrder)
# pX = np.poly1d(bestFitX)
# xpX= np.linspace(-1,1,100)
# plt.plot(t,pX(t),'--')
plt.legend(loc='best')
plt.pause(10)
plt.draw()