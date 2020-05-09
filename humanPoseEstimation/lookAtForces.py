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

path = np.loadtxt('armPath6.txt')
pathCart = sg.getCartPath(path)
forces = sg.getCartForces(pathCart)
# print(forces)

t = np.arange(np.shape(forces)[0])

xForces = forces[:,0]
# print(xForces)
yForces = forces[:,2]
zForces = forces[:,1]


#Savgol Filter
# window_size = 101
# poly_order = 3
# sg = savgol_filter(forces,window_size,poly_order)

# print(sg)

plt.figure()
#Moving Average Filter - looks like this one's the winner
plt.xlabel("t")
plt.ylabel("Force")
window_size = 15 #KEEP THIS BIG

xForcesMA = movingAverage(xForces,window_size)
yForcesMA = movingAverage(yForces,window_size)
zForcesMA = movingAverage(zForces,window_size)
# plt.plot(xForces[len(xForces)-len(yMA):],yMA)
plt.plot(t[len(t)-len(xForcesMA):],xForcesMA,label="Moving Average Fx")
# plt.plot(t[len(t)-len(yForcesMA):],yForcesMA,label="Moving Average Fy")
# plt.plot(t[len(t)-len(zForcesMA):],zForcesMA,label="Moving Average Fz")

# print("shape = ",np.shape(xForcesMA))

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
zTemp = zForcesMA
# zTemp[zTemp < 0.1] = 1
plt.plot(pathCart[window_size-1:,0],zForcesMA/xForcesMA,'.')

polyOrder = 4
bestFitX = np.polyfit(pathCart[window_size-1:,0],zForcesMA/xForcesMA,polyOrder)
pX = np.poly1d(bestFitX)
xpX= np.linspace(-1,1,100)
plt.plot(xpX,pX(xpX),'--')

#put forces into bins
# xForcesMA[xForcesMA < 1] = 1

numBins = 51
mostForceThresh = 0.5

#X estimate
xzvt = np.array([pathCart[window_size-1:,0],xForcesMA/zForcesMA])
zxvt = np.array([pathCart[window_size-1:,0],zForcesMA/xForcesMA])
print(xzvt)
bins = np.linspace(-0.5,0.5,numBins)
xzvt[0,:] = np.digitize(xzvt[0,:],bins)

binSum = np.zeros(len(bins))
i = 0
while i < len(bins):
	currentBin = np.argwhere([(xzvt[0,:]==i),(xzvt[1,:] > np.quantile(xzvt[:,1],mostForceThresh))]) #get upper mostForceThresh% values from each bin
	binSum[i] = np.sum(xzvt[1,currentBin])/(np.count_nonzero([xzvt[0,:]==i,(zxvt[1,:] > np.quantile(xzvt[:,1],mostForceThresh))]))#total number of times the bin is used
	i += 1

print(binSum)
print(" bins ", bins)
print(" binSum ", binSum)



plt.figure()
plt.plot(bins,binSum,'b.')
plt.xlabel("x (m)")
plt.title("Fx/Fz vs x for dataset 2")
plt.ylabel("Fx/Fz (normalized)")
polyOrder = 2
bestFitxzvt = np.polyfit(bins[np.logical_not(np.isnan(binSum))],binSum[np.logical_not(np.isnan(binSum))],polyOrder)
pbins = np.poly1d(bestFitxzvt)
xpbins= np.linspace(-0.5,0.5,100)
plt.plot(xpbins,pbins(xpbins),'--')
# plt.axis([-0.5,0.5,0,2])

critX = pbins.deriv().r
r_critX = critX[critX.imag==0].real
testX = pbins.deriv(2)(r_critX)
x_maxX = r_critX[testX>0]
y_min = pbins(x_maxX)
print("shoulder x is = ", x_maxX)



#Z estimate
zxvt = np.array([pathCart[window_size-1:,2],zForcesMA/xForcesMA])
# print(zvt)
bins = np.linspace(-0.5,0.5,numBins)
zxvt[0,:] = np.digitize(zxvt[0,:],bins)

binSum = np.zeros(len(bins))
i = 0
while i < len(bins):
	currentBin = np.argwhere([(zxvt[0,:]==i),(zxvt[1,:] > np.quantile(zxvt[:,1],mostForceThresh))]) #get upper mostForceThresh% values from each bin
	binSum[i] = np.sum(zxvt[1,currentBin])/(np.count_nonzero([zxvt[0,:]==i,(zxvt[1,:] > np.quantile(zxvt[:,1],mostForceThresh))]))#total number of times the bin is used
	i += 1

print(binSum)
print(" bins ", bins)
print(" binSum ", binSum)



plt.figure()
plt.plot(bins,binSum,'b.')
plt.title("Fz/Fx vs z for dataset 2")
plt.xlabel("z (m)")
plt.ylabel("Fz/Fx (normalized)")
polyOrder = 2
bestFitzxvt = np.polyfit(bins[np.logical_not(np.isnan(binSum))],binSum[np.logical_not(np.isnan(binSum))],polyOrder)
pbins = np.poly1d(bestFitzxvt)
xpbins= np.linspace(-0.5,0.5,100)
plt.plot(xpbins,pbins(xpbins),'--')
# plt.axis([-0.5,0.5,0,2])

critX = pbins.deriv().r
r_critX = critX[critX.imag==0].real
testX = pbins.deriv(2)(r_critX)
x_maxX = r_critX[testX<0]
y_min = pbins(x_maxX)
print("shoulder z is = ", x_maxX)


# xAnswer = bins[np.argwhere(binSum[:] == np.nanmax(binSum))]
# print(xAnswer)

#OLD POLYFIT STRATEGY
# polyOrder = 30
# bestFitX = np.polyfit(pathCart[:,0],t,polyOrder)
# pX = np.poly1d(bestFitX)
# xpX= np.linspace(-1,1,100)
# plt.plot(t,pX(t),'--')
plt.pause(20)
plt.draw()