import numpy as np
from numpy import convolve 
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shoulderGuesser import shoulderGuesser
from scipy.signal import savgol_filter
from scipy.interpolate import interp1d, RegularGridInterpolator
from ellipse import *
from matplotlib.patches import Ellipse


def movingAverage(values,window):
	weights = np.repeat(1.0,window)/window
	sma = np.convolve(values,weights,'valid')
	return sma

sg = shoulderGuesser()

path = np.loadtxt('armPath5.txt')
pathCart = sg.getCartPath(path)
print(pathCart)
forces = sg.getCartForces(pathCart) #TODO make forces both + AND -
print(forces)

t = np.arange(np.shape(forces)[0])

xForces = forces[:,0]
# print(xForces)
yForces = forces[:,2]
zForces = forces[:,1]


plt.figure()
#Moving Average Filter - looks like this one's the winner
plt.xlabel("t")
plt.ylabel("Force")
window_size = 30 #KEEP THIS BIG

xForcesMA = movingAverage(xForces,window_size)
yForcesMA = movingAverage(yForces,window_size)
zForcesMA = movingAverage(zForces,window_size)
# plt.plot(xForces[len(xForces)-len(yMA):],yMA)
plt.plot(t[len(t)-len(xForcesMA):],xForcesMA,label="Moving Average Fx")
plt.plot(t[len(t)-len(yForcesMA):],yForcesMA,label="Moving Average Fy")
plt.plot(t[len(t)-len(zForcesMA):],zForcesMA,label="Moving Average Fz")


#Cubic Interpolation of moving average data
f2 = interp1d(t[len(t)-len(xForcesMA):],xForcesMA,kind='cubic')
tNew = np.linspace(len(t)-len(xForcesMA),len(xForces),num=100,endpoint=True)
# cubicInterp, = plt.plot(tNew, f2(tNew), '--')

# plt.axis([0,np.shape(forces)[0],0,5])

#RAW DATA IS NOISY AF
ptsX, = plt.plot(t,forces[:,0],'r.', label='Raw Force Data')

plt.legend(loc='best')


#move points in path to grid

#draw ellipses from points
fig = plt.figure()
ax = fig.add_subplot()
plt.xlabel('x')
plt.ylabel('z')

xzvt = np.array([pathCart[window_size-1:,0],xForcesMA/zForcesMA])

point = 0

#bin path data into points
numStps = 20
pathCart[:,0] = np.floor(pathCart[:,0]*numStps)/numStps
pathCart[:,2] = np.floor(pathCart[:,2]*numStps)/numStps

while point < np.shape(pathCart)[0]:

	#find out if there are multiple ellipses at grid point
	# if len(np.argwhere(pathCart[:,0] == pathCart[point,0] and pathCart[:,2] == pathCart[point,2])): 
	# print(pathCart[point,0],pathCart[point,2])
	# print( np.intersect1d((np.argwhere(pathCart[:,0] == pathCart[point,0])), np.argwhere(pathCart[:,2] == pathCart[point,2])) )

	args = np.intersect1d((np.argwhere(pathCart[:,0] == pathCart[point,0])), np.argwhere(pathCart[:,2] == pathCart[point,2]))
	print(point)
	print(len(xForces))
	print(args)
	xForcesMA[point] = np.sum(xForcesMA[args])/len(args)
	zForcesMA[point] = np.sum(zForcesMA[args])/len(args) #out of range axis 0

	try:
		drawEllipse(ax,pathCart[point,0], pathCart[point,2] ,0.00025,0.0000025,np.arctan((zForcesMA[point]/xForcesMA[point]))) #+ np.pi/2)

	except:
		pass
	point += 1


#--------------------------------------
# #set up grid
# fidelity = 0.1 #how far apart each point should be
# x = np.arange(-0.7,0.7,fidelity)
# z = np.arange(-0.7,0.7,fidelity)

# for xstep in x:
# 	for zstep in z:
# 		print(xstep,zstep)
# 		#find points in grid where there are more than one ellipse
# 		ans1 = np.argwhere((abs(xstep - np.floor(11*pathCart[:,0])/11) < 0.01)) #and (abs(zstep - np.floor(11*pathCart[:,2])/11) < 0.01))
# 		# print(ans1)
# 		ans2 = np.argwhere((abs(zstep - np.floor(11*pathCart[:,2])/11) < 0.01))
# 		# print(ans2)
# 		print(np.intersect1d(ans1, ans2))


#--------------------------------------
numBins = 51
mostForceThresh = 0.5

#X estimate
# print(xzvt)
bins = np.linspace(-0.5,0.5,numBins)
xzvt[0,:] = np.digitize(xzvt[0,:],bins)

binSum = np.zeros(len(bins))
i = 0
while i < len(bins):
	currentBin = np.argwhere([(xzvt[0,:]==i),(xzvt[1,:] > np.quantile(xzvt[:,1],mostForceThresh))]) #get upper mostForceThresh% values from each bin
	binSum[i] = np.sum(xzvt[1,currentBin])/(np.count_nonzero([xzvt[0,:]==i,(xzvt[1,:] > np.quantile(xzvt[:,1],mostForceThresh))]))#total number of times the bin is used
	i += 1


plt.figure()
plt.plot(bins,binSum,'b.')
plt.xlabel("x (m)")
plt.title("Fx/Fz vs x for dataset 2")
plt.ylabel("Fx/Fz (normalized)")


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


plt.figure()
plt.plot(bins,binSum,'b.')
plt.title("Fz/Fx vs z for dataset 2")
plt.xlabel("z (m)")
plt.ylabel("Fz/Fx (normalized)")


plt.pause(100)
plt.draw()