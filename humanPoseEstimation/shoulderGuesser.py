import numpy as np 
from inertiaEstimator import inertiaEstimator
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.preprocessing import minmax_scale #unused 

class shoulderGuesser:

	ie = inertiaEstimator()
	# path = np.zeros(3)
	path = np.loadtxt('armPath5.txt')

	def getCartPath(self,path=path):
		"""converts trajectory from joint space to cartesian space"""
		i = 0
		pathCart = np.zeros(np.shape(path))
		while i < np.shape(path)[0]: #length of path 
			pathCart[i,:] = self.ie.joint2Cartesian(path[i,0],path[i,1],path[i,2])
			i += 1

		return(pathCart)

	def getCartForces(self,pathCart):
		"""gets forces exerted in cartesian space for duration of path"""
		#TODO normalize by timestep (not sure how important)

		velCart = np.zeros(np.shape(pathCart))
		i = 1
		while i < (np.shape(pathCart)[0]):
			velCart[i] = pathCart[i]-pathCart[i-1]
			i += 1
		#F = ma
		#m is arbitary since user is pushing around a virtual mass, set to 1 for simplicity
		cartForces = np.zeros(np.shape(velCart))
		# cartForces = cartForces
		j = 1
		while j < (np.shape(velCart)[0]):
			cartForces[j] = (velCart[j] - velCart[j-1])#**2
			# cartForces[cartForces == 0] = 0.0001
			# cartForces[j] = (abs(cartForces[j]))**0.25 # +/- direction of cart. components not important
			j += 1
		return(cartForces)

	def estimateFromForces(self,pathCart):

		def movingAverage(values,window):
			weights = np.repeat(1.0,window)/window
			sma = np.convolve(values,weights,'valid')
			return sma

		numBins = 51
		mostForceThresh = 0.5 #only look at data of (mostForceThresh*100) percentile
		window_size = 15 #for moving average filter

		forces = self.getCartForces(pathCart)
		t = np.arange(np.shape(forces)[0])
		xForces = forces[:,0]
		# print(xForces)
		yForces = forces[:,2]
		zForces = forces[:,1]

		xForcesMA = movingAverage(xForces,window_size)
		yForcesMA = movingAverage(yForces,window_size)
		zForcesMA = movingAverage(zForces,window_size)

		#estimate x position
		xzvt = np.array([pathCart[window_size-1:,0],xForcesMA/zForcesMA])
		# print(xzvt)
		bins = np.linspace(-0.5,0.5,numBins)
		xzvt[0,:] = np.digitize(xzvt[0,:],bins)

		binSum = np.zeros(len(bins))
		i = 0
		while i < len(bins):
			currentBin = np.argwhere([(xzvt[0,:]==i),(xzvt[1,:] > np.quantile(xzvt[:,1],mostForceThresh))]) #get upper mostForceThresh% values from each bin
			binSum[i] = np.sum(xzvt[1,currentBin])/(np.count_nonzero([xzvt[0,:]==i,(xzvt[1,:] > np.quantile(xzvt[:,1],mostForceThresh))]))#total number of times the bin is used
			i += 1

		print(binSum)

		polyOrder = 2
		bestFitxzvt = np.polyfit(bins[np.logical_not(np.isnan(binSum))],binSum[np.logical_not(np.isnan(binSum))],polyOrder)
		pbins = np.poly1d(bestFitxzvt)
		xpbins= np.linspace(-0.5,0.5,100)

		critX = pbins.deriv().r
		r_critX = critX[critX.imag==0].real
		testX = pbins.deriv(2)(r_critX)
		x_maxX = r_critX[testX>0]
		y_min = pbins(x_maxX)
		print("shoulder x is = ", x_maxX)

		
		#estimate z position
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

		polyOrder = 2
		bestFitzxvt = np.polyfit(bins[np.logical_not(np.isnan(binSum))],binSum[np.logical_not(np.isnan(binSum))],polyOrder)
		pbins = np.poly1d(bestFitzxvt)
		xpbins= np.linspace(-0.5,0.5,100)

		critZ = pbins.deriv().r
		r_critZ = critZ[critZ.imag==0].real
		testZ = pbins.deriv(2)(r_critZ)
		x_maxZ = r_critZ[testZ<0]
		y_min = pbins(x_maxZ)
		print("shoulder z is = ", x_maxZ)		


		#OLD VERSION WITHOUT MOVING AVERAGE AND BINNING
		# forcesCart = self.getCartForces(pathCart)
		# xForces = forcesCart[forcesCart[:,0].argsort()]
		# polyOrder = 4 #start with 2nd order, try again and again until there is a negative coeffieienct on largest term
		# bestFitX = np.polyfit(pathCart[:,0],forcesCart[:,0],polyOrder)
		# # print(bestFitX)
		# pX = np.poly1d(bestFitX)
		# xpX= np.linspace(-1,1,100)
		# critX = pX.deriv().r
		# r_critX = critX[critX.imag==0].real
		# testX = pX.deriv(2)(r_critX)
		# x_maxX = r_critX[testX<0]
		# y_min = pX(x_maxX)
		# # print("shoulder x is = ", max(x_maxX, key=abs))

		# yForces = forcesCart[forcesCart[:,2].argsort()]
		# polyOrder = 2
		# bestFitY = np.polyfit(pathCart[:,2],forcesCart[:,2],polyOrder)
		# # print(bestFitY)
		# pY = np.poly1d(bestFitY)
		# xpY = np.linspace(-1,1,100)
		# critY = pY.deriv().r
		# r_critY = critY[critY.imag==0].real
		# testY = pY.deriv(2)(r_critY)
		# x_maxY = r_critY[testY<0]
		# y_min = pY(x_maxY)
		# # print("shoulder y is = ", max(x_maxY, key=abs))

		# zForces = forcesCart[forcesCart[:,1].argsort()]
		# polyOrder = 2
		# bestFitZ = np.polyfit(pathCart[:,1],forcesCart[:,1],polyOrder)
		# # print(bestFitZ)
		# pZ = np.poly1d(bestFitZ)
		# xpZ = np.linspace(-1,1,100)
		# critZ = pZ.deriv().r
		# r_critZ = critZ[critZ.imag==0].real
		# testZ = pZ.deriv(2)(r_critZ)
		# x_maxZ = r_critZ[testZ<0]
		# y_min = pY(x_maxZ)
		# # print("shoulder Z is = ", max(x_maxZ, key=abs))

		#assume constant shoulder heighy x_maxY
		x_maxY = 0.2

		bestEst = np.array([[x_maxX,x_maxY,x_maxZ]])
		print("best estimate from forces = ",bestEst)
		return(bestEst)

	def estimateFromKinematics(self,pathCart,points):
		"""The goal here is to lower the overall fitness of a point guess 
		   if the end effector frequently gets out of reach"""

		punsiher = 0.9
		fitness = np.ones(np.shape(points)[1])

		step = 0
		point = 0
		larm = 0.635 #length of human arm
		# larm = 0.3
		while step < np.shape(pathCart)[0]:
			while point < np.shape(points)[1]:
				d = np.linalg.norm(points[:,point]-pathCart[step,:])
				# print(d)
				if d > larm:
					fitness[point] = fitness[point] * punsiher
				point += int(np.floor(20*np.random.rand()))
			step += 10
			point = 0

		#TODO return weight
		fitness = 1-fitness
		return(fitness)

if __name__ == "__main__":
	print("oooweee")
	
	mostFitThresh = 1
	leastFitThresh = 1.1
	forceWeight = 0.5
	kinematicsWeight = 1

	sg = shoulderGuesser()
	pathCart = sg.getCartPath()
	# print(pathCart)

	#estimate of shoulder position from FORCE MODEL
	#force model gives a slow big picture estimate of where it thinks the human could be while KM quickly rules out some solutions
	fromForce = sg.estimateFromForces(pathCart) #not a normal data structure becasue one of the solutions had multiple local maxima
	# print(fromForce[0])
	print(fromForce[0])
	forceEst = np.zeros(3)
	# forceEst[0] = np.argmax(abs(fromForce[0][0][1]))
	forceEst[0] = fromForce[0][0][0]
	# print(forceEst[0])
	forceEst[1] = fromForce[0][1]
	forceEst[2] = fromForce[0][2]


	fig = plt.figure()
	ax = fig.add_subplot(111,xlim=(-1,1), ylim=(-1,1), zlim=(-1,1), projection='3d', autoscale_on=False)
	ax.set_xlabel('x')
	ax.set_ylabel('z')
	ax.set_zlabel('y')

	#init points
	numPts = 100
	p = np.zeros([4,numPts])
	p[:3,:] = np.random.rand(3,numPts) #give each point a random starting value for x y and z
	p[:3,:] = np.interp(p[:3,:],[0,1],[-1,1]) #map to size of workspace -1 to 1 in x y z
	
	count = 0
	while count < 100:

		#estimate of shoulder from kinematic model
		fromKinematics = sg.estimateFromKinematics(pathCart,p[:3,:])

		# print(fromKinematics)

		distF = np.zeros(numPts)
		k = 0
		while k < numPts:
			distF[k] = np.linalg.norm(p[:3,k]-forceEst)
			k += 1

		distF = np.interp(distF,[0,2],[0,1])
		colors = np.zeros([3,numPts])
		colors[:,:] = distF
		#set cost p[3] to sum of the two cost metrics
		p[3,:] = distF**forceWeight + fromKinematics**kinematicsWeight
		avg = np.average(p[3,:])
		
		#color based on force Fitness kina inefficient though
		# m = 0
		# while m < numPts:
		# 	pts, = plt.plot([p[0,m]],[p[1,m]],[p[2,m]],'.',color=colors[:,m])
		# 	m += 1
		pts, = plt.plot(p[0,:],p[2,:],p[1,:],'b.')

		#get rid of the least fit particles
		unfit = np.argwhere(p[3,:]>(leastFitThresh*avg))
		mostFit = np.argwhere(p[3,:]<mostFitThresh*avg) #take note of most fit
		worstOfTheBest = np.max(p[3,mostFit])
		invFit = worstOfTheBest	- p[3,:] #temporarily rank fitness as higher being better so I can do roulette wheel random selection
		n = 0
		while n < np.shape(unfit)[0]:
			#TODO resample closer to more fit particles
			randomVal = np.random.rand()*np.sum(invFit)
			tempFitSum = 0
			countVar = 0
			while tempFitSum < randomVal:
				tempFitSum = tempFitSum	+ invFit[mostFit[countVar]]
				countVar += 1	

			p[:3,unfit[n]] = p[:3,mostFit[countVar]] + np.random.randn(3,1)*0.2 #resample around fit points and add some noise
			# print(p[:3,unfit[n]])
			n = n+1

		# p[:3,:] = np.interp(p[:3,:],[0,1],[-1,1])
		plt.draw()
		plt.pause(0.5)
		pts.remove()
		count += 1

	# forcesCart = sg.getCartForces(pathCart)
	# forcesCart[:,0] = np.power(forcesCart[:,0],0.1)
	# print(forcesCart)
	# forcesColor = minmax_scale(forcesCart[:,:],feature_range=(0,1),axis=0)

	# #force vs X
	# figX = plt.figure(1)
	# axX = figX.add_subplot()
	# plt.axis([-0.5,0.5,0,1])
	# axX.set_xlabel('x')
	# axX.set_ylabel('force magnitude in x direction')

	# xForces = forcesCart[forcesCart[:,0].argsort()]
	# axX.plot(pathCart[:,0],forcesCart[:,0],'b.')
	# polyOrder = 4 #start with 2nd order, try again and again until there is a negative coeffieienct on largest term
	# bestFitX = np.polyfit(pathCart[:,0],forcesCart[:,0],polyOrder)
	# print(bestFitX)
	# pX = np.poly1d(bestFitX)
	# xpX= np.linspace(-1,1,100)
	# axX.plot(xpX,pX(xpX),'--')

	# critX = pX.deriv().r
	# r_critX = critX[critX.imag==0].real
	# testX = pX.deriv(2)(r_critX)
	# x_maxX = r_critX[testX<0]
	# y_min = pX(x_maxX)

	# print("shoulder x is = ", max(x_maxX, key=abs))

	# #Y vs Force
	# figY = plt.figure(3)
	# axY = figY.add_subplot()
	# plt.axis([-0.5,0.5,0,1])
	# axY.set_xlabel('Y')
	# axY.set_ylabel('force magnitude in Y direction')

	# yForces = forcesCart[forcesCart[:,2].argsort()]
	# axY.plot(pathCart[:,2],forcesCart[:,2],'b.')
	# polyOrder = 2
	# bestFitY = np.polyfit(pathCart[:,2],forcesCart[:,2],polyOrder)
	# print(bestFitY)
	# pY = np.poly1d(bestFitY)
	# xpY = np.linspace(-1,1,100)

	# axY.plot(xpY,pY(xpY),'--')
	# plt.savefig('test.png')

	# #Z vs Force
	# figZ = plt.figure(2)
	# axZ = figZ.add_subplot()
	# plt.axis([-0.5,0.5,0,1])
	# axZ.set_xlabel('z')
	# axZ.set_ylabel('force magnitude in z direction')

	# zForces = forcesCart[forcesCart[:,1].argsort()]
	# axZ.plot(pathCart[:,1],forcesCart[:,1],'b.')
	# polyOrder = 2
	# bestFitZ = np.polyfit(pathCart[:,1],forcesCart[:,1],polyOrder)
	# print(bestFitZ)
	# pZ = np.poly1d(bestFitZ)
	# xpZ = np.linspace(-1,1,100)

	# axZ.plot(xpZ,pZ(xpZ),'--')



	#End Effector Viz
	# fig0 = plt.figure(0)
	# ax0 = fig0.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=True)
	# ax0.grid(False)
	# ax0.set_xlabel('x')
	# ax0.set_ylabel('z')
	# ax0.set_zlabel('y')
	# ax0.set_xlim(-0.5,0.5)
	# ax0.set_ylim(-0.2,0.5)
	# ax0.set_zlim(0,0.5)
	# scale = 0.9
	# count = 0

	# while count < np.shape(pathCart)[0]:
		# color = [forcesColor[count,0]**scale,forcesColor[count,2]**scale,forcesColor[count,1]**scale]
		# color = [abs((forcesCart[count,0])**scale),abs((forcesCart[count,2])**scale),abs((forcesCart[count,1])**scale)]
		# EE, = ax0.plot([pathCart[count,0]],[pathCart[count,2]],[pathCart[count,1]],'o',color=color)
		## xpts = axX.plot([pathCart[count,0],forcesCart[count,0]],'bo')
		# plt.pause(0.001)
		# count += 1
		# EE.remove()



	