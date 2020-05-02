import numpy as np 
from inertiaEstimator import inertiaEstimator
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.preprocessing import minmax_scale #unused 

class shoulderGuesser:

	ie = inertiaEstimator()
	# path = np.zeros(3)
	path = np.loadtxt('armPath4.txt')

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
			cartForces[j] = (velCart[j] - velCart[j-1])
			cartForces[cartForces == 0] = 0.0001
			cartForces[j] = (abs(cartForces[j]))**0.25
			# cartForces[j,0] = cartForces[j,0]**0.5
			# cartForces[j,1] = cartForces[j,1]**0.5
			# cartForces[j,2] = cartForces[j,2]**0.5
			j += 1
		return(cartForces)

if __name__ == "__main__":
	print("oooweee")
	
	sg = shoulderGuesser()
	pathCart = sg.getCartPath()
	# print(pathCart)
	forcesCart = sg.getCartForces(pathCart)
	# forcesCart[:,0] = np.power(forcesCart[:,0],0.1)
	# print(forcesCart)
	forcesColor = minmax_scale(forcesCart[:,:],feature_range=(0,1),axis=0)

	#force vs X
	figX = plt.figure(1)
	axX = figX.add_subplot()
	plt.axis([-0.5,0.5,0,1])
	axX.set_xlabel('x')
	axX.set_ylabel('force magnitude in x direction')

	xForces = forcesCart[forcesCart[:,0].argsort()]
	axX.plot(pathCart[:,0],forcesCart[:,0],'b.')
	polyOrder = 4 #start with 2nd order, try again and again until there is a negative coeffieienct on largest term
	bestFitX = np.polyfit(pathCart[:,0],forcesCart[:,0],polyOrder)
	print(bestFitX)
	pX = np.poly1d(bestFitX)
	xpX= np.linspace(-1,1,100)

	axX.plot(xpX,pX(xpX),'--')

	#Y vs Force
	figY = plt.figure(3)
	axY = figY.add_subplot()
	plt.axis([-0.5,0.5,0,1])
	axY.set_xlabel('Y')
	axY.set_ylabel('force magnitude in Y direction')

	yForces = forcesCart[forcesCart[:,2].argsort()]
	axY.plot(pathCart[:,2],forcesCart[:,2],'b.')
	polyOrder = 2
	bestFitY = np.polyfit(pathCart[:,2],forcesCart[:,2],polyOrder)
	print(bestFitY)
	pY = np.poly1d(bestFitY)
	xpY = np.linspace(-1,1,100)

	axY.plot(xpY,pY(xpY),'--')

	#Z vs Force
	figZ = plt.figure(2)
	axZ = figZ.add_subplot()
	plt.axis([-0.5,0.5,0,1])
	axZ.set_xlabel('z')
	axZ.set_ylabel('force magnitude in z direction')

	zForces = forcesCart[forcesCart[:,1].argsort()]
	axZ.plot(pathCart[:,1],forcesCart[:,1],'b.')
	polyOrder = 2
	bestFitZ = np.polyfit(pathCart[:,1],forcesCart[:,1],polyOrder)
	print(bestFitZ)
	pZ = np.poly1d(bestFitZ)
	xpZ = np.linspace(-1,1,100)

	axZ.plot(xpZ,pZ(xpZ),'--')



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



	plt.draw()
	plt.pause(15)