import numpy as np 


try:
	armPath = np.load('armPath.npy')
except:
	armPath = np.genfromtxt('../handGuidedPath/armPath.txt',delimiter=' ')
	np.save('armPath.npy',armPath)