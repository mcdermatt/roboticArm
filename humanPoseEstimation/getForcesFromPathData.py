from getEEInertia import getEEInertia as gei
from statePredictionWithInterpolation import statePrediction as spi 
import numpy as np
from forwardKinematicModel import forwardKinematicModel as fkm

def getForcesFromPathData(traj):
	"""read in path trajectory and calcuate external forces on end effector"""

	#traj = trajectory file, ideally in npy format
	#predictionTable = lookup table of next position states given current
	#inertiaTable = lookup table for robot inertia at every point XYZ in cartesian space

	trajTStep = 0.02 #timesteps between data points in trajectory file
	predTStep = 0.1 #length of time between in and out of prediction table
	stepLCM = int(predTStep / trajTStep)

	forces = np.zeros([3,int(np.floor(traj.shape[0]/stepLCM))])
	pos = np.zeros([3,int(np.floor(traj.shape[0]/stepLCM))])

	#To-Do- calculate velocity from trajectory
	theta0VelReal = 0
	theta1VelReal = 0
	theta2VelReal = 0
	theta0PosReal = 0
	theta1PosReal = 0
	theta2PosReal = 0

	step = 2
	count = 0
	while step < traj.shape[0]:
		
		#get predicted states from state prediction lookup table
		states = spi(theta0PosReal,theta1PosReal,theta2PosReal,theta0VelReal,theta1VelReal,theta2VelReal)
		theta0PosPredicted = states[0]
		theta1PosPredicted = states[1]
		theta2PosPredicted = states[2]

		#get actual measured current states from trajectory file
		theta0PosReal = traj[step,0]
		theta1PosReal = traj[step,1]
		theta2PosReal = traj[step,2]

		#transform both states from joint space to cartesian space
		predictedXYZ = fkm(theta0PosPredicted,theta1PosPredicted,theta2PosPredicted)
		actualXYZ = fkm(theta0PosReal,theta1PosReal,theta2PosReal)

		#get difference in actual and predicted states
		delta = abs(predictedXYZ - actualXYZ)

		#find endpoint inertia at predicted xyz
		inertiaMat = gei(actualXYZ[0],actualXYZ[1],actualXYZ[2])

		#F = Ia
		#normalize "a" because all movement takes place over 0.1s timestep
		forces[:,count] = inertiaMat*delta
		pos[:,count] = actualXYZ

		step += stepLCM
		count += 1

	ans = np.concatenate([pos,forces])

	return(ans)