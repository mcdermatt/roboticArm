from getEEInertia import getEEInertia as gei
from statePredictionWithInterpolation import statePrediction as statePredictionWithInterpolation
import numpy as np

def getForcesFromPathData(traj):
	"""read in path trajectory and calcuate external forces on end effector"""

	#traj = trajectory file, ideally in npy format
	#predictionTable = lookup table of next position states given current
	#inertiaTable = lookup table for robot inertia at every point XYZ in cartesian space

	trajTStep = 0.02 #timesteps between data points in trajectory file
	predTStep = 0.1 #length of time between in and out of prediction table






	return(forces)