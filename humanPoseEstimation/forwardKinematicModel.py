import numpy as np

def forwardKinematicModel(theta0,theta1,theta2):
	""" Inputs in degrees assume arm has reach of 1 unit """

	l0 = 0
	l1 = 0.40625
	l2 = 0.59375

	alpha = l1*np.sin(theta1*(np.pi/180))+l2*np.sin(theta2*(np.pi/180))

	x = alpha*np.sin(theta0*np.pi/180)
	y = l1*np.cos(theta1*np.pi/180) + l2*np.cos(theta2*np.pi/180)
	z = alpha*np.cos(theta0*np.pi/180)

	pos = np.array([x,y,z])
	return(pos)