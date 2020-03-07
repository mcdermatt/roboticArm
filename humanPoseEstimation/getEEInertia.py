import numpy as numpy

def getEEInertia(x,y,z):
	"""Get End Effector Inertia at point XYZ assumes ZERO VELOCITY"""

	table = "../multibodySim/ellAxis.txt" #from MatLab SimScape MultiBody

	try:
		inertiaTable = np.load('inertiaTable.npy')
	except:
		inertiaTable = np.genfromtxt(table, inertiaTable)
		np.save('inertiaTable.npy',inertiaTable)


	Ix = 0
	Iy = 0
	Iz = 0

	inertia = np.array([Ix, Iy, Iz])
	return(inertia)