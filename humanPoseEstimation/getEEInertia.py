import numpy as np

def getEEInertia(x,y,z):
	"""Get End Effector Inertia of ROBOT at point XYZ assumes ZERO VELOCITY"""

	table = "../multibodySim/ellAxis.txt" #from MatLab SimScape MultiBody

	try:
		#load in binary file for inertiaTable 
		#(will be deleted if MatLab generates replacement)
		inertiaTable = np.load('inertiaTable.npy')
	except:
		inertiaTable = np.genfromtxt(table,delimiter=',')
		np.save('inertiaTable.npy',inertiaTable)

	xmin = -0.5;
	xmax = 0.5;
	ymin = 0.25;
	ymax = 0.85;
	zmin = 0.2;
	zmax = 0.5;
	fidelity = 0.1;
	
	xPts = np.arange(xmin,xmax+0.0001,fidelity)
	yPts = np.arange(ymin,ymax+0.0001,fidelity)
	zPts = np.arange(zmin,zmax+0.0001,fidelity)
	xCt = 0
	yCt = 0
	zCt = 0

	for i in xPts:
		if i >=x:
			break
		xCt =+ 1
	for i in yPts:
		if i >=y:
			break
		zCt =+ 1
	for i in zPts:
		if i >=z:
			break
		zCt =+ 1

	address = (xPts.shape[0])*(yPts.shape[0])*xCt + (yPts.shape[0])*yCt + zCt
	Ix = inertiaTable[0,address]
	Iy = inertiaTable[1,address]
	Iz = inertiaTable[2,address]

	inertia = np.array([Ix, Iy, Iz])
	return(inertia)