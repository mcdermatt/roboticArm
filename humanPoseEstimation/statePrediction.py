import numpy as np


def statePrediction(j0pi,j1pi,j2pi,j0vi,j1vi,j2vi):
	
	#table = '../multibodySim/predictionTable666666.txt'
	table = '../multibodySim/predictionTable888888.txt'

	predictionTable = np.genfromtxt(table, delimiter=',')

	res = int(np.rint((predictionTable.shape[1])**(1/6)))
	j0PosPoints = np.linspace(-180,180,res);
	j1PosPoints = np.linspace(-105,45,res);
	j2PosPoints = np.linspace(-20,120,res);
	j0VelPoints = np.linspace(-120,120,res);
	j1VelPoints = np.linspace(-120,120,res);
	j2VelPoints = np.linspace(-120,120,res);

	j0pcount = 0
	j1pcount = 0
	j2pcount = 0
	j0vcount = 0
	j1vcount = 0
	j2vcount = 0

	#to-do figure out way to not have to loop through entire space (use previous states to get close??)
	for i in j0PosPoints:
		if i>=j0pi:
			j0piU = i #next closest value in table above j0pi
			break
		j0piL = i #next closest value below j0pi
		j0pcount += 1
	for i in j1PosPoints:
		if i>=j1pi:
			j1piU = i
			break
		j1piL = i
		j1pcount += 1
	for i in j2PosPoints:
		if i>=j2pi:
			j2piU = i
			break
		j2piL = i
		j2pcount += 1
	for i in j0VelPoints:
		if i>=j0vi:
			j0viU = i
			break
		j0viL = i
		j0vcount += 1
	for i in j1VelPoints:
		if i>=j1vi:
			j1viU = i
			break
		j1viL = i
		j1vcount += 1
	for i in j2VelPoints:
		if i>=j2vi:
			j2viU = i
			break
		j2viL = i
		j2vcount += 1

	print(j0pcount)
	print(j1pcount)
	print(j2pcount)
	print(j0vcount)
	print(j1vcount)
	print(j2vcount)

	#convert initial conditions to address in lookup table
	addressUpper = (res**5)*(j0pcount-1) + (res**4)*(j1pcount-1) + (res**3)*(j2pcount-1) + (res**2)*(j0vcount-1) + (res)*(j1vcount-1) + (j2vcount-1) 

	statesUpper = predictionTable[:,int(addressUpper)]
	#statesLower = predictionTable[:,addressLower]

	#convert back to deg
	statesUpper = statesUpper*180/np.pi
#	statesLower = statesLower*180/np.pi

	#add delta states to old states to get new states
	statesUpper = statesUpper+[j0pi,j1pi,j2pi,j0vi,j1vi,j2vi]

	#to-do make weighted sum to interpolate

	return(statesUpper)
