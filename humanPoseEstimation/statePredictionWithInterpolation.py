import numpy as np


def statePrediction(j0pi,j1pi,j2pi,j0vi,j1vi,j2vi):
	
	#table = '../multibodySim/predictionTable666666.txt'
	#table = '../multibodySim/predictionTable888888.txt'
	# table = '../multibodySim/predictionTable444777.txt'
	# table = '../multibodySim/predictionTable266777.txt'
	table = '../multibodySim/predictionTable244444.txt'

	try:
		predictionTable = np.load('predictionTable244444.npy')
	except:
		predictionTable = np.genfromtxt(table, delimiter=',')
		np.save('predictionTable244444.npy', predictionTable)


	res = int(np.rint((predictionTable.shape[1])**(1/6)))
	j0PosPoints = np.linspace(-180,180,2);
	j1PosPoints = np.linspace(-105,45,4);
	j2PosPoints = np.linspace(-20,120,4);
	j0VelPoints = np.linspace(-120,120,4);
	j1VelPoints = np.linspace(-120,120,4);
	j2VelPoints = np.linspace(-120,120,4);

	j0pcount = 0
	j1pcount = 0
	j2pcount = 0
	j0vcount = 0
	j1vcount = 0
	j2vcount = 0

	#to-do figure out way to not have to loop through entire space (use previous states to get close??)
	for i in j0PosPoints:
		if i>=j0pi:
			if j0pcount != 0 :
				j0pcountLower = j0pcount - 1
			else:
				j0pcountLower = j0pcount
			break
		j0pcount += 1
	for i in j1PosPoints:
		if i>=j1pi:
			if j1pcount != 0 :
				j1pcountLower = j1pcount - 1
			else:
				j1pcountLower = j1pcount
			break
		j1pcount += 1
	for i in j2PosPoints:
		if i>=j2pi:
			if j2pcount != 0 :
				j2pcountLower = j2pcount - 1
			else:
				j2pcountLower = j2pcount
			break
		j2pcount += 1
	for i in j0VelPoints:
		if i>=j0vi:
			if j0vcount != 0 :
				j0vcountLower = j0vcount - 1
			else:
				j0vcountLower = j0vcount
			break
		j0vcount += 1
	for i in j1VelPoints:
		if i>=j1vi:
			if j1vcount != 0 :
				j1vcountLower = j1vcount - 1
			else:
				j1vcountLower = j1vcount
			break
		j1vcount += 1
	for i in j2VelPoints:
		if i>=j2vi:
			if j2vcount != 0 :
				j2vcountLower = j2vcount - 1
			else:
				j2vcountLower = j2vcount
			break
		j2vcount += 1

	# print(j0pcount)
	# print(j1pcount)
	# print(j2pcount)
	# print(j0vcount)
	# print(j1vcount)
	# print(j2vcount)

	#convert initial conditions to address in lookup table
	# addressUpper = (res**5)*(j0pcount-1) + (res**4)*(j1pcount-1) + (res**3)*(j2pcount-1) + (res**2)*(j0vcount-1) + (res)*(j1vcount-1) + (j2vcount-1)
	# addressLower = (res**5)*(j0pcountLower-1) + (res**4)*(j1pcountLower-1) + (res**3)*(j2pcountLower-1) + (res**2)*(j0vcountLower-1) + (res)*(j1vcountLower-1) + (j2vcountLower-1)

	addressUpper = (49*72)*(j0pcount-1) + (49*36)*(j1pcount-1) + (49*6)*(j2pcount-1) + (49)*(j0vcount-1) + (7)*(j1vcount-1) + (j2vcount-1)
	addressLower = (49*72)*(j0pcountLower-1) + (49*36)*(j1pcountLower-1) + (49*6)*(j2pcountLower-1) + (49)*(j0vcountLower-1) + (7)*(j1vcountLower-1) + (j2vcountLower-1)

	statesUpper = predictionTable[:,int(addressUpper)]
	statesLower = predictionTable[:,int(addressLower)]

	#convert back to deg
	statesUpper = statesUpper*180/np.pi
	statesLower = statesLower*180/np.pi

	#add delta states to old states to get new states
	statesUpper = statesUpper+[j0pi,j1pi,j2pi,j0vi,j1vi,j2vi]
	statesLower = statesLower+[j0pi,j1pi,j2pi,j0vi,j1vi,j2vi]

	#make frankenstein state prediction based on weighted averages of upper and lower states

	upperInput = [j0PosPoints[j0pcount],j1PosPoints[j1pcount],j2PosPoints[j2pcount],j0VelPoints[j0vcount],j1VelPoints[j1vcount],j2VelPoints[j2vcount]]
	lowerInput = [j0PosPoints[j0pcountLower],j1PosPoints[j1pcountLower],j2PosPoints[j2pcountLower],j0VelPoints[j0vcountLower],j1VelPoints[j1vcountLower],j2VelPoints[j2vcountLower]]


	print("upperInput = ",upperInput)
	print("lowerInput = ",lowerInput)


	print("statesUpper = ", statesUpper)
	print("statesLower = ", statesLower)

	states = 0 #debug
	return(states)
