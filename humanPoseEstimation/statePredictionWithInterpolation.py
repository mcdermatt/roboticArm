import numpy as np


def statePrediction(j0pi,j1pi,j2pi,j0vi,j1vi,j2vi):
	"""give predicted joint angles for j0 j1 and j2 assumes no gravity"""
	
	# table = '../multibodySim/predictionTable666666.txt' #works
	table = '../multibodySim/predictionTable888888.txt'
	# table = '../multibodySim/predictionTable444777.txt'
	#table = '../multibodySim/predictionTable266777.txt' # seems to work ok? might need higher resolution in j0 position
	#table = '../multibodySim/predictionTable377777.txt' #bad data DO NOT USE

	try:
		predictionTable = np.load('predictionTable888888.npy')
	except:
		predictionTable = np.genfromtxt(table, delimiter=',')
		np.save('predictionTable888888.npy', predictionTable)


	res = int(np.rint((predictionTable.shape[1])**(1/6)))
	j0PosPoints = np.linspace(-180,180,res);
	j1PosPoints = np.linspace(-105,45,res);
	j2PosPoints = np.linspace(-20,120,res);
	j0VelPoints = np.linspace(-120,120,res);
	j1VelPoints = np.linspace(-120,120,res);
	j2VelPoints = np.linspace(-120,120,res);

	# j0VelPoints = np.array([-100, -20, -10, 0, 10, 20, 100])
	# j1VelPoints = np.array([-100, -20, -10, 0, 10, 20, 100])
	# j2VelPoints = np.array([-100, -20, -10, 0, 10, 20, 100])

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

	print(j0pcount)
	print(j1pcount)
	print(j2pcount)
	print(j0vcount)
	print(j1vcount)
	print(j2vcount)

	#convert initial conditions to address in lookup table
	addressUpper = (res**5)*(j0pcount) + (res**4)*(j1pcount) + (res**3)*(j2pcount) + (res**2)*(j0vcount) + (res)*(j1vcount) + (j2vcount)
	addressLower = (res**5)*(j0pcountLower) + (res**4)*(j1pcountLower) + (res**3)*(j2pcountLower) + (res**2)*(j0vcountLower) + (res)*(j1vcountLower) + (j2vcountLower)

	#uncomment for non regularly shaped lookup table
	# addressUpper = (49*72)*(j0pcount-1) + (49*36)*(j1pcount-1) + (49*6)*(j2pcount-1) + (49)*(j0vcount-1) + (7)*(j1vcount-1) + (j2vcount-1)
	# addressLower = (49*72)*(j0pcountLower-1) + (49*36)*(j1pcountLower-1) + (49*6)*(j2pcountLower-1) + (49)*(j0vcountLower-1) + (7)*(j1vcountLower-1) + (j2vcountLower-1)

	statesUpper = predictionTable[:,int(addressUpper)]
	statesLower = predictionTable[:,int(addressLower)]

	#convert back to deg
	statesUpper = statesUpper*180/np.pi
	statesLower = statesLower*180/np.pi

	#add delta states to old states to get new rough estimate states
	statesUpper = statesUpper+[j0pi,j1pi,j2pi,j0vi,j1vi,j2vi]
	statesLower = statesLower+[j0pi,j1pi,j2pi,j0vi,j1vi,j2vi]

	#make frankenstein state prediction based on weighted averages of upper and lower states

	upperInput = [j0PosPoints[j0pcount],j1PosPoints[j1pcount],j2PosPoints[j2pcount],j0VelPoints[j0vcount],j1VelPoints[j1vcount],j2VelPoints[j2vcount]]
	lowerInput = [j0PosPoints[j0pcountLower],j1PosPoints[j1pcountLower],j2PosPoints[j2pcountLower],j0VelPoints[j0vcountLower],j1VelPoints[j1vcountLower],j2VelPoints[j2vcountLower]]

	print("address Upper = ", addressUpper)
	print("upperInput = ",upperInput)
	print("address Lower = ", addressLower)
	print("lowerInput = ",lowerInput)


	print("statesUpper = ", statesUpper)
	print("statesLower = ", statesLower)

	#create weighted average for each state
	udp0 = abs(statesUpper[0] - j0pi) #distance from nearest upper input to actual j0pi
	ldp0 = abs(j0pi - statesLower[0]) #distance from nearest lower input to actual j0pi
	Dp0 =  ldp0 + udp0 #distance between upper and lower input values stored in table
	j0pf = statesLower[0]*(udp0/Dp0)+statesUpper[0]*(ldp0/Dp0)

	udp1 = abs(statesUpper[1] - j1pi)
	ldp1 = abs(j1pi - statesLower[1]) 
	Dp1 =  ldp1 + udp1
	j1pf = statesLower[1]*(udp1/Dp1)+statesUpper[1]*(ldp1/Dp1)

	udp2 = abs(statesUpper[2] - j2pi)
	ldp2 = abs(j2pi - statesLower[2]) 
	Dp2 =  ldp2 + udp2
	j2pf = statesLower[2]*(udp2/Dp2)+statesUpper[2]*(ldp2/Dp2)

	udv0 = abs(statesUpper[3] - j0vi)
	ldv0 = abs(j0vi - statesLower[3]) 
	Dv0 =  ldv0 + udv0 
	j0vf = statesLower[3]*(udv0/Dv0)+statesUpper[3]*(ldv0/Dv0)

	udv1 = abs(statesUpper[4] - j1vi)
	ldv1 = abs(j1vi - statesLower[4]) 
	Dv1 =  ldv1 + udv1
	j1vf = statesLower[4]*(udv1/Dv1)+statesUpper[4]*(ldv1/Dv1)

	udv2 = abs(statesUpper[5] - j2vi)
	ldv2 = abs(j2vi - statesLower[5]) 
	Dv2 =  ldv2 + udv2
	j2vf = statesLower[5]*(udv2/Dv2)+statesUpper[5]*(ldv2/Dv2)

	# states = np.array([j0pf,j1pf,j2pf,j0vf,j1vf,j2vf])
	states = np.array([j0pf,j1pf,j2pf]) #only really need predicted position values


	return(states)
