import cloudpickle
from numpy import zeros, array, linspace, deg2rad, sin, cos, pi
import numpy as np
from scipy.integrate import odeint
# from statePredictor import statePredictor
from time import time

class inertiaEstimator4DOF:

	#human_inertia_func assumes NO GRAVITY
	rhs = cloudpickle.load(open("human_inertia_func_4DOF.txt",'rb'))

	numerical_constants = array([0.0,  # j0_length [m]
                             0.0,  # j0_com_length [m]
                             4.0,  # j0_mass [kg]
                             0.001,  # NOT USED j0_inertia [kg*m^2]
                             0, #js stuff
                             0,
                             1,
                             0,
                             0.33,  # j1_length [m]
                             0.132,  # j1_com_length [m]
                             2.95,  # j1_mass (upper arm) [kg]
                             0.001,  # NOT USED j1_inertia [kg*m^2]
                             0.132,  # j2_com_length [m]
                             2.27,  # j2_mass (lower arm) [kg]
                             0.001,  # NOT USED j2_inertia [kg*m^2]
                             9.81, #gravity [m/s^2]
                             0, #Fx
                             0, #Fy
                             0,],  #Fz 
                            ) 

	numerical_specified = zeros(4) #applied torques to each joint 
	args = {'constants': numerical_constants,
	        'specified': numerical_specified}
	# frames_per_sec = 10
	# final_time = 0.1
	# t = linspace(0.0, final_time, final_time * frames_per_sec)
	t = linspace(0.0,0.1,10)

	x0 = zeros(8)
	x0[0] = deg2rad(90)
	x0[1] = deg2rad(0)
	x0[2] = deg2rad(45)
	x0[3] = deg2rad(45)

	# sp = statePredictor()

	def predictx(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#stateVec = zeros([3,6])
		#apply force in x direction [N]
		self.numerical_constants[16] = 100
		self.numerical_constants[17] = 0
		self.numerical_constants[18] = 0
		fxstates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		
		return(fxstates)

	def predicty(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#apply force in y direction [N]
		self.numerical_constants[16] = 0
		self.numerical_constants[17] = 100
		self.numerical_constants[18] = 0
		fystates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))

		# print('fystates = ' , fystates)

		#try movement again in opposite direction to avoid kinemtic constraints- 
		# self.numerical_constants[12] = 0
		# self.numerical_constants[13] = -0.001
		# self.numerical_constants[14] = 0
		# altfystates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		# #take whichever values is larger
		# if altfystates[1][-1] > fystates[1][-1]:
		# 	fystates = altfystates

		return(fystates)
#		stateVec[1,:] = newStates[-1]

	def predictz(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#apply force in z direction [N]
		self.numerical_constants[16] = 0
		self.numerical_constants[17] = 0
		self.numerical_constants[18] = 100
		fzstates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		#stateVec[2,:] = newStates[-1]
		#print(y)
		#print(numerical_constants[12])
		return(fzstates)

	def predictxz(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#apply force in z direction [N]
		self.numerical_constants[16] = 1
		self.numerical_constants[17] = 0
		self.numerical_constants[18] = 1
		fxzstates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		#stateVec[2,:] = newStates[-1]
		#print(y)
		#print(numerical_constants[12])
		initial = self.joint2CartesianV2(self.x0[0],self.x0[1],self.x0[2])[:3,3]
		final = self.joint2CartesianV2(fxzstates[-1][0],fxzstates[-1][1],fxzstates[-1][2])[:3,3]

		delta = final - initial
		print('delta = ', delta)
		theta = np.sign(delta[1])*np.sign(delta[0])*np.arctan(delta[1]/delta[0])
		print('theta = ',theta)

		return(theta)

	def predictXZGauss(self, numTrials = 20, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):

		i = 0
		thetai = 0
		x = np.zeros(numTrials)
		y = np.zeros(numTrials)
		z = np.zeros(numTrials)

		while i < numTrials:

			self.numerical_constants[16] = 2*np.random.randn()
			self.numerical_constants[17] = 2*np.random.randn()
			self.numerical_constants[18] = 2*np.random.randn()
			fxzstates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
			#stateVec[2,:] = newStates[-1]
			#print(y)
			#print(numerical_constants[12])
			initial = self.joint2CartesianV2(self.x0[0],self.x0[1],self.x0[2],self.x0[3])[:3,3]
			# print('initial = ',initial)
			final = self.joint2CartesianV2(fxzstates[-1][0], fxzstates[-1][1], fxzstates[-1][2],fxzstates[-1][3])[:3,3]
			# print('final = ',final)
			delta = final - initial
			# print('delta = ', delta)
			
			x[i] = delta[1]
			y[i] = delta[2]
			z[i] = delta[0]

			i += 1

		# try:
		# 	m,b = np.polyfit(x,z,1) #will not work - need intercept to be at origen
		# except:
		# 	m =  0

		# print('theta = ',theta)
		# print('x = ',x)
		# print('z = ',z)


		# print('x = ',x,' z = ', z)
		return(x,y,z)


	def predict(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		stateVec = zeros([3,6])
		stateVec[0,:] = self.predictx()[-1]
		stateVec[1,:] = self.predicty()[-1]
		stateVec[2,:] = self.predictz()[-1]

		return(stateVec)

	def joint2CartesianV2(self,theta0, thetaS, theta1,theta2):

		#joint 0 with respect to base
		d0 = 0 #j0 and j1 at same point
		alpha0 = np.pi/2
		a0 = 0
		theta0 = theta0 + np.pi/2
		A0 = np.array([[np.cos(theta0), -np.sin(theta0)*np.cos(alpha0), np.sin(theta0)*np.sin(alpha0), a0*np.cos(theta0)],
						[np.sin(theta0), np.cos(theta0)*np.cos(alpha0), -np.cos(theta0)*np.sin(alpha0), a0*np.sin(theta0)],
						[0, 	np.sin(alpha0),		np.cos(alpha0), 		d0],
						[0,		0,		0,		1]])

		#joint S with respect to j0
		dS = 0
		alphaS = 0 #messing with this rn
		aS = 0
		AS = np.array([[np.cos(thetaS), -np.sin(thetaS)*np.cos(alphaS), np.sin(thetaS)*np.sin(alphaS), a0*np.cos(thetaS)],
						[np.sin(thetaS), np.cos(theta0)*np.cos(alphaS), -np.cos(thetaS)*np.sin(alphaS), a0*np.sin(thetaS)],
						[0, 	np.sin(alphaS),		np.cos(alphaS), 		dS],
						[0,		0,		0,		1]])

		# #joint 1 with respect to joint S
		d1 = 0 #j0 and j1 at same point
		alpha1 = np.pi/2
		a1 = self.numerical_constants[8]
		theta1 = theta1 + np.pi/2 #need to set zero position for DH table- sympy has zero at vertical
		A1 = np.array([[np.cos(theta1), -np.sin(theta1)*np.cos(alpha1), np.sin(theta1)*np.sin(alpha1), a1*np.cos(theta1)],
						[np.sin(theta1), np.cos(theta1)*np.cos(alpha1), -np.cos(theta1)*np.sin(alpha1), a1*np.sin(theta1)],
						[0, 	np.sin(alpha1),		np.cos(alpha1), 		d1],
						[0,		0,		0,		1]])


		# #joint 2 with respect to joint 1
		d2 = 0 #j0 and j1 at same point
		alpha2 = np.pi/2
		a2 = self.numerical_constants[8]
		A2 = np.array([[np.cos(theta2), -np.sin(theta2)*np.cos(alpha2), np.sin(theta2)*np.sin(alpha2), a2*np.cos(theta2)],
						[np.sin(theta2), np.cos(theta2)*np.cos(alpha2), -np.cos(theta2)*np.sin(alpha2), a2*np.sin(theta2)],
						[0, 	np.sin(alpha2),		np.cos(alpha2), 		d2],
						[0,		0,		0,		1]])

		#transform of 2 with respect to 0
		T = A0.dot(AS).dot(A1).dot(A2)

		#Z
		#X
		#Y
		return(T)


	def cartesian2Joint(self, x, y, z):

		joint = zeros(3)

		l1 = self.numerical_constants[8] # upper arm
		l2 = self.numerical_constants[8] # lower arm
		
		r = np.sqrt((x*x)+(y*y)+(z*z))
		# print("r = ", r)
		phi = np.arctan2((np.sqrt((x  * x) + (z * z))), y )
		# print("phi = ", phi)
		theta = np.arctan2(z, x)
		# print("theta = ", theta)

		joint[2] = np.pi - 2*np.arcsin(r/(2*l2))

		#always putting shoudler negative
		joint[1] = np.pi - np.arcsin((np.sqrt(x**2 + z**2 - (l2*np.sin(np.pi - joint[2]))**2))/(l1*(1-np.cos(np.pi - joint[2]))))
		if y > 0:
			joint[1] = np.pi - joint[1]

		if (np.isnan(joint[1]) == 1) and (y == 0) :
			joint[1] = np.pi/2 #temp fix

		joint[0] = np.pi - np.arcsin((l2*np.sin(np.pi - joint[2]))/(np.sqrt(x**2 + z**2))) -theta


		return(joint)

	def getInertia(self):
		states = self.predict()
		print('predicted states = ',states)
		diff = zeros(3)

		print("x0 = ", self.x0)

		for i in range(0,3):
		#convert to cartesian space
			fin = self.joint2CartesianV2(states[i,0],states[i,1],states[i,2])
			print('fin = ', fin)
			initial = self.joint2CartesianV2(self.x0[0],self.x0[1],self.x0[2])
			print('initial = ',initial)

			#TODO - issue somewhere in here
			diff[i] = abs(fin[i,3] - initial[i,3])
			
			#dy = v0t + (1/2)at^2
			#diff = (1/2)(a)(0.1^2) #Remember to update this if you change how long to run predictx,y,z for
			#diff = 0.005*a
			#F = Ia
			#0.1 = I*(200*diff)
		print('diff = ',diff )
		inertias = 0.1/(200*diff)

		# return(inertias)
		return(diff)

	def getForces(self, cjp, ljp, dt):
		"""gets forces required to move manipulator to next state
			cjp = current joint positions
			ljp = last joint positions
			dt = time elapsed between measurements"""
		#traj = np.zeros([3,2])

		#basically this function looks at the difference in cartesian space between 
		#   the last two states and tries to figure out the torques required to 
		#   make the next state continue the pattern

		#this is used to make the arm act as if the end effector is a heavy object 
		#   floating in zero gravity that has its own virtual inertia in
		#   the world frame. The torque output of this function is to be added 
		#   to existing gravity and friction cancellation torques.
		
		#get start time so we can know how long this function takes
		#tstart = time() 

		#update initial values x0 for use in getInertia()
		self.x0[:3] = cjp
		self.x0[3:] = (cjp-ljp)/dt

		#get inertia of arm at cjp 
		inertias = self.getInertia() 
		print("inertias = ", inertias)

		#get change in xyz position
		#TODO -FIX JOINT TO CARTESIAN INCONSISTANCY -Y and Z axis flipped?????
		newCart = self.joint2Cartesian(cjp[0],cjp[1],cjp[2])
		#print("newCart = ",newCart)
		oldCart = self.joint2Cartesian(ljp[0],ljp[1],ljp[2])
		#print("oldCart = ", oldCart )
		delCart = newCart - oldCart
		#print("delCart = ", delCart)

		#we want to continue moving the same distance next timestep
		goalCart = newCart + delCart 

		#if we stop powering the joints right now this will happen
		#TODO get timesteps to match between this function and statePredictor-----------------------------------------------
		predictionJoint = self.sp.predict(x0 = self.x0, dt = dt)[-1]
		predictionCart = self.joint2Cartesian(predictionJoint[0],predictionJoint[1],predictionJoint[2])

		#so we need to give the joints just enough power to move from the prediction to the goal pos such that EE reaches goal at timestep
		requiredForcesCart = zeros(3)
		#F = Ma
		#x = vi*t + (1/2)at^2
		#F = 2*x*m/(t^2)
		requiredForcesCart = 2*delCart*inertias/(dt**2)

		#init joint lengths
		l0 = 0
		l1 = self.numerical_constants[4]
		l2 = 0.243


		#manipulator jacobian allows conversion between joint torques and EE forces
		#   because we know the inertia of the end effector at any point xyz
		#   (because of getInertia() func.) we can just use that to figure out how much
		#   force is required to cancel inertia. 
		#   Internal robot dynamics are already accounted for in getInertia()
		J = zeros([3,3])
		#dx/dq0
		J[0,0] = l2*(-cos(cjp[0])*cos(cjp[1] + pi/2)*cos(cjp[2]) + cos(cjp[0])*sin(cjp[1] + pi/2)*sin(cjp[2])) - l1*cos(cjp[0])*cos(cjp[1] + pi/2)
		#dx/dq1
		J[0,1] = l2*(sin(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2]) + sin(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2])) + l1*sin(cjp[0])*sin(cjp[1] + pi/2)
		#dx/dq2
		J[0,2] = l2*(sin(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2]) + sin(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2])) 
		#dy/dq0
		J[2,0] = l2*(-sin(cjp[0])*cos(cjp[1] + pi/2)*cos(cjp[2]) + sin(cjp[0])*sin(cjp[1] + pi/2)*sin(cjp[2])) - l1*sin(cjp[0])*cos(cjp[1] + pi/2)
		#dy/dq1
		J[2,1] = l2*(-cos(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2]) - cos(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2])) - l1*cos(cjp[0])*sin(cjp[1] + pi/2)
		#dy/dq2
		J[2,2] = l2*(-cos(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2]) - cos(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2]))
		#dz/dq0
		J[1,0] = 0 #this makes sense becasue there is no way j0 can affect z
		#dz/dq1
		J[1,1] = l2*(cos(cjp[1] + pi/2)*cos(cjp[2]) - sin(cjp[1] + pi/2)*sin(cjp[2])) + l1*cos(cjp[1])
		#dz/dq2
		J[1,2] = l2*(-sin(cjp[1] + pi/2)*sin(cjp[2]) + cos(cjp[1] + pi/2)*cos(cjp[2]))

		#use geometric jacobian to convert cartesian forces to joint torques
		#I think this might not be correct
		# jointTorques = np.linalg.inv(J).dot(requiredForcesCart)

		#coutesy of http://bionics.seas.ucla.edu/education/MAE_263D/MAE_263D_C08_V01.pdf p44
		jointTorques = J.T.dot(requiredForcesCart)

		#tend = time()
		#elapsed = tend - tstart
		#print("time elapsed = ", elapsed)
		print("inertia cancellation torques: ",jointTorques)

		return(jointTorques)

