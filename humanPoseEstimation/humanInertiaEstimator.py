import cloudpickle
from numpy import zeros, array, linspace, deg2rad, sin, cos, pi
import numpy as np
from scipy.integrate import odeint
# from statePredictor import statePredictor
from time import time

class inertiaEstimator:

	#human_inertia_func assumes NO GRAVITY
	rhs = cloudpickle.load(open("human_inertia_func.txt",'rb'))

	numerical_constants = array([0.0,  # j0_length [m]
                             0.0,  # j0_com_length [m]
                             4.0,  # j0_mass [kg]
                             0.001,  # NOT USED j0_inertia [kg*m^2]
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

	numerical_specified = zeros(3) #applied torques to each joint 
	args = {'constants': numerical_constants,
	        'specified': numerical_specified}
	# frames_per_sec = 10
	# final_time = 0.1
	# t = linspace(0.0, final_time, final_time * frames_per_sec)
	t = linspace(0.0,0.1,10)

	x0 = zeros(6)
	x0[0] = deg2rad(90)
	x0[1] = deg2rad(45)
	x0[2] = deg2rad(45)

	# sp = statePredictor()

	def predictx(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#stateVec = zeros([3,6])
		#apply force in x direction [N]
		self.numerical_constants[12] = 0.001
		self.numerical_constants[13] = 0
		self.numerical_constants[14] = 0
		fxstates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		
		return(fxstates)

	def predicty(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#apply force in y direction [N]
		self.numerical_constants[12] = 0
		self.numerical_constants[13] = 0.001
		self.numerical_constants[14] = 0
		fystates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))

		print('fystates = ' , fystates)

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
		self.numerical_constants[12] = 0
		self.numerical_constants[13] = 0
		self.numerical_constants[14] = 0.001
		fzstates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		#stateVec[2,:] = newStates[-1]
		#print(y)
		#print(numerical_constants[12])
		return(fzstates)

	def predict(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		stateVec = zeros([3,6])
		stateVec[0,:] = self.predictx()[-1]
		stateVec[1,:] = self.predicty()[-1]
		stateVec[2,:] = self.predictz()[-1]

		return(stateVec)

	def joint2Cartesian(self, theta0, theta1, theta2):
		#Depricated
		cart = zeros(3)

		l1 = self.numerical_constants[4] # upper arm
		l2 = self.numerical_constants[4] # lower arm

		# r = l1 + l2*np.cos(theta2)
		theta = np.pi - theta0 - np.arctan((l2*np.sin(np.pi - theta2))/(l1 - l2*np.cos(np.pi - theta2)))
		r = (l1 + l2*np.cos(theta2))/(np.cos(np.arctan((l2*np.sin(np.pi - theta2))/(l1 - l2*np.cos(np.pi - theta2)))))
		phi = np.pi - np.arccos((l1*np.cos(np.pi- theta1) - l2*np.cos(np.pi - theta2)*np.cos(np.pi-theta1))/(r))

		print('r = ', r, ' phi = ', phi, ' theta = ', theta)

		#x
		cart[0] = r*np.cos(theta)*np.sin(phi)
		#y
		cart[1] = r*np.cos(phi)
		#z
		cart[2] = r*np.sin(theta)*np.sin(phi)

		# #x hand
		# cart[0] = np.sin(theta1)*(l1*np.cos(np.pi - theta0) + l2*np.cos(np.pi - theta0 + theta2))
		# #z hand
		# cart[2] = np.sin(theta1)*(l1*np.sin(np.pi - theta0) + l2*np.sin(np.pi - theta0 + theta2))
		# #y hand - correct?
		# cart[1] = np.cos(theta1)*(l1 + l2*np.cos(np.pi - theta2))

		# cart[0] = (self.numerical_constants[4]*np.sin(theta1) + 0.243*np.sin(theta1+theta2)) * np.sin(-1* theta0) #x l2=0.243
		# cart[1] = self.numerical_constants[4]*np.cos(theta1) + 0.243*np.cos(theta1+theta2) #y
		# cart[2] = (self.numerical_constants[4]*np.sin(theta1) + 0.243*np.sin(theta1+theta2)) * np.cos(-1*theta0) #z

		# #x
		# cart[0] = l2*(-sin(theta0)*cos(theta1 - pi/2)*cos(theta2) + sin(theta0)*sin(theta1-pi/2)*sin(theta2)) - l1*sin(theta0)*sin(theta1 - pi/2)
		# #y
		# cart[2] = l2*(cos(theta0)*cos(theta1 - pi/2)*cos(theta2) - cos(theta0)*sin(theta1-pi/2)*sin(theta2)) + l1*cos(theta0)*cos(theta1 - pi/2)
		# #z
		# cart[1] = l2*(sin(theta1 - pi/2)*cos(theta2) + cos(theta1-pi/2)*sin(theta2)) + l1*sin(theta1 - pi/2)

		return(cart)

	def cartesian2Joint(self, x, y, z):

		joint = zeros(3)

		l1 = self.numerical_constants[4] # upper arm
		l2 = self.numerical_constants[4] # lower arm
		
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

		joint[0] = np.pi - np.arcsin((l2*np.sin(np.pi - joint[2]))/(np.sqrt(x**2 + z**2))) -theta

		#prev attempt (5/26/20)
		# joint[0] = np.pi - np.arccos((np.sin(np.pi -joint[1])*(l1 - np.sqrt(l2**2 + (l2*np.sin(np.pi - joint[2]))**2)))/(np.sqrt(x**2 + z**2))) + theta

		# joint[0] = theta + np.arcsin((l2*(np.sin(joint[2])**2))/np.sqrt(x ** 2 + z ** 2))

		# joint[1] = np.pi - np.arcsin((((l2*(np.sin(joint[2])**2))/np.tan(joint[0]-theta))-(l2*(np.sin(joint[2])**2)))/l1)

		joint = np.rad2deg(joint)

		return(joint)

	def getInertia(self):
		states = self.predict()
		diff = zeros(3)

		print("x0 = ", self.x0)

		for i in range(0,3):
		#convert to cartesian space
			fin = self.joint2Cartesian(states[i,0],states[i,1],states[i,2])
		
			initial = self.joint2Cartesian(self.x0[0],self.x0[1],self.x0[2])
			
			diff[i] = abs(abs(fin[i]) - abs(initial[i]))
			
			#dy = v0t + (1/2)at^2
			#diff = (1/2)(a)(0.1^2) #Remember to update this if you change how long to run predictx,y,z for
			#diff = 0.005*a
			#F = Ia
			#0.1 = I*(200*diff)
		inertias = 0.1/(200*diff)

		return(inertias)

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

