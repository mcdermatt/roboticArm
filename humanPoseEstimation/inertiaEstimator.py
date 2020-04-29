import cloudpickle
from numpy import zeros, array, linspace, deg2rad, sin, cos, pi
import numpy as np
from scipy.integrate import odeint
from statePredictor import statePredictor
from time import time

class inertiaEstimator:

	#robot_inertia_func assumes NO GRAVITY
	rhs = cloudpickle.load(open("robot_inertia_func.txt",'rb'))

	numerical_constants = array([0.05,  # j0_length [m]
                             0.01,  # j0_com_length [m]
                             4.20,  # j0_mass [kg]
                             0.001,  # NOT USED j0_inertia [kg*m^2]
                             0.164,  # j1_length [m]
                             0.08,  # j1_com_length [m]
                             1.81,  # j1_mass [kg]
                             0.001,  # NOT USED j1_inertia [kg*m^2]
                             0.158,  # j2_com_length [m]
                             2.259,  # j2_mass [kg]
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

	sp = statePredictor()

	def predictx(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#stateVec = zeros([3,6])
		#apply force in x direction [N]
		self.numerical_constants[12] = 0.1
		self.numerical_constants[13] = 0
		self.numerical_constants[14] = 0
		fxstates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		
		return(fxstates)

	def predicty(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#apply force in y direction [N]
		self.numerical_constants[12] = 0
		self.numerical_constants[13] = 0.1
		self.numerical_constants[14] = 0
		fystates = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))

		return(fystates)
#		stateVec[1,:] = newStates[-1]

	def predictz(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):
		#apply force in z direction [N]
		self.numerical_constants[12] = 0
		self.numerical_constants[13] = 0
		self.numerical_constants[14] = 0.1
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

		cart = zeros(3)
		l1 = 0.164
		l2 = 0.243
		cart[0] = (self.numerical_constants[4]*np.sin(theta1) + 0.243*np.sin(theta1+theta2)) * np.sin(-1* theta0) #x l2=0.243
		cart[1] = self.numerical_constants[4]*np.cos(theta1) + 0.243*np.cos(theta1+theta2) #y
		cart[2] = (self.numerical_constants[4]*np.sin(theta1) + 0.243*np.sin(theta1+theta2)) * np.cos(-1*theta0) #z

		# #x
		# cart[0] = l2*(-sin(theta0)*cos(theta1 - pi/2)*cos(theta2) + sin(theta0)*sin(theta1-pi/2)*sin(theta2)) - l1*sin(theta0)*sin(theta1 - pi/2)
		# #y
		# cart[2] = l2*(cos(theta0)*cos(theta1 - pi/2)*cos(theta2) - cos(theta0)*sin(theta1-pi/2)*sin(theta2)) + l1*cos(theta0)*cos(theta1 - pi/2)
		# #z
		# cart[1] = l2*(sin(theta1 - pi/2)*cos(theta2) + cos(theta1-pi/2)*sin(theta2)) + l1*sin(theta1 - pi/2)

		return(cart)

	def cartesian2Joint(self, x, y, z):
		#incorrect rn
		joint = zeros(3)

		l1 = self.numerical_constants[4] # upper arm
		l2 = 0.243 # lower arm
		
		r = np.sqrt((x*x)+(y*y)+(z*z))
		phi = np.arctan2((np.sqrt((x  * x) + (y * y))), z )
		theta = np.arctan2(y, x)

		#elbow
		joint[2] = np.arccos(((l1*l1)+(l2*l2)-(r*r))/(-2*l1*l2))
		#shoulder side to side
		joint[0] = theta
		#shoulder up down
		joint[1] = np.pi + phi + np.arccos(((l1*l1)-(l2*l2)+(r*r))/(-2*l1*r))

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

