import cloudpickle
from numpy import zeros, array, linspace, deg2rad, sin, cos, pi
import numpy as np
from scipy.integrate import odeint

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
                             2.72,  # j2_mass [kg]
                             0.001,  # NOT USED j2_inertia [kg*m^2]
                             9.81, #gravity [m/s^2]
                             0, #Fx
                             0, #Fy
                             0,],  #Fz 
                            ) 

	numerical_specified = zeros(3) #applied torques to each joint 
	args = {'constants': numerical_constants,
	        'specified': numerical_specified}
	frames_per_sec = 100
	final_time = 1
	t = linspace(0.0, final_time, final_time * frames_per_sec)

	x0 = zeros(6)
	x0[0] = deg2rad(90)
	x0[1] = deg2rad(45)
	x0[2] = deg2rad(45)

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

		cart[0] = (self.numerical_constants[4]*np.sin(theta1) + 0.2374*np.sin(theta1+theta2)) * np.cos(-1* theta0) #x l2=0.2374
		cart[1] = self.numerical_constants[4]*np.cos(theta1) + 0.2374*np.cos(theta1+theta2) #y
		cart[2] = (self.numerical_constants[4]*np.sin(theta1) + 0.2374*np.sin(theta1+theta2)) * np.sin(-1*theta0) #z

		return(cart)

	def cartesian2Joint(self, x, y, z):
		joint = zeros(3)

		l1 = numerical_constants[4] # upper arm
		l2 = 0.2374 # lower arm
		
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
			print("fin = ",fin)
			initial = self.joint2Cartesian(self.x0[0],self.x0[1],self.x0[2])
			print("initial = ", initial)
			diff[i] = abs(abs(fin[i]) - abs(initial[i]))
			print("diff = ", diff)
			#dy = v0t + (1/2)at^2
			#diff = (1/2)(a)(1^2)
			#diff = a/2
			#F = Ia
			#0.1 = I*(2*diff)
		inertias = 0.1/(2*diff)

		return(inertias)

	def getForces(self, traj):
		"""gets forces required to move manipulator to next state
			tarj = x y z cords of last two states"""
		#traj = np.zeros([3,2])

		#basically this function looks at the difference in cartesian space between 
		#   the last two states and tries to figure out the torques required to 
		#   make the next state continue the pattern

		#this is used to make the arm act as if the end effector is a heavy object 
		#   floating in zero gravity that has its own virtual inertia in
		#   the world frame. The torque output of this function is to be added 
		#   to existing gravity and friction cancellation torques.

		#convert input trajectory from world space to joint space
		#lastJointPos = self.cartesian2Joint(traj[:,0])
		#cjp = self.cartesian2Joint(traj[:,1]) #current joint pos
		cjp = np.zeros(3)
		#figure out what angles joints must be at to continue movement in world space
		# nextJointPos = 

		#get current velocities of each joint for later use in inertia calculation
		# dTheta = cjp - lastJointPos

		#manipulator jacobian allows conversion between joint torques and EE forces
		#   because we know the inertia of the end effector at any point xyz
		#   (because of getInertia() func.) we can just use that to figure out how much
		#   force is required to cancel inertia. 
		#   Internal robot dynamics are already accounted for in getInertia()

		#init joint lengths
		l0 = 0
		l1 = self.numerical_constants[4]
		l2 = 0.2374

		#need derivative of this
		# J = np.array([[l2*(-np.sin(cjp[0])*np.cos(cjp[1])*np.cos(cjp[2]) + np.sin(cjp[0])*np.sin(cjp[1])*np.sin(cjp[2])) - l1*np.sin(cjp[0])*np.cos(cjp[1])],
		# 			  [l2*(np.cos(cjp[0])*np.cos(cjp[1])*np.cos(cjp[2]) - np.cos(cjp[0])*np.sin(cjp[1])*np.sin(cjp[2])) + l1*np.cos(cjp[0])*np.cos(cjp[1])],
		# 			  [l2*(np.sin(cjp[1])*np.cos(cjp[2]) + np.cos(cjp[1])*np.sin(cjp[2])) + l1*np.sin(cjp[1]) + l0]])

		J = zeros([3,3])

		#dx/dq0
		J[0,0] = l2*(-cos(cjp[0])*cos(cjp[1] + pi/2)*cos(cjp[2]) + cos(cjp[0])*sin(cjp[1] + pi/2)*sin(cjp[2])) - l1*cos(cjp[0])*cos(cjp[1] + pi/2)
		#dx/dq1
		J[0,1] = l2*(sin(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2]) + sin(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2])) + l1*sin(cjp[0])*sin(cjp[1] + pi/2)
		#dx/dq2
		J[0,2] = l2*(sin(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2]) + sin(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2])) 
		#dy/dq0
		J[1,0] = l2*(-sin(cjp[0])*cos(cjp[1] + pi/2)*cos(cjp[2]) + sin(cjp[0])*sin(cjp[1] + pi/2)*sin(cjp[2])) - l1*sin(cjp[0])*cos(cjp[1] + pi/2)
		#dy/dq1
		J[1,1] = l2*(-cos(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2]) - cos(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2])) - l1*cos(cjp[0])*sin(cjp[1] + pi/2)
		#dy/dq2
		J[1,2] = l2*(-cos(cjp[0])*cos(cjp[1] + pi/2)*sin(cjp[2]) - cos(cjp[0])*sin(cjp[1] + pi/2)*cos(cjp[2]))
		#dz/dq0
		J[2,0] = 0 #this makes sense becasue there is no way j0 can affect z
		#dz/dq1
		J[2,1] = l2*(cos(cjp[1] + pi/2)*cos(cjp[2]) - sin(cjp[1] + pi/2)*sin(cjp[2])) + l1*cos(cjp[1])
		#dz/dq2
		J[2,2] = l2*(-sin(cjp[1] + pi/2)*sin(cjp[2]) + cos(cjp[1] + pi/2)*cos(cjp[2]))


		#this might not be necessary----------------------------------------
		#loop through different torques applied at each joint until a satisfactory
		#   solution is obtained 
		
		#starting with end effector, get torque values that will produce desired result
		#   using binary search tree for n iterations.
		#   It is importatnt to note that this value is NOT the final solution because
		#   it does not take into account acceleration from j0 and j1.
		#   Using obtained j2 torque as a stand in, torque of j0 and j1 is calculated and
		#   values are then refined m times by repeating this process

		#forces = zeros(3) #debug

		#speed test
		# count = 0
		# while count <= 5:
		# 	self.numerical_specified[0] = count*0.1
		# 	states = self.predict()
		# 	print(states)
		# 	count +=1

		return(J)

