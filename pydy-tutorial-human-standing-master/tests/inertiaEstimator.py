import cloudpickle
from numpy import zeros, array, linspace, deg2rad
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
	x0[0] = deg2rad(45)
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