import cloudpickle
from numpy import zeros, array, linspace, deg2rad
from scipy.integrate import odeint
import time

class statePredictor:

	#using EOM_func with gravity
	rhs = cloudpickle.load(open("full_EOM_func_NO_GRAVITY.txt", 'rb'))

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
	                             9.81],  # acceleration due to gravity [m/s^2]
	                            ) 

	numerical_specified = zeros(3)
	args = {'constants': numerical_constants,
	        'specified': numerical_specified}
	frames_per_sec = 100
	final_time = 1
	t = linspace(0.0, final_time, final_time * frames_per_sec)

	x0 = zeros(6)

	def predict(self, numerical_constants = numerical_constants, numerical_specified = numerical_specified, x0 = x0, rhs = rhs, t=t):

		y = odeint(rhs, x0, t, args=(numerical_specified, numerical_constants))
		#print(y)

		return(y)
