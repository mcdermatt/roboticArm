from __future__ import print_function, division
from IPython.display import Image
from sympy.printing.pretty.pretty import pretty_print
import numpy as np
from scipy.integrate import odeint
from pydy.codegen.ode_function_generators import generate_ode_function
import matplotlib.pyplot as plt
from pydy.viz.shapes import Cylinder, Sphere
import pydy.viz
from pydy.viz.visualization_frame import VisualizationFrame
from pydy.viz.scene import Scene
from utils import controllable
from scipy.linalg import solve_continuous_are
import control
import sympy
from sympy import symbols, simplify, trigsimp
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame, Point, inertia, RigidBody, KanesMethod
from sympy.physics.vector import init_vprinting, vlatex
#from matplotlib.pyplot import plot, legend, xlabel, ylabel, rcParams

init_vprinting(use_latex='mathjax', pretty_print=True)

#Kinematics ---------------------------------------------------------------------------
#Init reference frames, assume foot is fixed to floor
inertial_frame = ReferenceFrame('I')
lower_leg_frame = ReferenceFrame('L')
upper_leg_frame = ReferenceFrame('U')
torso_frame = ReferenceFrame('T')

#declare dynamic symbols for three joints- dynamic symbols allow derivative notation
theta1, theta2, theta3 = dynamicsymbols('theta1, theta2, theta3')

#Reference frames - rotation only
#orient lower leg frame with respect to inertial (base) frame
lower_leg_frame.orient(inertial_frame, 'Axis', (theta1, inertial_frame.z))
#command below displays direction cosine matrix (DCM)
#pretty_print(lower_leg_frame.dcm(inertial_frame))

#orient upper leg frame with respect to lower leg frame
upper_leg_frame.orient(lower_leg_frame, 'Axis',(theta2,lower_leg_frame.z))
#pretty_print(simplify(upper_leg_frame.dcm(inertial_frame)))

#orient torso
torso_frame.orient(upper_leg_frame, 'Axis', (theta3, upper_leg_frame.z))
#pretty_print(simplify(torso_frame.dcm(inertial_frame)))

#Init joints- points = translation from existing points
ankle = Point('A')
lower_leg_length = symbols('l_L')
knee = Point('K')
knee.set_pos(ankle, lower_leg_length * lower_leg_frame.y)
#pretty_print(knee.pos_from(ankle))
#pretty_print(knee.pos_from(ankle).express(inertial_frame).simplify())
upper_leg_length = symbols('l_U')
hip = Point('H')
hip.set_pos(knee, upper_leg_length * upper_leg_frame.y)

#COM Locations
lower_leg_com_length, upper_leg_com_length, torso_com_length = symbols('d_L, d_U, d_T')
lower_leg_mass_center = Point('L_o')
lower_leg_mass_center.set_pos(ankle, lower_leg_com_length * lower_leg_frame.y)
lower_leg_mass_center.pos_from(ankle)
upper_leg_mass_center = Point('U_o')
upper_leg_mass_center.set_pos(knee, upper_leg_com_length * upper_leg_frame.y)
upper_leg_mass_center.pos_from(ankle)
torso_mass_center = Point('T_o')
torso_mass_center.set_pos(hip, torso_com_length * torso_frame.y)
torso_mass_center.pos_from(ankle)

#kinematic differential equations
#init var for GENERALIZED speeds (aka angular velocities)
omega1, omega2, omega3 = dynamicsymbols('omega1, omega2, omega3')
kinematical_differential_equations = [omega1 - theta1.diff(),
                                      omega2 - theta2.diff(),
                                      omega3 - theta3.diff()]
#pretty_print(kinematical_differential_equations)

lower_leg_frame.set_ang_vel(inertial_frame,omega1*inertial_frame.z)
upper_leg_frame.set_ang_vel(lower_leg_frame, omega2 * inertial_frame.z)
torso_frame.set_ang_vel(upper_leg_frame, omega3 * inertial_frame.z)

#set ankle as fixed in place
ankle.set_vel(inertial_frame, 0)
#get linear velocities of each point
lower_leg_mass_center.v2pt_theory(ankle, inertial_frame, lower_leg_frame)
knee.v2pt_theory(ankle, inertial_frame, lower_leg_frame)
upper_leg_mass_center.v2pt_theory(knee, inertial_frame, upper_leg_frame)
hip.v2pt_theory(knee, inertial_frame, upper_leg_frame)
torso_mass_center.v2pt_theory(hip, inertial_frame, torso_frame)

#Inertia---------------------------------------------------------------------
#this problem is the case of a 2D planar system, all rotations are about the z axis
lower_leg_mass, upper_leg_mass, torso_mass = symbols('m_L, m_U, m_T')
lower_leg_inertia, upper_leg_inertia, torso_inertia = symbols('I_Lz, I_Uz, I_Tz')

lower_leg_inertia_dyadic = inertia(lower_leg_frame, 0, 0, lower_leg_inertia)
#pretty_print(lower_leg_inertia_dyadic.to_matrix(lower_leg_frame)) #very simple matrix because 2D case
#define the point at which the inertia of a body is in respect to
lower_leg_central_inertia = (lower_leg_inertia_dyadic, lower_leg_mass_center)
#repeat for other 2 links
upper_leg_inertia_dyadic = inertia(upper_leg_frame, 0, 0, upper_leg_inertia)
upper_leg_central_inertia = (upper_leg_inertia_dyadic, upper_leg_mass_center)
torso_inertia_dyadic = inertia(torso_frame, 0, 0, torso_inertia)
torso_central_inertia = (torso_inertia_dyadic, torso_mass_center)

#completely define rigid bodies
lower_leg = RigidBody('Lower Leg', lower_leg_mass_center, lower_leg_frame,
                      lower_leg_mass, lower_leg_central_inertia)
upper_leg = RigidBody('Upper Leg', upper_leg_mass_center, upper_leg_frame,
                      upper_leg_mass, upper_leg_central_inertia)
torso = RigidBody('Torso', torso_mass_center, torso_frame,
                  torso_mass, torso_central_inertia)

#Kinetics---------------------------------------------------------------------
#gravity
g = symbols('g')

#Forces must be exerted at a point
lower_leg_grav_force_vector = -lower_leg_mass * g * inertial_frame.y
lower_leg_grav_force = (lower_leg_mass_center, lower_leg_grav_force_vector)
upper_leg_grav_force = (upper_leg_mass_center, -upper_leg_mass * g * inertial_frame.y)
torso_grav_force = (torso_mass_center, -torso_mass * g * inertial_frame.y)

#joint torques
ankle_torque, knee_torque, hip_torque = dynamicsymbols('T_a, T_k, T_h')

lower_leg_torque_vector = ankle_torque * inertial_frame.z - knee_torque * inertial_frame.z
lower_leg_torque = (lower_leg_frame, lower_leg_torque_vector)

upper_leg_torque = (upper_leg_frame, knee_torque * inertial_frame.z - hip_torque * inertial_frame.z)
torso_torque = (torso_frame, hip_torque * inertial_frame.z)

#Equations of Motion------------------------------------------------------------
#At the bare minimum for unconstrained systems, the KanesMethod class needs to know
# the generalized coordinates, the generalized speeds, kinematical differential equations,
# loads, the bodies, and a Newtonian reference frame
coordinates = [theta1, theta2, theta3]
speeds = [omega1, omega2, omega3]

kane = KanesMethod(inertial_frame, coordinates, speeds, kinematical_differential_equations)
loads = [#lower_leg_grav_force,
         #upper_leg_grav_force,
         #torso_grav_force, 
         lower_leg_torque,
         upper_leg_torque,
         torso_torque]
bodies = [lower_leg, upper_leg, torso]

#fr + frstar = 0
#M(q,t)u˙=f(q,q˙,u,t)
fr, frstar = kane.kanes_equations(bodies,loads)
#pretty_print(trigsimp(fr + frstar))

#TODO may need to NOT simplify MM and FV for performance
mass_matrix = trigsimp(kane.mass_matrix_full)
#pretty_print(mass_matrix)
forcing_vector = trigsimp(kane.forcing_full)
#pretty_print(forcing_vector)

#simulation--------------------------------------------------------------------
constants = [lower_leg_length,
             lower_leg_com_length,
             lower_leg_mass,
             lower_leg_inertia,
             upper_leg_length,
             upper_leg_com_length,
             upper_leg_mass,
             upper_leg_inertia,
             torso_com_length,
             torso_mass,            
             torso_inertia,
             g]
#declare torque magnitude variables
specified = [ankle_torque, knee_torque, hip_torque]

#generate ODE function that numerically evaluates the RHS of first order diffeq 
#odeint is ODE integrator
right_hand_side = generate_ode_function(forcing_vector, coordinates,
                                        speeds, constants,
                                        mass_matrix=mass_matrix,
                                        specifieds=specified)
#right_hand_side is a FUNCTION
#initial values for system
x0 = np.zeros(6)
# x0[:3] = deg2rad(2.0)
x0[0] = np.deg2rad(45)
x0[1] = np.deg2rad(45)
x0[2] = np.deg2rad(45)
x0[3] = np.deg2rad(45)
# x0[4] = deg2rad(-180)
# x0[5] = deg2rad(180)

numerical_constants = np.array([0.611,  # lower_leg_length [m]
                             0.387,  # lower_leg_com_length [m]
                             6.769,  # lower_leg_mass [kg]
                             0.101,  # lower_leg_inertia [kg*m^2]
                             0.424,  # upper_leg_length [m]
                             0.193,  # upper_leg_com_length
                             17.01,  # upper_leg_mass [kg]
                             0.282,  # upper_leg_inertia [kg*m^2]
                             0.305,  # torso_com_length [m]
                             32.44,  # torso_mass [kg]
                             1.485,  # torso_inertia [kg*m^2]
                             9.81],  # acceleration due to gravity [m/s^2]
                            ) 
#set joint torques to zero for first simulation
#numerical_specified = zeros(3)

#set equilibrium point as straight up at zero velocity
equilibrium_point = np.zeros(len(coordinates + speeds))
# equilibrium_point[0] = 1 #does not work
# equilibrium_point[1] = np.pi/3
# equilibrium_point[2] = pi/3
#equilibrium_point[3] = deg2rad(45) #should not work and does not work

#create dict containing numerical values of equilibrium point
equilibrium_dict = dict(zip(coordinates + speeds, equilibrium_point))
pretty_print(equilibrium_dict)

#maps symbolic constants to numerical constants
parameter_dict = dict(zip(constants, numerical_constants))

#create object to hold generalized form of EOM to be used to linearize the system
linearizer = kane.to_linearizer()
#linearizer defaults to alphabetical sorting- don't use that
linearizer.r = sympy.Matrix(specified)

#use linearizer object to create symbolic linearization of system
#BE CAREFUL: arg <A_and_B=True> is VERY computationally expensive
#old strategy, works with zero operating points and nothing else
A, B = linearizer.linearize(op_point=[equilibrium_dict, parameter_dict], A_and_B=True)

# op_point = {theta1: 0, theta2: 0, theta3:0,omega1: 0, omega2: 0, omega3: 0}
#new attempt- not working
#A, B = linearizer.linearize(A_and_B=True, op_point=op_point)

# A, B, inp_vec = kane.linearize(A_and_B=True, op_point=op_point, new_method = True)

#convert to numpy
A = sympy.matrix2numpy(A, dtype=float)
B = sympy.matrix2numpy(B, dtype=float)
# pretty_print(A)
# pretty_print(B)

#C gets positon states for output y
C = np.zeros((6,6))
C[0,0] = 1
C[1,1] = 1
C[2,2] = 1
D = np.zeros((6,3))

SS = control.StateSpace(A,B,C,D)
print(SS)

#true if system is controllable 
#print(controllable(A,B))

#setup cost function that minimizes deviation from desired state
#should also minimize joint torques
#cost func J = 0.5*integral_of((x^T)Qx + (u^T)Ru)dt from t=0 to t=infinity

#There exists a matrix K that calculates optimal inputs u(t) as func of given states x
# u(t) = -K*x(t)
# K can be calculated from the RICCATI EQUATION as follows:
# K = (R^-1)(B^T)S 

#Generate weighting matrices Q and R that allow how much solution should be weighted towards
# minimizing error in states vs effort in the inputs u
#(Usually a good idea to start out with identity matrices)
Q = np.eye(6) #*1000 # if you make everything big, velocity still wants to be zero
Q[0,0] = 10000000
Q[1,1] = 10000000
Q[2,2] = 10000000

R = np.eye(3)

#compute K using python-control library
# Kstar = control.care(A,B,Q,R) #needs slycot lib
# print("Kstar = ", Kstar)

#compute K using method from pydy tutorial
S = solve_continuous_are(A, B, Q, R)
K = np.dot(np.dot(np.linalg.inv(R), B.T),  S)
pretty_print(K)
# K[1,1] = 0 #kill joint 1 by setting this to zero

def controller(x, t):
    """Returns the output of the controller, i.e. the joint torques, given
    the current state."""
    return -np.dot(K, x)

# args = {'constants': numerical_constants,
#         'specified': numerical_specified}

frames_per_sec = 60
final_time = 10
t = np.linspace(0.0, final_time, final_time * frames_per_sec)

#integrate equations of motion
right_hand_side(x0, 0.0, controller(x0,t), numerical_constants)

#create variable to store trajectories of states as func of time
#NO CONTROL 
#y = odeint(right_hand_side, x0, t, args=(numerical_specified, numerical_constants))
#Control
y = odeint(right_hand_side,x0,t,args=(controller, numerical_constants))

#plot trajectory of each joint
# fig = plt.figure(1)
# ax = fig.add_subplot()
# plt.plot(t, rad2deg(y[:, :3])) #generalized positions-first 3 states
# plt.draw()
# plt.pause(1)

#Visualization-------------------------------------------------------------------
#print(pydy.viz.shapes.__all__)

#draw spheres at each joint
ankle_shape = Sphere(color='black', radius=0.1)
knee_shape = Sphere(color='black', radius=0.1)
hip_shape = Sphere(color='black', radius=0.1)
head_shape = Sphere(color='black', radius=0.125)

#create VisualizationFrame - attaches a shape to a reference frame and a point
ankle_viz_frame = VisualizationFrame(inertial_frame, ankle, ankle_shape)
knee_viz_frame = VisualizationFrame(inertial_frame, knee, knee_shape)
hip_viz_frame = VisualizationFrame(inertial_frame, hip, hip_shape)

#create "Head"
head = Point('N')  # N for Noggin
head.set_pos(hip, 2 * torso_com_length * torso_frame.y)
head_viz_frame = VisualizationFrame(inertial_frame, head, head_shape)

#make cylindrical links to connect joints
lower_leg_center = Point('l_c')
upper_leg_center = Point('u_c')
torso_center = Point('t_c')
lower_leg_center.set_pos(ankle, lower_leg_length / 2 * lower_leg_frame.y)
upper_leg_center.set_pos(knee, upper_leg_length / 2 * upper_leg_frame.y)
torso_center.set_pos(hip, torso_com_length * torso_frame.y)

constants_dict = dict(zip(constants, numerical_constants))

lower_leg_shape = Cylinder(radius=0.08, length=constants_dict[lower_leg_length], color='blue')
lower_leg_viz_frame = VisualizationFrame('Lower Leg', lower_leg_frame, lower_leg_center, lower_leg_shape)

upper_leg_shape = Cylinder(radius=0.08, length=constants_dict[upper_leg_length], color='green')
upper_leg_viz_frame = VisualizationFrame('Upper Leg', upper_leg_frame, upper_leg_center, upper_leg_shape)

torso_shape = Cylinder(radius=0.08, length=2 * constants_dict[torso_com_length], color='red')
torso_viz_frame = VisualizationFrame('Torso', torso_frame, torso_center, torso_shape)

scene = Scene(inertial_frame, ankle)
#make list of frames we want in the scene
scene.visualization_frames = [ankle_viz_frame,
                              knee_viz_frame,
                              hip_viz_frame,
                              head_viz_frame,
                              lower_leg_viz_frame,
                              upper_leg_viz_frame,
                              torso_viz_frame]
scene.states_symbols = coordinates + speeds
scene.constants = constants_dict
scene.states_trajectories = y
scene.display()
#scene.display_ipython()