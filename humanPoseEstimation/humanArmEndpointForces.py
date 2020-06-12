from __future__ import print_function, division
from sympy import symbols, simplify, trigsimp
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame, Point, inertia, RigidBody, KanesMethod
from sympy.physics.vector import init_vprinting, vlatex
from IPython.display import Image
from sympy.printing.pretty.pretty import pretty_print
from numpy import deg2rad, rad2deg, array, zeros, linspace
from scipy.integrate import odeint
from pydy.codegen.ode_function_generators import generate_ode_function
import matplotlib.pyplot as plt
from pydy.viz.shapes import Cylinder, Sphere
import pydy.viz
from pydy.viz.visualization_frame import VisualizationFrame
from pydy.viz.scene import Scene
import cloudpickle
import os
import inspect

#this file is used to generate serialized function to predict end effector inertia
# ellipse as function of current state

init_vprinting(use_latex='mathjax', pretty_print=True)

#Kinematics ------------------------------
#init reference frames, assume base attached to floor
inertial_frame = ReferenceFrame('I')
j0_frame = ReferenceFrame('J0')
j1_frame = ReferenceFrame('J1')
j2_frame = ReferenceFrame('J2')

#declare dynamic symbols for the three joints
theta0, theta1, theta2 = dynamicsymbols('theta0, theta1, theta2')

#orient frames
j0_frame.orient(inertial_frame,'Axis',(theta0, inertial_frame.y))
j1_frame.orient(j0_frame,'Axis',(theta1, j0_frame.z))
j2_frame.orient(j1_frame,'Axis',(theta2, j1_frame.x))

#TODO figure out better name for joint points
#init joints
joint0 = Point('j0')
j0_length = symbols('l_j0')
joint1 = Point('j1')
joint1.set_pos(joint0, j0_length*j0_frame.y)
j1_length = symbols('l_j1')
joint2 = Point('j2')
joint2.set_pos(joint1,j1_length*j1_frame.y)
#create End Effector
EE = Point('E')


#COM locations
j0_com_length, j1_com_length, j2_com_length = symbols('d_j0, d_j1, d_j2')
j0_mass_center = Point('j0_o')
j0_mass_center.set_pos(joint0, j0_com_length * j0_frame.y)
j1_mass_center = Point('j1_o')
j1_mass_center.set_pos(joint1, j1_com_length * j1_frame.y)
j2_mass_center = Point('j2_o')
j2_mass_center.set_pos(joint2, j2_com_length * j2_frame.y)
EE.set_pos(joint2, j2_com_length*2*j2_frame.y)
#kinematic differential equations
#init var for generalized speeds (aka angular velocities)
omega0, omega1, omega2 = dynamicsymbols('omega0, omega1, omega2')
kinematical_differential_equations = [omega0 - theta0.diff(),
                                    omega1 - theta1.diff(),
                                    omega2 - theta2.diff()]
pretty_print(kinematical_differential_equations)

j0_frame.set_ang_vel(inertial_frame,omega0*j0_frame.y)
j1_frame.set_ang_vel(j0_frame,omega1*j0_frame.z)
j2_frame.set_ang_vel(j1_frame,omega2*j1_frame.x)

#set base link fixed to ground plane
joint0.set_vel(inertial_frame,0)
#get linear velocities of each point
j0_mass_center.v2pt_theory(joint0, inertial_frame, j0_frame)
joint1.v2pt_theory(joint0, inertial_frame, j0_frame)
j1_mass_center.v2pt_theory(joint1, inertial_frame, j1_frame)
joint2.v2pt_theory(joint1, inertial_frame, j1_frame)
j2_mass_center.v2pt_theory(joint2, inertial_frame, j2_frame)
EE.v2pt_theory(joint2,inertial_frame,j2_frame)

print("finished Kinematics")

#Inertia---------------------------------------------------------------
j0_mass, j1_mass, j2_mass = symbols('m_j0, m_j1, m_j2')
j0_inertia, j1_inertia, j2_inertia = symbols('I_j0, I_j1, I_j2')

#inertia values taken from CAD model (same as simscape multibody) [kg*m^2]
#<joint>.inertia(frame, ixx, iyy, izz, ixy = _, iyz= _, izx = _)
j0_inertia_dyadic = inertia(j0_frame, 0.012, 0.01, 0.0114, ixy = 0.00007, iyz = -0.00002, izx = -0.0002) #not sure how important this is for ball and socket joint (shoulder)
j0_central_inertia = (j0_inertia_dyadic, j0_mass_center)
#pretty_print(j0_inertia_dyadic.to_matrix(j0_frame))

j1_inertia_dyadic = inertia(j1_frame, 0.024, 0.003, 0.24, ixy = 0.001, iyz = -0.001, izx = 0.00)
j1_central_inertia = (j1_inertia_dyadic, j1_mass_center)
#pretty_print(j1_inertia_dyadic.to_matrix(j1_frame))

j2_inertia_dyadic = inertia(j2_frame, 0.018, 0.001, 0.018, ixy = 0.000, iyz = 0.000, izx = 0.00)
j2_central_inertia = (j2_inertia_dyadic, j2_mass_center)
#pretty_print(j2_inertia_dyadic.to_matrix(j2_frame))

#fully define rigid bodies
link0 = RigidBody('Link 0', j0_mass_center, j0_frame, j0_mass, j0_central_inertia)
link1 = RigidBody('Link 1', j1_mass_center, j1_frame, j1_mass, j1_central_inertia)
link2 = RigidBody('Link 2', j2_mass_center, j2_frame, j2_mass, j2_central_inertia)

print("finished Inertia")

#Kinetics---------------------------------------------------------------
g = symbols('g')

#ASSUMES GRAVITY BEING COMPENSATED BY ROBOT
#exert forces at a point
#j0_grav_force = (j0_mass_center, -j0_mass * g * inertial_frame.y)
j0_grav_force = (j0_mass_center, 0*inertial_frame.y) #perp to dir of grav
j1_grav_force = (j1_mass_center, -j1_mass * g * inertial_frame.y)
#j1_grav_force = (j1_mass_center, 0*inertial_frame.y)
j2_grav_force = (j2_mass_center, -j2_mass * g * inertial_frame.y)

EE_force_x, EE_force_y, EE_force_z = symbols('Fx,Fy,Fz')
EE_force = (EE,EE_force_x*inertial_frame.x + EE_force_y*inertial_frame.y + EE_force_z*inertial_frame.z)

#joint torques
j0_torque, j1_torque, j2_torque = dynamicsymbols('T_j0, T_j1, T_j2')
# l0_torque = (j0_frame, j0_torque * inertial_frame.y + j1_torque * inertial_frame.y)
l0_torque = (j0_frame, j0_torque * j0_frame.y + j1_torque * j0_frame.y) #debug
l1_torque = (j1_frame, j1_torque * inertial_frame.z - j2_torque * inertial_frame.z)
l2_torque = (j2_frame, j2_torque * inertial_frame.x)

# l0_torque = (j0_frame, j0_torque * inertial_frame.y + j1_torque * inertial_frame.y)
# l1_torque = (j1_frame, j1_torque * j1_frame.z - j2_torque * j2_frame.z)
# l2_torque = (j2_frame, j2_torque * j2_frame.z)

print("finished Kinetics")

#Equations of Motion----------------------------------------------------
coordinates = [theta0, theta1, theta2]
speeds = [omega0, omega1, omega2]

#kane is object
kane = KanesMethod(inertial_frame, coordinates, speeds, kinematical_differential_equations)

loads = [#j0_grav_force, 
		 #j1_grav_force, 
		 #j2_grav_force,
		 EE_force,
		 l0_torque,
		 l1_torque,
		 l2_torque]
bodies = [link0, link1, link2]

#fr + frstar = 0
#M(q,t)u˙=f(q,q˙,u,t)
fr, frstar = kane.kanes_equations(bodies,loads)

print("finished KanesMethod")

#mass_matrix = trigsimp(kane.mass_matrix_full)
mass_matrix = kane.mass_matrix_full

# mass_matrix_file = "massMatrix.txt"
# dill.dump(mass_matrix,open(mass_matrix_file, 'wb'))

print("finished mass_matrix")
# pretty_print(mass_matrix)
#forcing_vector = trigsimp(kane.forcing_full)
forcing_vector = kane.forcing_full

print("finished forcing_vector")
# pretty_print(forcing_vector)

print("finished Equations of Motion")

#Simulation -----------------------------------------------------------
constants = [j0_length,
			 j0_com_length,
			 j0_mass,
			 j0_inertia,
			 j1_length,
			 j1_com_length,
			 j1_mass,
			 j1_inertia,
			 j2_com_length,
			 j2_mass,
			 j2_inertia,
			 g,
			 EE_force_x,
			 EE_force_y,
			 EE_force_z]
specified = [j0_torque, j1_torque, j2_torque]

#generate ODE function that numerically evaluates the RHS of first order diffeq 
#odeint is ODE integrator
right_hand_side = generate_ode_function(forcing_vector, coordinates,
                                        speeds, constants,
                                        mass_matrix=mass_matrix,
                                        specifieds=specified)

#store right_hand_side to dill file so we don't have to go through solving every time
EOM_file = "human_inertia_func.txt"
# dill.dump(right_hand_side, open(EOM_file, 'wb'))
# rhsString = dill.dumps(right_hand_side)
# print(rhsString)

# cloudPickleString = cloudpickle.dumps(right_hand_side)
# print(cloudPickleString)

cloudpickle.dump(right_hand_side,open(EOM_file, 'wb'))

print("generated right_hand_side")
print(right_hand_side)

print(os.path.abspath(inspect.getfile(right_hand_side)))

#right_hand_side is a FUNCTION
#initial values for system
x0 = zeros(6)
# x0[:3] = deg2rad(2.0) 
#start with pendulum upside down
x0[0] = deg2rad(51)
x0[1] = deg2rad(0)
x0[2] = deg2rad(104)
#initial vel
#x0[3] = deg2rad(180)
#x0[4] = deg2rad(-90)
#x0[5] = 0


#Tool used for estimating mass of arm segments (Matt is 6'5" and weighs 200lbs)
# http://robslink.com/SAS/democd79/body_part_weights.htm

numerical_constants = array([0.0,  # j0_length [m]
                             0.0,  # j0_com_length [m]
                             4.0,  # j0_mass [kg]
                             0.001,  # NOT USED j0_inertia [kg*m^2]
                             0.264,  # j1_length [m]
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

#set joint torques to zero for first simulation
numerical_specified = zeros(3)
# numerical_specified[0] = 0.1 #changing this value will add constant torque to joint
args = {'constants': numerical_constants,
        'specified': numerical_specified}
frames_per_sec = 60
final_time = 3
t = linspace(0.0, final_time, final_time * frames_per_sec)

#integrate equations of motion
right_hand_side(x0, 0.0, numerical_specified, numerical_constants)

#create variable to store trajectories of states as func of time
y = odeint(right_hand_side, x0, t, args=(numerical_specified, numerical_constants))

#plot trajectory of each joint
# fig = plt.figure(1)
# ax = fig.add_subplot()
# plt.plot(t, rad2deg(y[:, :3])) #generalized positions-first 3 states
# plt.draw()
# plt.pause(30)

#visualization ----------------------------------------------------------------
#print(pydy.shapes.__all__)

#draw spheres at each joint
j0_shape = Sphere(color='black',radius = 0.025)
j1_shape = Sphere(color='black',radius = 0.025)
j2_shape = Sphere(color='peachpuff',radius = 0.025)
EE_shape = Sphere(color='red',radius = 0.025)

#create VisualizationFrame - attaches a shape to a reference frame and a point
j0_viz_frame = VisualizationFrame(inertial_frame, joint0, j0_shape)
j1_viz_frame = VisualizationFrame(inertial_frame, joint1, j1_shape)
j2_viz_frame = VisualizationFrame(inertial_frame, joint2, j2_shape)
EE_viz_frame = VisualizationFrame(inertial_frame, EE, EE_shape)

#make cylindrical links to connect joints
j0_center = Point('j0_c')
j1_center = Point('j1_c')
j2_center = Point('j2_c')
j0_center.set_pos(joint0, j0_length / 2 * j0_frame.y)
j1_center.set_pos(joint1, j1_length / 2 * j1_frame.y)
j2_center.set_pos(joint2, j2_com_length * j2_frame.y)

constants_dict = dict(zip(constants, numerical_constants))

l0_shape = Cylinder(radius=0.025, length=constants_dict[j0_length], color = 'peachpuff')
l0_viz_frame = VisualizationFrame('Link 0', j0_frame, j0_center, l0_shape)

l1_shape = Cylinder(radius=0.025, length=constants_dict[j1_length], color = 'peachpuff')
l1_viz_frame = VisualizationFrame('Link 1', j1_frame, j1_center, l1_shape)

l2_shape = Cylinder(radius=0.025, length=constants_dict[j2_com_length]*2, color = 'peachpuff')
l2_viz_frame = VisualizationFrame('Link 2', j2_frame, j2_center, l2_shape)

scene = Scene(inertial_frame,joint0)
#make list of frames we want in scene
scene.visualization_frames = [j0_viz_frame,
							  j1_viz_frame,
							  j2_viz_frame,
							  EE_viz_frame,
							  l0_viz_frame,
							  l1_viz_frame,
							  l2_viz_frame]
scene.states_symbols = coordinates + speeds
scene.constants = constants_dict
scene.states_trajectories = y
scene.display()