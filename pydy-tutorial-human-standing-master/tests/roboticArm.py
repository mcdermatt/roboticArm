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
j2_frame.orient(j1_frame,'Axis',(theta2, j1_frame.z))

#TODO figure out better name for joint points
#init joints
joint0 = Point('j0')
j0_length = symbols('l_j0')
joint1 = Point('j1')
joint1.set_pos(joint0, j0_length*j0_frame.y) #not sure if correct coordinate direction
j1_length = symbols('l_j1')
joint2 = Point('j2')
joint2.set_pos(joint1,j1_length*j1_frame.y) #also not sure if .y is correct

#COM locations
j0_com_length, j1_com_length, j2_com_length = symbols('d_j0, d_j1, d_j2')
j0_mass_center = Point('j0_o')
j0_mass_center.set_pos(joint0, j0_com_length * j0_frame.y)
j1_mass_center = Point('j1_o')
j1_mass_center.set_pos(joint1, j1_com_length * j1_frame.y)
j2_mass_center = Point('j2_o')
j2_mass_center.set_pos(joint2, j2_com_length * j2_frame.y)

#kinematic differential equations
#init var for generalized speeds (aka angular velocities)
omega0, omega1, omega2 = dynamicsymbols('omega0, omega1, omega2')
kinematical_differential_equations = [omega0 - theta0.diff(),
                                    omega1 - theta1.diff(),
                                    omega2 - theta2.diff()]
#pretty_print(kinematic_differential_equations)

j0_frame.set_ang_vel(inertial_frame,omega0*inertial_frame.z)
j1_frame.set_ang_vel(j0_frame,omega1*inertial_frame.z)
j2_frame.set_ang_vel(j1_frame,omega2*inertial_frame.z)

#set base link fixed to ground plane
joint0.set_vel(inertial_frame,0)
#get linear velocities of each point
j0_mass_center.v2pt_theory(joint0, inertial_frame, j0_frame)
joint1.v2pt_theory(joint0, inertial_frame, j0_frame)
j1_mass_center.v2pt_theory(joint1, inertial_frame, j1_frame)
joint2.v2pt_theory(joint1, inertial_frame, j1_frame)
j2_mass_center.v2pt_theory(joint2, inertial_frame, j2_frame)

print("finished Kinematics")

#Inertia---------------------------------------------------------------
j0_mass, j1_mass, j2_mass = symbols('m_j0, m_j1, m_j2')
j0_inertia, j1_inertia, j2_inertia = symbols('I_j0, I_j1, I_j2')

#inertia values taken from CAD model (same as simscape multibody) [kg*m^2]
#<joint>.inertia(frame, ixx, iyy, izz, ixy = _, iyz= _, izx = _)
j0_inertia_dyadic = inertia(j0_frame, 0.012, 0.01, 0.0114, ixy = 0.00007, iyz = -0.00002, izx = -0.0002)
j0_central_inertia = (j0_inertia_dyadic, j0_mass_center)
#pretty_print(j0_inertia_dyadic.to_matrix(j0_frame))

j1_inertia_dyadic = inertia(j1_frame, 0.016, 0.008, 0.0125, ixy = 0.0016, iyz = -0.0012, izx = 0.0011)
j1_central_inertia = (j1_inertia_dyadic, j1_mass_center)
#pretty_print(j1_inertia_dyadic.to_matrix(j1_frame))

j2_inertia_dyadic = inertia(j2_frame, 0.0118, 0.009, 0.007, ixy = 0.0006, iyz = -0.00089, izx = 0.006)
j2_central_inertia = (j2_inertia_dyadic, j2_mass_center)
#pretty_print(j2_inertia_dyadic.to_matrix(j2_frame))

#fully define rigid bodies
link0 = RigidBody('Link 0', j0_mass_center, j0_frame, j0_mass, j0_central_inertia)
link1 = RigidBody('Link 1', j1_mass_center, j1_frame, j1_mass, j1_central_inertia)
link2 = RigidBody('Link 2', j2_mass_center, j2_frame, j2_mass, j2_central_inertia)

print("finished Inertia")

#Kinetics---------------------------------------------------------------
g = symbols('g')

#exert forces at a point
j0_grav_force = (j0_mass_center, -j0_mass * g * inertial_frame.y)
j1_grav_force = (j1_mass_center, -j1_mass * g * inertial_frame.y)
j2_grav_force = (j2_mass_center, -j2_mass * g * inertial_frame.y)

#joint torques
j0_torque, j1_torque, j2_torque = dynamicsymbols('T_j0, T_j1, T_j2')
l0_torque = (j0_frame, j0_torque * inertial_frame.z - j1_torque * inertial_frame.y)
l1_torque = (j1_frame, j1_torque * inertial_frame.z - j2_torque * inertial_frame.z)
l2_torque = (j2_frame, j2_torque * inertial_frame.z)

print("finished Kinetics")

#Equations of Motion----------------------------------------------------
coordinates = [theta0, theta1, theta2]
speeds = [omega0, omega1, omega2]

kane = KanesMethod(inertial_frame, coordinates, speeds, kinematical_differential_equations)

loads = [j0_grav_force, 
		 j1_grav_force, 
		 j2_grav_force,
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
print("finished mass_matrix")
#pretty_print(mass_matrix)
# forcing_vector = trigsimp(kane.forcing_full)
forcing_vector = kane.forcing_full
#pretty_print(forcing_vector)
print("finished forcing_vector")

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
			 g]
specified = [j0_torque, j1_torque, j2_torque]

#generate ODE function that numerically evaluates the RHS of first order diffeq 
#odeint is ODE integrator
right_hand_side = generate_ode_function(forcing_vector, coordinates,
                                        speeds, constants,
                                        mass_matrix=mass_matrix,
                                        specifieds=specified)
print("generated right_hand_side")

#right_hand_side is a FUNCTION
#initial values for system
x0 = zeros(6)
# x0[:3] = deg2rad(2.0) 
#start with pendulum upside down
x0[1] = deg2rad(150)

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
                             9.81],  # acceleration due to gravity [m/s^2]
                            ) 

#set joint torques to zero for first simulation
numerical_specified = zeros(3)
args = {'constants': numerical_constants,
        'specified': numerical_specified}
frames_per_sec = 60
final_time = 10
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
j2_shape = Sphere(color='black',radius = 0.025)
EE_shape = Sphere(color='red',radius = 0.025)

#create VisualizationFrame - attaches a shape to a reference frame and a point
j0_viz_frame = VisualizationFrame(inertial_frame, joint0, j0_shape)
j1_viz_frame = VisualizationFrame(inertial_frame, joint1, j1_shape)
j2_viz_frame = VisualizationFrame(inertial_frame, joint2, j2_shape)

#create End Effector
EE = Point('E')
EE.set_pos(joint2, j2_com_length*j2_frame.y)
EE_viz_frame = VisualizationFrame(inertial_frame, EE, EE_shape)

#make cylindrical links to connect joints
j0_center = Point('j0_c')
j1_center = Point('j1_c')
j2_center = Point('j2_c')
j0_center.set_pos(joint0, j0_length / 2 * j0_frame.y)
j1_center.set_pos(joint1, j1_length / 2 * j1_frame.y)
j2_center.set_pos(joint2, j2_com_length / 2 * j2_frame.y)

constants_dict = dict(zip(constants, numerical_constants))

l0_shape = Cylinder(radius=0.01, length=constants_dict[j0_length], color = 'blue')
l0_viz_frame = VisualizationFrame('Link 0', j0_frame, j0_center, l0_shape)

l1_shape = Cylinder(radius=0.01, length=constants_dict[j1_length], color = 'blue')
l1_viz_frame = VisualizationFrame('Link 1', j1_frame, j1_center, l1_shape)

l2_shape = Cylinder(radius=0.01, length=constants_dict[j2_com_length], color = 'blue')
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