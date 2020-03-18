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
kinematic_differential_equations = [omega0 - theta0.diff(),
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

#Kinetics---------------------------------------------------------------
g = symbols('g')

#exert forces at a point
j0_grav_force = (j0_mass_center, -j0_mass * g * inertial_frame.y)
j1_grav_force = (j1_mass_center, -j1_mass * g * inertial_frame.y)
j2_grav_force = (j2_mass_center, -j2_mass * g * inertial_frame.y)

#joint torques
j0_torque, j1_torque, j2_torque = dynamicsymbols('T_j0, T_j1, T_j2')

