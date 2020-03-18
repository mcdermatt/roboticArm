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

#init joints
joint0 = Point('0')
j0_length = symbols('l_j0')
joint1 = Point('1')
joint1.set_pos(joint0, j0_length*j0_frame.y) #not sure if correct coordinate direction
j1_length = symbols('l_j1')
joint2 = Point('2')
joint2.set_pos(joint1,j1_length*j1_frame.y) #also not sure if .y is correct

#COM locations


