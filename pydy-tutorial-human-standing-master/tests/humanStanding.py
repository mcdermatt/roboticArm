from __future__ import print_function, division
from sympy import symbols, simplify, trigsimp
from sympy.physics.mechanics import dynamicsymbols, ReferenceFrame, Point, inertia, RigidBody, KanesMethod
from sympy.physics.vector import init_vprinting
from IPython.display import Image
from sympy.printing.pretty.pretty import pretty_print


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
loads = [lower_leg_grav_force,
         upper_leg_grav_force,
         torso_grav_force, 
         lower_leg_torque,
         upper_leg_torque,
         torso_torque]
bodies = [lower_leg, upper_leg, torso]

#TODO - figure out significance of fr and frstar
fr, frstar = kane.kanes_equations(bodies,loads)
#pretty_print(trigsimp(fr + frstar))

mass_matrix = trigsimp(kane.mass_matrix_full)
pretty_print(mass_matrix)
forcing_vector = trigsimp(kane.forcing_full)
pretty_print(forcing_vector)

#simulation--------------------------------------------------------------------
