from __future__ import print_function
import ctypes
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import time
import odrive
from odrive.enums import*
import math
import numpy as np
import keyboard

#OpenGL + Odrive Reference Tracking on joints 0 1 and 2
#uses closed loop position control to mimic trajectory arm was moved through in handGuidedPathV2.py

#code is combination of armV8RenderTest2.py (real time opengl simulation) and playbackArmPath.py (odrive communication)
#displays what arm SHOULD be doing

#system parameters
#l0- shoulder side/side
serial0 = "206A337B304B"
l0cpr = 90
l0reduction = 6

#l1- shoulder up/down
l1m = 3.07 #kg was 3.07
l1com = 0.0729 #m from center of rotation
l1 = 0.1651
l1cpr = 90
l1reduction = 6 #was 9, switched to og opentorque for less friction
l1kv = 16
#beta1 = 0
beta1 = -0.025 #to do- make beta a function of motor velocity to cancel out inertia? was -0.025 with 9:1
serial1 = "2084377D3548"

#l2- elbow
l2m = 3.1 #kg was 1.85, moved up to compensate for wrist? was 2.6 before end effector
l2com = 0.30 #m was 0.1893 moved up to compensate for wrist and end effector
l2cpr = 8192 
l2reduction = 6
l2kv = 100
beta2 = -0.000005
serial2 = "205737993548"

print("finding odrive0")
od0 = odrive.find_any(serial_number=serial0)
print("finding odrive2")
od2 = odrive.find_any(serial_number=serial2)
print("finding odrive1")
od1 = odrive.find_any(serial_number=serial1)

#arm moves through pre-recorded sequence
print("WARNING PROGRAM ASSUMES ARM IS CALIBRATED TO VERTICAL POSITION")
print("move arm to downward resting position")
time.sleep(5)
print("press q for ESTOP")
time.sleep(1)

#sets each axis to reference tracking position control (PID)
od0.axis0.controller.config.control_mode = 3
od1.axis0.controller.config.control_mode = 3
od2.axis0.controller.config.control_mode = 3

#all axis enter closed loop control
od0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
od1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
od2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


#init openGL stuff
window = pyglet.window.Window(width=1280, height=720)
keys = key.KeyStateHandler()
window.push_handlers(keys)

base = Wavefront('base.obj')
link0 = Wavefront('l0.obj')
link1 = Wavefront('l1.obj')
link2 = Wavefront('l2.obj')
link3 = Wavefront('l3.obj')
link4 = Wavefront('l4.obj')

l1 = 6.5
l2 = 6.5
l3 = 2.65

i = 0
pathArr = np.genfromtxt('armPath.txt',delimiter=" ")

# link0Rot,link1Rot,link2Rot = get_joint_angles(xIn,yIn,zIn)
link0Rot = (180/np.pi)*pathArr[i,0]
link1Rot = (180/np.pi)*pathArr[i,1]
link2Rot = (180/np.pi)*pathArr[i,2]
link3Rot = 0
link4Rot = 45

rotation = 0.0 #count variable for spinning
lightfv = ctypes.c_float * 4

link2RotEff = link1Rot + link2Rot

xElb = ( l1 * np.sin(link0Rot*(np.pi/180))*np.sin(link1Rot*(np.pi/180)))
yElb = ( l1 * np.cos((link1Rot*(np.pi/180)))) 
zElb =  ( l1 * np.cos(link0Rot*(np.pi/180))*np.sin(link1Rot*(np.pi/180)))

xl3 = xElb + ( l2 * np.sin(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))
yl3 = yElb + ( l2 * np.cos((link2RotEff*(np.pi/180)))) 
zl3 = zElb + ( l2 * np.cos(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))

xl4 = xElb + ( (l2+l3) * np.sin(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))
yl4 = yElb + ( (l2+l3) * np.cos((link2RotEff*(np.pi/180)))) 
zl4 = zElb + ( (l2+l3) * np.cos(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))

# cameraZ = -40


@window.event
def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(50., float(width)/height, 1., 100.) #change first argument for fov
    glTranslatef(0,-5,-40) #fits arm into camera view
    glMatrixMode(GL_MODELVIEW)
    return True


@window.event
def on_draw():
    window.clear()
    glClearColor(1,1,1,0.5) #sets background color
    glViewport(0,0,1280,720)
    glLoadIdentity()
    glMatrixMode(GL_PROJECTION)
    glRotatef(0,0,1,0)
    glRotatef(rotation*0.0035,0,1,0)
    glMatrixMode(GL_MODELVIEW)
    link0Rot = (180/np.pi)*pathArr[i,0]
    link1Rot = (180/np.pi)*pathArr[i,1]
    link2Rot = (180/np.pi)*pathArr[i,2]
    link3Rot = 0
    link4Rot = 45

    #get actual position and velocity of actuator encoders - will be useful for simulating actual arm position later
    #pos2 = od2.axis0.encoder.pos_estimate #- j2offset # assumes arm had been zeroed properly in handGuidedPath.py
    #vel2 = od2.axis0.encoder.vel_estimate
    #pos1 = od1.axis0.encoder.pos_estimate #- j1offset
    #vel1 = od1.axis0.encoder.vel_estimate
    #pos0 = od0.axis0.encoder.pos_estimate #- j0offset
    #vel0 = od0.axis0.encoder.vel_estimate
    #convert encoder positions to joint angles
    # theta2Actual = (2 * np.pi * pos2) / (l2cpr * l2reduction)
    # theta1Actual = (2 * np.pi * pos1) / (l1cpr * l1reduction)
    # theta0Actual = (2 * np.pi * pos0) / (l0cpr * l0reduction)
    # theta2effActual = theta2Actual + theta1Actual

    #convert to position setpoint for j0 j1 and j2
    j0encoderGoal = (link0Rot * l0cpr * l0reduction) / (np.pi * 2)
    od0.axis0.controller.pos_setpoint = j0encoderGoal

    j1encoderGoal = (link1Rot * l1cpr * l1reduction) / (np.pi * 2)
    od1.axis0.controller.pos_setpoint = j1encoderGoal

    j2encoderGoal = (link2Rot * l2cpr * l2reduction) / (np.pi * 2)
    od2.axis0.controller.pos_setpoint = j2encoderGoal


 #   rotation = 0.0 #count variable for spinning
    lightfv = ctypes.c_float * 4

    link2RotEff = link1Rot + link2Rot

    xElb = ( l1 * np.sin(link0Rot*(np.pi/180))*np.sin(link1Rot*(np.pi/180)))
    yElb = ( l1 * np.cos((link1Rot*(np.pi/180)))) 
    zElb =  ( l1 * np.cos(link0Rot*(np.pi/180))*np.sin(link1Rot*(np.pi/180)))

    xl3 = xElb + ( l2 * np.sin(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))
    yl3 = yElb + ( l2 * np.cos((link2RotEff*(np.pi/180)))) 
    zl3 = zElb + ( l2 * np.cos(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))

    xl4 = xElb + ( (l2+l3) * np.sin(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))
    yl4 = yElb + ( (l2+l3) * np.cos((link2RotEff*(np.pi/180)))) 
    zl4 = zElb + ( (l2+l3) * np.cos(link0Rot*(np.pi/180))*np.sin(link2RotEff*(np.pi/180)))

    #glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0*np.sin(rotation*0.1), 1.0, 0.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 0.9))
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightfv(0.0,0.0,0.0,0.1))

    draw_base(base)
    draw_link0(link0, 0, 0, 0, link0Rot)
    draw_link1(link1, 0, 0, 0,link0Rot, link1Rot)
    draw_link2(link2, xElb, yElb, zElb, link0Rot, link1Rot, link2Rot)
    draw_link3(link3, xl3, yl3, zl3, link0Rot, link1Rot, link2Rot,link3Rot)
    draw_link4(link4, xl4, yl4, zl4, link0Rot, link1Rot, link2Rot,link3Rot,link4Rot)

    time.sleep(0.01)

def draw_base(link):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glTranslatef(0,-3.4,0)
    visualization.draw(link)

def draw_link0(link, x, y, z, link0Rot):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glRotatef(link0Rot, 0.0, 1.0 , 0.0)
    glTranslatef(x, y, z)
#    glRotatef(-25.0, 1.0, 0.0, 0.0)
#    glRotatef(45.0, 0.0, 0.0, 1.0)

    visualization.draw(link)

def draw_link1(link, x, y, z,link0Rot, link1Rot):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    #glRotatef(180,0,1,0) #flips l1 around so it isnt bending backwards
    glRotatef(link0Rot, 0.0, 1.0 , 0.0)
    glRotatef(link1Rot, 1.0, 0.0 , 0.0)
    #glTranslated(x, y, z) # link 1 does not translate about workspace, only pivots on shoulder base
#    glRotatef(-25.0, 1.0, 0.0, 0.0)
#    glRotatef(45.0, 0.0, 0.0, 1.0)

    visualization.draw(link)
    
def draw_link2(link, x, y, z, link0Rot, link1Rot, link2Rot):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glTranslatef(x, y, z)
    #print("link0Rot: ", link0Rot, " Link1Rot: ", link1Rot, " Link2Rot: ", link2Rot)
    glRotatef(link0Rot, 0.0, 1.0 , 0.0)
    glRotatef(link1Rot, 1.0, 0.0 , 0.0)
    glRotatef(link2Rot, 1.0, 0.0, 0.0)
    #glRotatef(180,0,1,0) #flipped around to match l1

    #glTranslatef(-x, -y, -z)
    
#    glRotatef(45.0, 0.0, 0.0, 1.0)

    visualization.draw(link)
    
def draw_link3(link, x, y, z, link0Rot, link1Rot, link2Rot,rotation):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glTranslatef(x, y, z)
    glRotatef(link0Rot, 0.0, 1.0 , 0.0)
    glRotatef(link1Rot, 1.0, 0.0 , 0.0)
    glRotatef(link2Rot, 1.0, 0.0, 0.0)
    glRotatef(rotation,0.0,1.0,0.0)
    
#    glRotatef(45.0, 0.0, 0.0, 1.0)

    visualization.draw(link)

def draw_link4(link, x, y, z, link0Rot, link1Rot, link2Rot,rotation,link4Rot):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glTranslatef(x, y, z)
    glRotatef(link0Rot, 0.0, 1.0 , 0.0)
    glRotatef(link1Rot, 1.0, 0.0 , 0.0)
    glRotatef(link2Rot, 1.0, 0.0, 0.0)
    glRotatef(rotation,0.0,1.0,0.0)
    glRotatef(link4Rot,1.0,0.0,0.0)

    visualization.draw(link)


def update(dt):
    global rotation
    global i
 #  rotation += 10.0 * dt
    if keys[key.A]:
        rotation += 10
    if keys[key.D]:
        rotation -= 10

    if keys[key.S]:
        cameraZ -= 5
    i += 1

#    if rotation > 720.0:
#        rotation = 0.0


pyglet.clock.schedule(update)
pyglet.app.run()

