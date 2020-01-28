import ctypes
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import numpy
#from euclid import *
#import pyrr


#game plan:
#   1: convert x,y,z to spherical coordinates
#   2: find effective radius of sphere and use that value to determine angle of elbow (t2)
#   3: phi is angle from z axis 
#   4: theta is rotation about x axis + angle to compensate for elbow

rotation = 0.0
l0=0.25 #shoulder offset
l1 = 9.5 #upper arm
l2 = l1 #forearm

def cartesian_to_spherical(x,y,z):
    
    # r = numpy.sqrt((x*x)+(y*y)+(z*z))
    # phi = numpy.arctan2((numpy.sqrt((x  * x) + (z * z))), y )
    # theta = numpy.arctan2(z, x)
    # return (r,phi,theta)
    r = numpy.sqrt((x*x)+(y*y)+(z*z))
    phi = numpy.arctan2((numpy.sqrt((x  * x) + (y * y))), z )
    theta = numpy.arctan2(y, x)
    return (r,phi,theta)

def get_joint_angles(x,y,z):
    
    (r,phi,theta) = cartesian_to_spherical(x,y,z)
    
    l0=0.25 #shoulder offset
    l1 = 9.5 #upper arm
    l2 = l1 #forearm

    # #elbow
    # a2 = numpy.arcsin(r/(2*l1))
    
    # #shoulder side to side
    # a0 = theta
    
    # #shoulder up down
    # a1 = phi - (numpy.pi-(a2 / 2))
    #elbow
    a2 = numpy.arccos(((l1*l1)+(l2*l2)-(r*r))/(-2*l1*l2))
    #shoulder side to side
    a0 = theta
    #shoulder up down
    a1 = numpy.pi + phi + numpy.arccos(((l1*l1)-(l2*l2)+(r*r))/(-2*l1*r))
    
    return(a0,a1,a2)

def get_elbow_pos(x,y,z):
    
    # xElbow = ( l1 * numpy.cos(a0)*numpy.sin(a1))
    # yElbow = ( l1 * numpy.sin(a0)*numpy.sin(a1))
    # zElbow = ( l1 * numpy.cos(a0))
    
    # return (xElbow, yElbow, zElbow)
    l0 = 0.25 #shoulder offset
    l1 = 9.5 #upper arm
    l2 = l1 #forearm

    (r, phi, theta) = cartesian_to_spherical(x,y,z)
    (a0,a1,a2) = get_joint_angles(x,y,z)
    
    xElbow = ( l1 * numpy.cos(a0)*numpy.sin(a1))
    yElbow = ( l1 * numpy.sin(a0)*numpy.sin(a1))
    zElbow = ( l1 * numpy.cos(a1))
    
    return (xElbow, yElbow, zElbow)

def get_l3_pos(x,y,z):
    
    a0,a1,a2 = get_joint_angles(x,y,z)
    xElbow,yElbow,zElbow = get_elbow_pos(a0,a1,a2)
    
    if xElbow > 0:
        xl3 = (xElbow + (7 * numpy.cos(a1)*numpy.sin(a0)))
    else:
        xl3 = (xElbow + (7 * numpy.cos(a1)*numpy.sin(a0)))
        
    yl3 = (zElbow + (7 * numpy.cos(a1)))
    
    if zElbow > 0:
        zl3 = (yElbow + (7 * numpy.sin(a1)*numpy.sin(a0)))
    else: 
        zl3 = (yElbow + (7 * numpy.sin(a1)*numpy.sin(a0)))

    
    return (xl3,yl3,zl3)  
   
    # r,phi,theta = cartesian_to_spherical(x,y,z)
    # a0,a1,a2 = get_joint_angles(x,y,z)
    # xElbow,yElbow,zElbow = get_elbow_pos(a0,a1,a2)
        
    # xl3 = r*numpy.cos(theta)*numpy.sin(phi)     
    # yl3 = r*numpy.sin(theta)*numpy.sin(phi) 
    # zl3 = r*numpy.cos(phi)
    
    # return (xl3,yl3,zl3)
    
    

#init openGL stuff
window = pyglet.window.Window(width=1280, height=720)
keys = key.KeyStateHandler()
window.push_handlers(keys)

#ENTER DESIRED POSITION HERE----------------------------------------------
xgoal = 5
ygoal = 10 #+ 2*numpy.random.rand(1)
zgoal = 5 #+ 5*numpy.random.rand(1)

goals = [xgoal,ygoal,zgoal]
print(goals)

(A0,A1,A2) = get_joint_angles(xgoal,ygoal,zgoal)
(xElb,yElb,zElb) = get_elbow_pos(xgoal,ygoal,zgoal)
(xl3,yl3,zl3) = get_l3_pos(xgoal,ygoal,zgoal)

print("xElb func: ", xElb,yElb,zElb)
print("actual elbow: ", (9.5*numpy.cos(A0)*numpy.sin(A1)))


link0 = Wavefront('link0.obj')
link0Rot = numpy.rad2deg(A0)

link1 = Wavefront('link1.obj')
link1Rot = -1*numpy.rad2deg(A1)

link2 = Wavefront('link2.obj')
link2Rot = numpy.rad2deg(A2)

link3 = Wavefront('link3.obj')
link3Rot = 90

#box3 = Wavefront('data/box/box-N3F_V3F.obj')
#box4 = Wavefront('data/box/box-T2F_V3F.obj')
#box5 = Wavefront('data/box/box-T2F_C3F_V3F.obj')
#box6 = Wavefront('data/box/box-T2F_N3F_V3F.obj')

#rotation = 0.0
lightfv = ctypes.c_float * 4


@window.event
def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(50., float(width)/height, 1., 100.) #change first argument for fov
    glTranslatef(0,-5,-30) #fits arm into camera view
    glMatrixMode(GL_MODELVIEW)
    return True

@window.event
def on_draw():
    window.clear()
    glViewport(0,0,1280,720)
    glLoadIdentity()

    glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0*numpy.sin(rotation*0.1), 1.0, 0.0))

    draw_link0(link0, 0, 0, 0, link0Rot)
    draw_link1(link1, 0, 0, 0,link0Rot, link1Rot)
    draw_link2(link2, xElb, yElb, zElb, link0Rot, link1Rot, link2Rot)
    draw_link3(link3, xl3, yl3, zl3, link0Rot, link1Rot, link2Rot,rotation)

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
    glRotatef(180,0,1,0) #flips l1 around so it isnt bending backwards
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
    glRotatef(link1Rot, -1.0, 0.0 , 0.0)
    glRotatef(link2Rot, 1.0, 0.0, 0.0)
    glRotatef(180,0,1,0) #flipped around to match l1

    #glTranslatef(-x, -y, -z)
    
#    glRotatef(45.0, 0.0, 0.0, 1.0)

    visualization.draw(link)
    
def draw_link3(link, x, y, z, link0Rot, link1Rot, link2Rot,rotation):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glTranslatef(x, y, z)
    glRotatef(link0Rot, 0.0, 1.0 , 0.0)
    glRotatef(link1Rot, -1.0, 0.0 , 0.0)
    glRotatef(link2Rot + 90, 1.0, 0.0, 0.0)
    glRotatef(rotation,0.0,0.0,1.0)
    
#    glRotatef(45.0, 0.0, 0.0, 1.0)

    visualization.draw(link)


def update(dt):
    global rotation
    rotation += 10.0 * dt

#    if rotation > 720.0:
#        rotation = 0.0


pyglet.clock.schedule(update)
pyglet.app.run()