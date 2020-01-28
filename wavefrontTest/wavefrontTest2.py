import ctypes
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import numpy

#init openGL stuff
window = pyglet.window.Window(width=1280, height=720)
keys = key.KeyStateHandler()
window.push_handlers(keys)

link0 = Wavefront('link0.obj')
link0Rot = 90

link1 = Wavefront('link1.obj')
link1Rot = 10

link2 = Wavefront('link2.obj')
link2Rot = 45

link3 = Wavefront('link3.obj')
link3Rot = 90
rotation = 0.0 #count variable for spinning l3

lightfv = ctypes.c_float * 4

xElb = 0
yElb = 9.5
zElb = 0

xl3 = 0
yl3 = 15
zl3 = 0

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