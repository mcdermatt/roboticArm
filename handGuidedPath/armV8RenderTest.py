import ctypes
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import numpy

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
link0Rot = 15
link1Rot = 30
link2Rot = 80
link3Rot = 0
link4Rot = 45
rotation = 0.0 #count variable for spinning l3
lightfv = ctypes.c_float * 4

l1 = 6.5
l2 = 6.5
l3 = 2.65

link2RotEff = link1Rot + link2Rot

xElb = ( l1 * numpy.sin(link0Rot*(numpy.pi/180))*numpy.sin(link1Rot*(numpy.pi/180)))
yElb = ( l1 * numpy.cos((link1Rot*(numpy.pi/180)))) 
zElb =  ( l1 * numpy.cos(link0Rot*(numpy.pi/180))*numpy.sin(link1Rot*(numpy.pi/180)))


xl3 = xElb + ( l2 * numpy.sin(link0Rot*(numpy.pi/180))*numpy.sin(link2RotEff*(numpy.pi/180)))
yl3 = yElb + ( l2 * numpy.cos((link2RotEff*(numpy.pi/180)))) 
zl3 = zElb + ( l2 * numpy.cos(link0Rot*(numpy.pi/180))*numpy.sin(link2RotEff*(numpy.pi/180)))

xl4 = xElb + ( (l2+l3) * numpy.sin(link0Rot*(numpy.pi/180))*numpy.sin(link2RotEff*(numpy.pi/180)))
yl4 = yElb + ( (l2+l3) * numpy.cos((link2RotEff*(numpy.pi/180)))) 
zl4 = zElb + ( (l2+l3) * numpy.cos(link0Rot*(numpy.pi/180))*numpy.sin(link2RotEff*(numpy.pi/180)))


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
    glRotatef(rotation*0.0035,0,1,0)
    glMatrixMode(GL_MODELVIEW)

    #glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0*numpy.sin(rotation*0.1), 1.0, 0.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 0.9))
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightfv(0.0,0.0,0.0,0.1))

    draw_base(base)
    draw_link0(link0, 0, 0, 0, link0Rot)
    draw_link1(link1, 0, 0, 0,link0Rot, link1Rot)
    draw_link2(link2, xElb, yElb, zElb, link0Rot, link1Rot, link2Rot)
    draw_link3(link3, xl3, yl3, zl3, link0Rot, link1Rot, link2Rot,link3Rot)
    draw_link4(link4, xl4, yl4, zl4, link0Rot, link1Rot, link2Rot,link3Rot,link4Rot)

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
    rotation += 10.0 * dt

#    if rotation > 720.0:
#        rotation = 0.0


pyglet.clock.schedule(update)
pyglet.app.run()