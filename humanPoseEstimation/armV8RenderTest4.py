import ctypes
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import numpy
import time

#file plays back path recorded during handGuidedPath and DRAWS HUMAN GUIDING ROBOT

#init openGL stuff
#uncomment these two lines if using computer with graphics card
config = pyglet.gl.Config(sample_buffers=1, samples=9) #samples = number of points used for AA
window = pyglet.window.Window(width=1280, height=720, config = config)

#use this window setting if using laptop
# window = pyglet.window.Window(width=1280,height=720)

keys = key.KeyStateHandler()
window.push_handlers(keys)

base = Wavefront('base.obj')
link0 = Wavefront('l0.obj')
link1 = Wavefront('l1.obj')
link2 = Wavefront('l2andl3Dummy.obj')
torso = Wavefront('./human/torso.obj')
upperArm = Wavefront('./human/upperArm.obj')
lowerArm = Wavefront('./human/lowerArm.obj')
hand = Wavefront('./human/hand.obj')
greenCheck = pyglet.image.load('greenCheck.png')
gc = pyglet.sprite.Sprite(img=greenCheck)
gc.scale = 0.01
gc.x = -10
gc.y = 12
redX = pyglet.image.load('redX.png')
rx = pyglet.sprite.Sprite(img=redX)
rx.scale = 0.005
rx.x = -10
rx.y = 12

shoulderX = 0.17 * 39.37
shoulderY = 0.2 * 39.37
shoulderZ = 0.3 * 39.37 + 20
bodyRot = 180
bodyTilt = 0

l1 = 6.5
l2 = 6.5
l3 = 4.5

i = 0
path = 'armPath3.txt'
pathArr = numpy.genfromtxt(path,delimiter=" ")


link0Rot = (180/numpy.pi)*pathArr[i,0]
link1Rot = (180/numpy.pi)*pathArr[i,1]
link2Rot = (180/numpy.pi)*pathArr[i,2]
link3Rot = 0
link4Rot = 45

rotation = 0.0 #count variable for spinning
cameraZ = 0.0 #distance offset froms starting camera location
lightfv = ctypes.c_float * 4

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
    # gluLookAt(5,cameraZ,cameraZ,0,0,0,0,1,0)
    glRotatef(rotation*0.0035,0,1,0)
    glTranslatef(0,0,cameraZ)
    # glTranslatef(-cameraZ*numpy.sin(numpy.deg2rad(rotation)),0,-cameraZ*numpy.cos(numpy.deg2rad(rotation)))
    glMatrixMode(GL_MODELVIEW)
    link0Rot = (180/numpy.pi)*pathArr[i,0]
    link1Rot = (180/numpy.pi)*pathArr[i,1]
    link2Rot = (180/numpy.pi)*pathArr[i,2]
    link3Rot = 0
    link4Rot = 45

 #   rotation = 0.0 #count variable for spinning
    lightfv = ctypes.c_float * 4

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


    # glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1, 1.0, 0.0))
    # glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0*numpy.sin(rotation*0.1), 1.0, 0.0))
    # glLightfv(GL_LIGHT0, GL_AMBIENT, lightfv(0.5,0.5,0.5,0.1))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 0.9))
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightfv(0.0,0.0,0.0,0.1))

    glEnable(GL_DEPTH_TEST)

    #draw robot
    draw_base(base)
    draw_link0(link0, 0, 0, 0, link0Rot)
    draw_link1(link1, 0, 0, 0,link0Rot, link1Rot)
    draw_link2(link2, xElb, yElb, zElb, link0Rot, link1Rot, link2Rot)
    #draw human
    draw_torso(torso,shoulderX, shoulderY, shoulderZ, bodyRot, bodyTilt)
    draw_upperArm(upperArm,-5,0,0,0)
    draw_lowerArm(lowerArm,-10,0,0,0,0)
    draw_hand(hand,xl4,yl4,zl4,270)


    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE)
    draw_cube()
    
    #draw green check if EE is inside the workspace
    if (xl3 > -8) and (xl3 < 8) and (yl3 < 10) and (yl3 > -5) and (zl3 > 3) and (zl3 < 13):
        gc.draw()
    #if EE is outside workspace draw the red x
    else:
        rx.draw()

    time.sleep(0.01)

def draw_base(link):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glRotatef(45,0,1,0)
    glTranslatef(0,-3.4,0)
    visualization.draw(link)

def draw_cube():
    glLoadIdentity()
    # glDisable(GL_POLYGON_SMOOTH)
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
    glLineStipple(10, 0xAAAA)
    glEnable(GL_LINE_STIPPLE)
    glColor3f(0.8, 0.0, 0.1)
    glLineWidth(1)
    #robot right
    glBegin(GL_QUADS)
    glVertex3f( -8, -5,  13 )
    glVertex3f( -8,  10,  13 )
    glVertex3f( -8,  10,  3 )
    glVertex3f( -8, -5,  3 )
    glEnd()
    #robot left
    glBegin(GL_QUADS)
    glVertex3f( 8, -5,  13 )
    glVertex3f( 8,  10,  13 )
    glVertex3f( 8,  10,  3 )
    glVertex3f( 8, -5,  3 )
    glEnd()
    #robot top
    glBegin(GL_QUADS)
    glVertex3f( -8,  10,  13 )
    glVertex3f( -8,  10,  3 )
    glVertex3f( 8,  10,  3 )
    glVertex3f( 8,  10,  13 )
    glEnd()
    #robot bottom
    glBegin(GL_QUADS)
    glVertex3f( -8,  -5,  13 )
    glVertex3f( -8,  -5,  3 )
    glVertex3f( 8,  -5,  3 )
    glVertex3f( 8,  -5,  13 )
    glEnd()

    #returns polygon mode to smooth
    # glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
    glPolygonMode(GL_FRONT, GL_FILL)
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
    # glEnable(GL_BLEND)
    glEnable(GL_MULTISAMPLE)
    # glfwWindowHint(GLFW_SAMPLES, 4)
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
    # glDisable(GL_DEPTH_TEST)

def draw_torso(link,x,y,z,bodyRot, bodyTilt):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glScalef(0.8,0.8,0.8)
    glTranslatef(x,y,z)
    glRotatef(bodyRot,0,1,0)
    glRotatef(bodyTilt,1,0,0)

    visualization.draw(link)

def draw_upperArm(link,x,y,z,thetaArm0):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glScalef(0.8,0.8,0.8)
    glRotatef(thetaArm0,0,1,0)
    glTranslatef(x,y,z)
    visualization.draw(link)

def draw_lowerArm(link,x,y,z,thetaArm0,thetaArm1):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glScalef(0.8,0.8,0.8)
    glRotatef(thetaArm0,0,1,0)
    glRotatef(thetaArm1,0,1,0)
    glTranslatef(x,y,z)
    visualization.draw(link)

def draw_hand(link,x,y,z,thetaHand):
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glTranslatef(x,y,z)
    glRotatef(thetaHand,1,0,0)
    glRotatef(180,0,0,1)
    glScalef(0.8,0.8,0.8)
    
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
    global cameraZ
 #  rotation += 10.0 * dt
    if keys[key.A]:
        rotation -= 10
    if keys[key.D]:
        rotation += 10
    if keys[key.S]:
        cameraZ -= 0.1
    if keys[key.W]:
        cameraZ += 0.1
    i += 1

#    if rotation > 720.0:
#        rotation = 0.0


pyglet.clock.schedule(update)
pyglet.app.run()