from __future__ import print_function
import odrive
from odrive.enums import*
import time
import math
import numpy as np
import keyboard
import ctypes
from pyglet.gl import *
from pyglet.window import key
from pywavefront import visualization, Wavefront
import statePrediction as sp

# base = Wavefront('../handGuidedPath/base.obj')
# link0 = Wavefront('../handGuidedPath/l0.obj')
# link1 = Wavefront('../handGuidedPath/l1.obj')
# link2 = Wavefront('../handGuidedPath/l2.obj')
# link3 = Wavefront('../handGuidedPath/l3.obj')

#link lengths (in)
l1 = 6.5
l2 = 6.5
l3 = 2.65

predictionTable = np.genfromtxt('../multibodySim/predictionTable888888.txt', delimiter=',') #usecols=range(__)
print(predictionTable)