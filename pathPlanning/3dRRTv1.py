import rrtpath
from time import sleep
import numpy
import matplotlib.pyplot as plt
import IKModel as ikm

#add arbitrary obstacles for testing
#set up obstacles - read in as 3d grid representing search space. if xyz cordinate is 1, it is obstacle
xMax = 20
yMax = 20
zMax = 20
cloudDensity = 2
i = xMax*cloudDensity
j = yMax*cloudDensity
k = zMax*cloudDensity
obstacle = numpy.zeros((i,j,k))
x1 = 7
y1 = 5
z1 = 3
while x1 < 13:
	while y1 < 13:
		while z1 < 18:
			obstacle[x1,y1,z1] = 1
			#display obstacle on graph
			#obsPoint = ax.plot([x1],[y1],[z1], '.', lw = 5, color = [ 0.7, 0.5, 0.5])
			#plt.draw()
			z1 = z1 + 1
		z1 = 3
		y1 = y1 + 1
	y1 = 5
	x1 = x1 + 1

x2 = 11
y2 = 0
z2 = 10
while x2 < 16:
	while y2 < 5:
		while z2 < 15:
			obstacle[x2,y2,z2] = 1
			#display obstacle on graph
			#obsPoint = ax.plot([x2],[y2],[z2], '.', lw = 5, color = [ 0.7, 0.5, 0.5])
			#plt.draw()
			z2 = z2 + 1
		z2 = 10
		y2 = y2 + 1
	y2 = 0
	x2 = x2 + 1

bestPath,ax = rrtpath.rrtpath(10,0,10,0,15,-5,fidelity=1000,obstacle=obstacle)

#draw prism surrounding point cloud rather than drawing points individually
cube_definition1 = [(7,5,3), (7,12,3), (12,5,3), (7,5,17)]
rrtpath.plot_cube(cube_definition1,ax)
cube_definition2 = [(11,0,10), (11,4,10), (15,0,10), (11,0,14)]
rrtpath.plot_cube(cube_definition2,ax)

print(bestPath)

ikm.IKModel(path=bestPath,ax=ax)

plt.pause(5)