import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep

def rrtpath(xstart=1,ystart=1,zstart=1,xend=10,yend=10,zend=10,fidelity=300,barrier=0):
	fig = plt.figure()
	ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)

	plt.xlabel("x",fontdict=None,labelpad=None)
	plt.ylabel("y",fontdict=None,labelpad=None)
	#plt.zlabel("z",fontdict=None,labelpad=None)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	plt.draw()
	plt.pause(0.5)
	plt.cla()
	ax.set_xlim(0,20)
	ax.set_ylim(0,20)
	ax.set_zlim(0,20)

	xMax = 20
	yMax = 20
	zMax = 20
	stepdist = 1
	runLen = fidelity
	nodex = numpy.zeros(runLen)
	nodey = numpy.zeros(runLen)
	nodez = numpy.zeros(runLen)
	nodeCost = numpy.zeros(runLen)
	nodeParent = numpy.zeros(runLen)
	xstep = numpy.zeros(runLen)
	ystep = numpy.zeros(runLen)
	zstep = numpy.zeros(runLen)
	nodex[0] = xstart
	nodey[0] = ystart
	nodez[0] = zstart

	#debug
	nodeParent[2] = 1
	nodeParent[1] = 0
	bestNode = 0

	n = 1

	while n < runLen:
		print('n = ',n)
		pt = [float(numpy.floor(xMax*numpy.random.rand(1))), float(numpy.floor(yMax*numpy.random.rand(1))), float(numpy.floor(zMax*numpy.random.rand(1)))]
		print('randomly chosen point: ', pt)
		mindist = 1000
		#finds closest existing node to new random point
		nodecheck = 0
		while nodecheck < n:
			dist = numpy.sqrt((pt[0]-nodex[nodecheck])**2 + (pt[1]-nodey[nodecheck])**2 + (pt[2]-nodez[nodecheck])**2)
			if dist < mindist:
				mindist = dist
				bestNode = nodecheck
				nodeCost[n] = mindist
			nodecheck = nodecheck + 1
		
		nodeParent[n] = bestNode
		#c = n
		#p = nodeParent[c]
		#while (p != 0):
		#	nodeCost[c] = nodeCost[c] + numpy.sqrt((nodex[c]-nodex[p])**2 + (nodey[c]-nodey[p])**2 + (nodez[c]-nodez[p])**2)
		#	c = p
		#	p = nodeParent[c]


		xstep = numpy.linspace(nodex[bestNode],pt[0],10)
		ystep = numpy.linspace(nodey[bestNode],pt[1],10)
		zstep = numpy.linspace(nodez[bestNode],pt[2],10) # for third entry: abs(numpy.ceil(nodez[bestNode]-pt[3]))+2

		nodex[n] = xstep[2]
		nodey[n] = ystep[2]
		nodez[n] = zstep[2]



		#plot line between node[n] and bestnode
		xs = [nodex[bestNode],nodex[n]]
		ys = [nodey[bestNode],nodey[n]]
		zs = [nodez[bestNode],nodez[n]]
		line = ax.plot(xs,ys,zs, '-', lw = 2, color = [ 0.2, 0.5, 0.5])



		n = n + 1

	#trace back path closest to xyz goal

	return (nodex[1], nodey[1], nodez[1]) 
