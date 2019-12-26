import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import bezier

#to do
#make graphical output optional
#dynamically check linkage position for collision detection- make changes to obstacle
	
def rrtpath(xstart=1,ystart=1,zstart=1,xend=15,yend=15,zend=15,fidelity=300,obstacle=[]):
	fig = plt.figure()
	ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)
	ax.grid(False)

	plt.xlabel("x",fontdict=None,labelpad=None)
	plt.ylabel("y",fontdict=None,labelpad=None)
	#plt.zlabel("z",fontdict=None,labelpad=None)
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	plt.draw()
	plt.pause(0.5)
	plt.cla()
	ax.grid(False)
	ax.set_xlim(0,20)
	ax.set_ylim(0,20)
	ax.set_zlim(0,20)

	startpt = plt.plot([xstart],[ystart],[zstart], 'o', lw = 6, color = [1,0,0])
	endpt = plt.plot([xend],[yend],[zend], 'o', lw = 6, color = [1,0,0])
	plt.draw()



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

		#check if point is in obstacle
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

		if obstacle[int(numpy.floor(nodex[n])),int(numpy.floor(nodey[n])),int(numpy.floor(nodez[n]))] != 1:
			#line = ax.plot(xs,ys,zs, '-', lw = 2, color = [ 0.2, 0.5, 0.5])
			pass
		else:
			nodex[n] = nodex[int(nodeParent[n])]
			nodey[n] = nodey[int(nodeParent[n])]
			nodez[n] = nodez[int(nodeParent[n])]




		n = n + 1

	#trace back path closest to xyz goal

	#find node closest to goal point
	node = 1
	bestDist = 1000
	while node < runLen:
		nodeDist = numpy.sqrt((nodex[node]-xend)**2 + (nodey[node]-yend)**2 + (nodez[node]-zend)**2)
		if nodeDist < bestDist:
			bestDist = nodeDist
			closestToGoal = node
		node = node + 1

	print('node closet to goal is: ', closestToGoal)

	#start recording best chain of nodes from goal to start pos
	solnChain = []
	xsoln = []
	ysoln = []
	zsoln = []
	solnChain.append(closestToGoal)
	xsoln.append(nodex[closestToGoal])
	ysoln.append(nodey[closestToGoal])
	zsoln.append(nodez[closestToGoal])
	child = closestToGoal
	parent = int(nodeParent[child])

	while nodeParent[child] != 0:
		xseg = [nodex[child],nodex[parent]]
		yseg = [nodey[child],nodey[parent]]
		zseg = [nodez[child],nodez[parent]]
		#solnline = ax.plot(xseg,yseg,zseg, '-', lw = 3, color = [ 0.8, 0.5, 0.5])
		solnChain.append(parent)

		xsoln.append(nodex[parent])
		ysoln.append(nodey[parent])
		zsoln.append(nodez[parent])

		child = parent
		parent = int(nodeParent[parent])

	#one more time to finish path
	xseg = [nodex[child],nodex[parent]]
	yseg = [nodey[child],nodey[parent]]
	zseg = [nodez[child],nodez[parent]]
	#solnline = ax.plot(xseg,yseg,zseg, '-', lw = 3, color = [ 0.8, 0.5, 0.5])
	solnChain.append(parent)

	xsoln.append(nodex[parent])
	ysoln.append(nodey[parent])
	zsoln.append(nodez[parent])
	
	#use solution arrays to generate spline
	nodes = numpy.array([xsoln,ysoln,zsoln])
	curve = bezier.Curve(nodes, degree=1)
	#interpolate points from spline
	t_fine = numpy.linspace(0, 1, 60) # Curvilinear coordinate
	points_fine = curve.evaluate_multi(t_fine)
	#display spline on graph
	plt.plot(*points_fine, lw = 3, color = [1,0.1,0.1])


	#return (solnChain, xsoln, ysoln, zsoln) 
	#plt.pause(10)
	return(points_fine,ax)

#def isInside()

def plot_cube(cube_definition,ax):
    cube_definition_array = [
        numpy.array(list(item))
        for item in cube_definition
    ]

    points = []
    points += cube_definition_array
    vectors = [
        cube_definition_array[1] - cube_definition_array[0],
        cube_definition_array[2] - cube_definition_array[0],
        cube_definition_array[3] - cube_definition_array[0]
    ]

    points += [cube_definition_array[0] + vectors[0] + vectors[1]]
    points += [cube_definition_array[0] + vectors[0] + vectors[2]]
    points += [cube_definition_array[0] + vectors[1] + vectors[2]]
    points += [cube_definition_array[0] + vectors[0] + vectors[1] + vectors[2]]

    points = numpy.array(points)

    edges = [
        [points[0], points[3], points[5], points[1]],
        [points[1], points[5], points[7], points[4]],
        [points[4], points[2], points[6], points[7]],
        [points[2], points[6], points[3], points[0]],
        [points[0], points[2], points[4], points[1]],
        [points[3], points[6], points[7], points[5]]
    ]

    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection='3d')

    faces = Poly3DCollection(edges, linewidths=1, edgecolors='k')
    faces.set_facecolor((0,0,1,0.1))

    ax.add_collection3d(faces)

    # Plot the points themselves to force the scaling of the axes
    ax.scatter(points[:,0], points[:,1], points[:,2], s=0)

    plt.draw()
    plt.pause(0.5)
    #plt.cla()

    #ax.set_aspect('equal')



