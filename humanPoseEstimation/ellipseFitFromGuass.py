import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import numpy
from time import sleep
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from humanInertiaEstimator import inertiaEstimator
from ellipse import *
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms


if __name__ == '__main__':
	ie = inertiaEstimator()

	ie.x0[0] = np.deg2rad(60)
	ie.x0[1] = np.deg2rad(90)
	ie.x0[2] = np.deg2rad(30)

	fig = plt.figure()
	ax = fig.add_subplot(xlim=(-0.05,0.05),ylim=(-0.05,0.05))
	ax.set_xlabel('x')
	ax.set_ylabel('z')

	patches = []

	x,z = ie.predictXZGauss()

	data = np.array([x,z])
	cov = np.cov(data)
	print('cov = ', cov)

	ax.plot(x,z,'b.')

	lam1,lam2,theta = cov2Ell(cov)

	print('lam1 = ',lam1,'lam2 = ', lam2, 'theta =  ', theta)

	drawEllipse(ax,0,0,lam1,lam2,theta)

	# confidence_ellipse(x,z,ax,edgecolor='fuchsia', linestyle='--')
	# ell = confidence_ellipse(x,z,ax)
	# patches.append(ell)
	# ax.add_patch(ell)

	plt.pause(15)
	plt.draw()