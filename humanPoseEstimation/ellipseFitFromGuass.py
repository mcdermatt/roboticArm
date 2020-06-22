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


def confidence_ellipse(x, y, ax, n_std=3.0, facecolor='none', **kwargs):
    """
    Create a plot of the covariance confidence ellipse of *x* and *y*.

    Parameters
    ----------
    x, y : array-like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : float
        The number of standard deviations to determine the ellipse's radiuses.

    **kwargs
        Forwarded to `~matplotlib.patches.Ellipse`

    Returns
    -------
    matplotlib.patches.Ellipse
    """
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensionl dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor=facecolor, **kwargs)

    # Calculating the stdandard deviation of x from
    # the squareroot of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    mean_x = np.mean(x)

    # calculating the stdandard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std
    mean_y = np.mean(y)

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)

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