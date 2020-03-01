import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#be careful- z and y axis switched between SM and MPL

fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=False)
ax.grid(False)

plt.xlabel("x",fontdict=None,labelpad=None)
plt.ylabel("y",fontdict=None,labelpad=None)
#plt.zlabel("z",fontdict=None,labelpad=None)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.grid(False)

#draw ellipsoid
phi = np.linspace(0,2*np.pi, 256).reshape(256, 1) # the angle of the projection in the xy-plane
theta = np.linspace(0, np.pi, 256).reshape(-1, 256) # the angle from the polar axis, ie the polar angle
Ax = 0.5
Ay = 0.1
Az = 0.6

x = Ax*np.sin(theta)*np.cos(phi)
y = Ay*np.sin(theta)*np.sin(phi)
z = Az*np.cos(theta)

ax.plot_surface(x, y, z, color='b')
ax.set_xlim(-1,1)
ax.set_ylim(-1,1)
ax.set_zlim(-1,1)

plt.draw()
plt.pause(5)
plt.cla()