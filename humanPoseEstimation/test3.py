from getForcesFromPathData import getForcesFromPathData as getForces
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Draws 3d plot of focres exerted by human on robot EE
#points colored to demonstrate which cartsian projection of force is dominant
#XYZ = RGB

#Loads traj file from handGuidedPath and converts to deg
# traj = np.loadtxt('../handGuidedPath/armPath.txt')
# traj = traj  * 180 / np.pi
# np.save('armPath.npy',traj)

#get forces
traj = np.load('armPath.npy')
forces = getForces(traj)
# forces = np.loadtxt('forces.txt')
# print(forces)
#np.savetxt('forces.txt',forces)
#np.save('forces.npy',forces)

fig = plt.figure()
ax = fig.add_subplot(111, xlim=(-1,1), ylim=(-1,1), zlim=(0,1), projection='3d', autoscale_on=True)
ax.grid(False)
ax.set_xlabel('x')
ax.set_ylabel('z')
ax.set_zlabel('y')
count = 0
scale = 1/3
while count < forces.shape[1]:
	#set color of force as function of X Y and Z components
	color = [(forces[3,count])**scale,(forces[4,count])**scale,(forces[5,count])**scale]
	plt.plot([forces[0,count]],[forces[2,count]],[forces[1,count]],'o',color=color)
	count += 1

plt.draw()
plt.pause(30)

