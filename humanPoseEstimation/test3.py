from getForcesFromPathData import getForcesFromPathData as getForces
import numpy as np

traj = np.load('armPath.npy')


forces = getForces(traj)

print(forces)