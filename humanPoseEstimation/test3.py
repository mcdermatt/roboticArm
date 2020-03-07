from getForcesFromPathData import getForcesFromPathData as getForces

traj = 'armPath.npy'


forces = getForces(traj)

print(forces)