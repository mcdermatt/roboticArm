from sympy import symbols, solveset, linsolve, nonlinsolve
import sympy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


def getRotAng(X,Z):
	''' determines most likely rotated ellipse based on cartesian projections.
			BE CAREFUL THERE ARE 2 VALID SOLNS '''

	#Eqn of ellipse = ((x-h)**2)/a**2 + ((y-k)**2)/b**2 = 1 
	
	#loop through different angles and find volume of each ellipse
	#	constrained by: center at 0,0 and two points 
	#	as func of cross and angle (specified in param sweep)

	theta = np.linspace(0,np.pi,20)
	bestArea= 0
	bestAng = 0
	bestMajor = 0
	bestMinor = 0
	i = 0 #count var
	for ang in theta:

		x, z, a, b = symbols('x z a b', real=True)

		#eqn of ellipse
		expr = (x**2 - 0)/(a**2) + (z**2 - 0)/(b**2) - 1

		#x and z coords of cross rotated by theta
		xp = np.array([X*np.cos(theta),X*np.sin(theta)]) #x'
		zp = np.array([-Z*np.sin(theta),Z*np.cos(theta)]) #z'

		#plug in values of first point x'
		expr1 = expr.subs({x: xp[0,i], z: xp[1,i]})
		# print(expr1)
		#plug in values of second point z'
		expr2 = expr.subs({x: zp[0,i], z: zp[1,i]})
		# print(expr2)

		#solve system of equations
		#TODO - stop this from returning complex numbers
		coeff = nonlinsolve([expr1,expr2],[a,b])
		# print(coeff)
		# print(coeff.args[0]) #shows values of a and b for each angle

		#get area of ellipse represented by above equation
		major = coeff.args[0][0]
		minor = coeff.args[0][1]

		area = np.pi * major * minor

		#whatever angle produes ell of largest volume will be the answer (or pi - that since there will always be 2 solutions?)
		
		try:
			if area > bestArea:
				bestArea = area
				bestAng = ang
				bestMinor = minor
				bestMajor = major
		except:
			pass
		i += 1

	return(bestAng,bestMajor,bestMinor)

def drawCross(x,z,Ix,Iz):
	'''x and z projections of inertia'''

	sf = 1 #scaling factor

	#horizontal line
	ax.plot([x+Ix*sf,x-Ix*sf],[z,z],'b-')

	#vertical line
	ax.plot([x,x],[z+Iz*sf,z-Iz*sf],'b-')

	return

def drawEllipse(bestAng,bestMajor,bestMinor):

	ellipse = Ellipse([0,0],bestMajor,bestMinor,angle = np.rad2deg(bestAng))
	patches.append(ellipse)
	ax.add_patch(ellipse)
	return

if __name__ == "__main__":

	Ix = 0.3
	Iz = 0.1

	patches = []

	bestAng, bestMajor, bestMinor = getRotAng(Ix,Iz) #iputs are half lengths of cross line segments
	print('angle of rotation = ', bestAng)
	print('major length = ', bestMajor)
	print('minor length = ', bestMinor)

	#mpl display
	fig = plt.figure()
	ax = fig.add_subplot(xlim=(-0.5,0.5),ylim=(-0.5,0.5))

	drawCross(0,0,Ix/2,Iz/2)

	drawEllipse(bestAng,bestMajor,bestMinor)


	plt.draw()
	plt.pause(5)

