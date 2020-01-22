from __future__ import print_function	
# import odrive
# from odrive.enums import*
import time
import math
import numpy as np
import keyboard

arr = [[1, 2, 3]]

count = 0
while count < 100:
	# print(arr)
	new = [[np.random.rand(1),0,0]]
	arr = np.append(arr,new,axis=0)
	count += 1
np.savetxt('armPath.txt',arr)
loadedArray = np.loadtxt('armPath.txt',dtype=float)
	
# loadedArray = np.append(loadedArray,[[1,1,1]],axis=0)
# np.savetxt('armPath.txt',loadedArray)
count2 = 0
while count2 < count:
	currentStep = loadedArray[count2,:]
	print(currentStep)
	time.sleep(0.01)
	try:
		if keyboard.is_pressed('q'):
			print('input detected')
			break
	except:
		break
	count2 += 1
