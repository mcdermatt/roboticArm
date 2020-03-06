import numpy as np

#test to make sure I am indexing my state estimation lookup 

res = 6
index = 215

j0 = np.linspace(1,6,res)
j1 = np.linspace(1,6,res)
j2 = np.linspace(1,6,res)

count = 0
arr = np.zeros((res**3,3))

for i in j0:
	for j in j1:
		for k in j2:
			arr[count,0] = i
			arr[count,1] = j
			arr[count,2] = k
			count += 1			


print("index= ", index)
print(arr[index,:])