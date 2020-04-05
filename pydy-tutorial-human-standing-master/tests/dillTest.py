import dill
import pickle
from returnFuncTest import returnFunction as rf
import numpy as np

# z = rf(1) #z is function

# ser = dill.dumps(z) #ser is bytes
# # print(ser)

# test_file = open("testFunc.txt","w")
# test_file.write(str(ser))
# test_file.close()
# #need to be able to save ser to some type of persistant file

# # ser = dill.dump(z,"testFunc.txt")

# f = open("testFunc.txt", "r")
# f = str(f.read())
# print(f)
# ser2 = f.encode('utf-8')
# # ser2 = bytes(f, "utf-8")

# z2 = dill.loads(ser)

# print(z2(10))

z = rf(0)

file = "testFunc.txt"
dill.dump(z, open(file, 'wb'))


fun = dill.load(open(file, 'rb'))
print(fun)

print(fun(10))