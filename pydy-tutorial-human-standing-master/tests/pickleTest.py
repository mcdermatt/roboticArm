import pickle
from returnFuncTest import returnFunction, test2

z = returnFunction(0)
# z = test2(1)
print(z)
pickle_out = open("z.pickle","wb")
pickle.dump(z,pickle_out)

pickle_out.close()