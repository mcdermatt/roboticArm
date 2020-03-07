from statePredictionWithInterpolation import statePrediction as spi
import numpy as np
import time

runlen = 100

t1 = time.time()
count = 0
while count < runlen:
	ans = spi(10*np.random.rand(),10*np.random.rand(),10*np.random.rand(),10*np.random.rand(),10*np.random.rand(),10*np.random.rand())
	count += 1

t2 = time.time()

print(f"Time took to run: {t2-t1} seconds.")


#takes ~1.5s for 100 runs ---> 66 Hz