import numpy as np
from math import *


lis = np.random.randint(100,500,2)
print(lis)
lis = list(lis)
# lis.append(700)
# lis.append(800)
# lis.append(900)
lis = np.array(lis)

remove = np.where(lis < 100)
print(remove)
newlis = np.delete(lis, remove)
# sortedlis = np.sort(lis)

# print(sortedlis)
# # remove treshold
# index = np.where(sortedlis <100)
# print(index)
# newlis = sortedlis[np.max(index)+1:]
print(newlis)
avg = np.mean(newlis)
std = np.std(newlis)
thr = 0.5
outlier = []

for x in newlis:
    z_score = (x-avg)/std
    if(abs(z_score)>thr):
        newlis = np.delete(newlis, np.where(newlis==x))
        outlier.append(x)
print("avg: ", avg)
print("std: ", std)
print("outlier: ", outlier)

print(newlis)
# remove = np.where()

# print(remove)

# tp = [("a",int),("b",int),("c",int)]
# data = [(1,2,3),(1,2,3)]
# data = np.array(data, dtype=tp)

# print(data)
# print(data[0])
# print(np.mean(data["a"]))
# print(data["a"][0])