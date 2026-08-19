import numpy as np, os, matplotlib.pyplot as plt

vec1 = np.zeros(10)
vec2 = np.ones(10)
vec3 = np.zeros(10)

i = 0
for i in range(len(vec1)-1): 
    vec1[i+1] += vec1[i] + 1

for i in range(len(vec2)-1):
    vec2[i+1] -= -vec2[i] - 1

for i in range(len(vec3)-1):
    vec3[i+1] = 1

vec = np.array([vec1, vec2, vec3])

#print(vec.T)
print(len(vec1))

