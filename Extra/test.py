import numpy as np, matplotlib.pyplot as plt, os

os.system("cls")

total_pts = 10000
pts_in_circle = 0

x = np.random.uniform(0, 1, total_pts)
y = np.random.uniform(0, 1, total_pts)

pts_in_circle = np.sum(x**2 + y**2 <= 1)

quart_pi = pts_in_circle / total_pts

print(quart_pi * 4)

