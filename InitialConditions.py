import numpy as np, os
from GuidanceLaws import *
from Trajectories import *

#T = target, P = Pursuer

dt = 0.01 # time step
speed_multipier = 1.7 # How much faster P is than T
target_velocity_mag = 1000

#Pursuer inital state vector
p0 = np.array([0, 500, 0, (speed_multipier * target_velocity_mag), 0, 0]) #[x0, y0, z0, xdot0, ydot0, zdot0]

# np.random.randint(-1000,1000)
x_start = 1000
x_range = 10000

#global constants
g = 9.81
N = 3

#chosen algorithm
traj = Helix
law = AugPropNav
