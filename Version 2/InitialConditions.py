import numpy as np, os
from GuidanceLaws import *
from Trajectories import *

#T = target, P = Pursuer

#simulation physics
dt = 0.01 # time step
g = 9.81
x_start = 1000
x_range = 10000

#target setup
target_velocity_mag = 1000
t0 = np.array([x_start, 0, 0, target_velocity_mag, 0, 0])
traj = Helix

#pursuer setup
speed_multipier = 1.7 # How much faster P is than T
N = 3
max_gs = 40
kill_zone_size = 25 #effective size of missile
p0 = np.array([0, 500, 0, (speed_multipier * target_velocity_mag), 0, 0]) #[x0, y0, z0, xdot0, ydot0, zdot0]
e = 0 #error in which seeker will read target trajectory

#Guidance law you want Pursuer to follow
law_params = {"N": N, "dt": dt, "max_gs": max_gs} #edit only if a guidance law is being added!
law_name = "APN"
