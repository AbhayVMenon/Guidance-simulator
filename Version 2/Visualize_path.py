import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from Simulator import *

p_traj, t_traj = run_trial()

p_x, p_y, p_z, p_vx, p_vy, p_vz = p_traj.T
t_x, t_y, t_z, t_vx, t_vy, t_vz = t_traj.T


fig = plt.figure()
ax = fig.add_subplot(projection="3d")

# plot full trajectories as faint trails
ax.plot(t_x, t_y, t_z, color="red", label = "target")
ax.plot(p_x, p_y, p_z, color="blue", label = "pursuer")

# # animated points
# target_point, = ax.plot([], [], [], 'o', color="red", markersize=4, label="Target")
# pursuer_point, = ax.plot([], [], [], 'o', color="blue", markersize=4, label="Pursuer")

# target_label = ax.text(t_x[0], t_y[0], t_z[0], "Target", color="red")
# pursuer_label = ax.text(p_x[0], p_y[0], p_z[0], "Pursuer", color="blue")

# # animated trails that grow over time
# target_trail, = ax.plot([], [], [], color="red", linewidth=1.5)
# pursuer_trail, = ax.plot([], [], [], color="blue", linewidth=1.5)

plt.xlabel("x")
plt.ylabel("y")

ax.grid(False)

plt.legend()
plt.show()

