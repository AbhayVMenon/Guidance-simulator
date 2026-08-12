from InitialConditions import *
from GuidanceLaws import State
import numpy as np

chosen_trajectory = traj(x, x_start, dt, target_velocity_mag)

def target_states_values(dt, x_range, x_start, target_velocity_mag, chosen_trajectory):

    x = np.arange(x_start, x_range + dt * target_velocity_mag, dt * target_velocity_mag)

    y, z, vx, vy, vz = chosen_trajectory(x, x_start)

    return x, y, z, vx, vy, vz

t_traj = target_states_values(dt, x_range, x_start, target_velocity_mag, traj) #outputs position and velocity of target


chosen_law = law(N, dt)

def ac_into_velocity_and_position(p_state, dt, a_c):

    p_current_r = p_state.r

    p_current_v = p_state.v
    
    p_velocity_new_raw = p_current_v + a_c * dt

    unit_p_velocity_new = p_velocity_new_raw / np.linalg.norm(p_velocity_new_raw)

    p_velocity_new = np.linalg.norm(p_current_v) * unit_p_velocity_new

    p_r_new = p_current_r + p_velocity_new * dt

    return State(p_r_new, p_velocity_new)