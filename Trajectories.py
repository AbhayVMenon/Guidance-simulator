import numpy as np

def get_velocity(t_x, t_y, t_z, dt, target_velocity_mag):

    t_v_raw = np.array([np.gradient(t_x, dt), np.gradient(t_y, dt), np.gradient(t_z, dt)])

    unit_t_v = t_v_raw / np.linalg.norm(t_v_raw, axis = 0)

    (t_vx, t_vy, t_vz) = target_velocity_mag * unit_t_v

    return t_vx, t_vy, t_vz


def Helix(x, x_start, dt, target_velocity_mag):

    A = 500
    L = 3300

    t_y = A * np.sin(2 * np.pi / L * (x - x_start))
    t_z = A * np.cos(2 * np.pi / L * (x - x_start))

    t_vx, t_vy, t_vz = get_velocity(x, t_y, t_z, dt, target_velocity_mag)

    return t_y, t_z, t_vx, t_vy, t_vz

def Straight(x, x_start, dt, target_velocity_mag):
    t_y = np.zeros(len(x))
    t_z = np.zeros(len(x))
    t_vx, t_vy, t_vz = get_velocity(x, t_y, t_z, dt, target_velocity_mag)

    return t_y, t_z, t_vx, t_vy, t_vz