from InitialConditions import *
from GuidanceLaws import *
import numpy as np

def run_trial(e=e, law_name=law_name, law_params = law_params): #runs the trial and outputs both the target and pursuer trajectories

    #create target trajectory
    def target_states_values(dt, x_range, x_start, target_velocity_mag, traj):

        x = np.arange(x_start, x_range + dt * target_velocity_mag, dt * target_velocity_mag)

        y, z, vx, vy, vz = traj(x, x_start, dt, target_velocity_mag)

        return x, y, z, vx, vy, vz 

    #clean target trajectory
    t_x, t_y, t_z, t_vx, t_vy, t_vz = target_states_values(dt, x_range, x_start, target_velocity_mag, traj) #outputs position and velocity of target

    #courrupted seeker estimate of target positon
    def introduce_error(clean_pos):

        corrupt_pos = np.zeros(len(t_x))

        for i in range(len(clean_pos)):
            error = np.random.uniform(-e, e)
            corrupt_pos[i] = clean_pos[i] + error

        return corrupt_pos

    t_x_corrupt = introduce_error(t_x)
    t_y_corrupt = introduce_error(t_y)
    t_z_corrupt = introduce_error(t_z)

    # clean_position = ([t_x, t_y, t_z]).T

    # corrupt_pos = np.zeros(len(clean_position))
    
    # for i in len(clean_position): 
    #     error = np.random.uniform(-e, e, size = 3)
    #     corrupt_pos[i] = clean_position[i] + error


    #initialising pursuer vector
    p_post = np.zeros(len(t_x)-1) 

    p_x = np.insert(p_post, 0, p0[0])   # position values
    p_y = np.insert(p_post, 0, p0[1])
    p_z = np.insert(p_post, 0, p0[2])

    p_vx = np.insert(p_post, 0, p0[3]) # velocity values
    p_vy = np.insert(p_post, 0, p0[4])
    p_vz = np.insert(p_post, 0, p0[5])


    #finding the law that user has chosen
    law_registry = {"PP": (PurePursuit, ["max_gs"]), 
                    "PN": (PropNav, ["N"]),
                    "APN":(AugPropNav, ["N", "dt"])}

    law_class, parameters = law_registry[law_name]
    law_kwargs = {k: law_params[k] for k in parameters}
    chosen_law = law_class(**law_kwargs)

    def ac_into_velocity_and_position(p_state, dt, a_c):

        p_current_r = p_state.r

        p_current_v = p_state.v

        p_velocity_new_raw = p_current_v + a_c * dt

        unit_p_velocity_new = p_velocity_new_raw / np.linalg.norm(p_velocity_new_raw)

        p_velocity_new = np.linalg.norm(p_current_v) * unit_p_velocity_new

        p_r_new = p_current_r + p_velocity_new * dt

        return State(p_r_new, p_velocity_new)

    intercept_index = None

    for k in range(0, len(t_x) - 1): #runs one entire trial

        #t_state = State(r=np.array([t_x[k], t_y[k], t_z[k]]), v=np.array([t_vx[k], t_vy[k], t_vz[k]]))
        t_state = State(r=np.array([t_x_corrupt[k], t_y_corrupt[k], t_z_corrupt[k]]), v=np.array([t_vx[k], t_vy[k], t_vz[k]])) #seeker taken in the corrupted target data

        p_state = State(r=np.array([p_x[k], p_y[k], p_z[k]]), v=np.array([p_vx[k], p_vy[k], p_vz[k]]))

        a_c = chosen_law.compute_command(p_state, t_state)

        new_p_state = ac_into_velocity_and_position(p_state, dt, a_c)

        p_x[k+1], p_y[k+1], p_z[k+1] =  new_p_state.r
        p_vx[k+1], p_vy[k+1], p_vz[k+1] = new_p_state.v

        #check for conditionals at k+1
        p_t_dist = np.sqrt((t_x[k+1] - p_x[k+1])**2 + (t_y[k+1] - p_y[k+1])**2 + (t_z[k+1] - p_z[k+1])**2)

        if p_t_dist < kill_zone_size: 
            intercept_index = k + 1
            break

    if intercept_index is None:
        intercept_index = len(p_x)
        print("Unsuccessful interception")
    else:
        intercept_time = intercept_index * dt
        print(f"Intercept occurs {intercept_time}s after launch")

    stop = intercept_index + 1

    def cut(array, stop):
        final_array = array[0:stop]
        return final_array

    p_traj = np.array([cut(p_x, stop), cut(p_y, stop), cut(p_z, stop), cut(p_vx, stop), cut(p_vy, stop), cut(p_vz, stop)]).T
    t_traj = np.array([cut(t_x, stop), cut(t_y, stop), cut(t_z, stop), cut(t_vx, stop), cut(t_vy, stop), cut(t_vz, stop)]).T

    return p_traj, t_traj