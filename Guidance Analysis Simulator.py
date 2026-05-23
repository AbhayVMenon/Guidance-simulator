import numpy as np, os, matplotlib.pyplot as plt, math as m

os.system("cls")

#T = target, P = Pursuer

dt = 0.1 # time step

#target inital state vector 
t0 = np.array([1000, 0, 1000, 0]) # m
t1 = t0 + np.array([(dt * t0[2]), 0, 0, 0]) # @t = 0.01

#Pursuer inital state vector
p0 = np.array([0, 500, (1.1 * t0[2]), 0])
p1 = p0 + np.array([(dt * 1.1 * t0[2]), 0, 0, 0]) # @t = 0.01

# np.random.randint(-1000,1000)
x_range = 10000
x = np.arange(0,x_range,0.1)   #range of sim window
#global constants
g = 9.81
a = [0, -9.81] # m/s^2
N = 3
dt = 0.01


# Guidance Laws
def Pure_Pursuit_Guidance(ti2, pi2):

    p_ri = pi2[0:2]          #P inital position
    t_ri = ti2[0:2]          #T inital position

    los = t_ri - p_ri       #vector between P & T

    mag_los = np.linalg.norm(los)

    if mag_los < 3:         #size of missile
        unit_los = np.zeros(2)
    
    else:
        unit_los = los / mag_los

    return unit_los

def Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt):

    #LOS Rate
    t_ri1 = ti1[0:2]
    t_ri2 = ti2[0:2]  #T inital positions 

    p_ri1 = pi1[0:2]
    p_ri2 = pi2[0:2]  #P inital positions

    t_vi2 = ti2[2:4]
    p_vi2 = pi2[2:4]  #P inital velocities 

    los1 = t_ri1 - p_ri1
    los2 = t_ri2 - p_ri2
    unit_los2 = los2 / np.linalg.norm(los2)

    los_angle1 = np.arctan2(los1[1],los1[0])
    los_angle2 = np.arctan2(los2[1],los2[0])

    los_rate = (los_angle2 - los_angle1) / dt

    #closing rate 
    v_rel = p_vi2 - t_vi2

    Vc = np.dot(v_rel, unit_los2) # closing speed 

    perp_los2 = np.array(([0, -1], [1, 0])) @ los2

    unit_perp_los2 = perp_los2 / np.linalg.norm(perp_los2)

    a_c = N * Vc * los_rate * unit_perp_los2

    return a_c, unit_perp_los2

def Aug_Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt):

    a_c_PN, unit_perp_los2 = Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt)

    t_vi1 = ti1[2:4]
    t_vi2 = ti2[2:4]

    t_a = (t_vi2 - t_vi1)/dt

    t_a_perp = np.dot(unit_perp_los2, t_a)  #component perp to LOS

    a_c_Aug = a_c_PN + N/2 * t_a_perp


    return a_c_Aug

# Target Trajectories
def straight_T_trajectory(x, x_start):
    return np.zeros(len(x))

def sin_T_trajectory(x, x_start):
    c = x_start
    A = 5000
    w = 0.002
    return A * np.sin(w * (x - c))

# plot coordinates
def target_states_values(ti1, ti2, dt, x_range, trajectory):

    x_start = ti2[0]

    x_pre = np.arange(x_start, x_range + dt, dt)

    x = np.insert(x_pre, 0, ti1[0]) #prepending 1st state

    y_pre = trajectory(x_pre, x_start)

    y = np.insert(y_pre, 0, ti1[1])

    return x,y

def update_state_PP(Pure_Pursuit_Guidance, pi2, ti2, dt):

    p_current_r = pi2[0:2]    #current position of P
    t_current_r = ti2[0:2]    #current position of T

    p_current_v = pi2[2:4]    #current velocity of P

    p_v_mag = np.linalg.norm(p_current_v)

    new_direction = Pure_Pursuit_Guidance(t_current_r, p_current_r)

    p_velocity_new = p_v_mag * new_direction  # amount in x & y that P needs to move

    p_r_new = p_current_r + p_velocity_new * dt          # new coordinates of P

    return [p_r_new, p_velocity_new]

def update_state_PN(Prop_Nav_Guidance,ti1, ti2, pi1, pi2, N, dt): 
    p_r_new = 0
    p_velocity_new = 0 
    return [p_r_new, p_velocity_new]

def update_state_APN(Aug_Prop_Nav_Guidance,ti1, ti2, pi1, pi2, N, dt): 
    p_r_new = 0
    p_velocity_new = 0 
    return [p_r_new, p_velocity_new]


def Main_loop(ti1, ti2, pi1, pi2, dt, x_range):

    Gl = "PP"

    # T trajectory

    t_x, t_y = target_states_values(ti1, ti2, dt, x_range, sin_T_trajectory)

    plt.plot(t_x, t_y, color = "red")

    plt.show()

    # P trajectory

    # initalize P x & y values
    p_x_pre = np.array([pi1[0], pi2[0]])
    p_y_pre = np.array([pi1[1], pi2[1]])

    p_vx_pre = np.array(([pi1[2], pi2[2]]))
    p_vy_pre = np.array(([pi1[3], pi2[3]]))

    p_x_post = np.zeros(len(t_x)-2) 
    p_y_post = np.zeros(len(t_x)-2)

    p_x = np.insert(p_x_post, 0, p_x_pre)   # position values
    p_y = np.insert(p_y_post, 0, p_y_pre)

    p_vx = np.insert(p_x_post, 0, p_vx_pre) # velocity values
    p_vy = np.insert(p_y_post, 0, p_vy_pre)

    # Loop to find P coords based on T place

    for k in range(1, len(t_x - 1)): #iterate through all values and assign a time value

        t_previous_coords = np.array([t_x[k-1], t_y[k-1]])
        t_current_coords = np.array([t_x[k], t_y[k]])

        # p_previous_coords = np.array([p_x[k-1], p_y[k-1]])
        # p_current_coords = np.array([p_x[k], p_y[k]])

        # p_previous_velocity = np.array([p_vx[k-1], p_vy[k-1]])
        # p_current_velocity = np.array([p_vx[k], p_vy[k]])

        p_previous_state = np.array([p_x[k-1], p_y[k-1], p_vx[k-1], p_vy[k-1]])
        p_current_state = np.array([p_x[k], p_y[k], p_vx[k], p_vy[k]])

        if Gl == "PP":
            [p_x[k+1], p_y[k+1]] = update_state_PP(Pure_Pursuit_Guidance, p_current_state, t_current_coords, dt)

        elif Gl == "PN":
            [p_x[k+1], p_y[k+1]] =update_state_PN()

        #[p_x[k+1], p_y[k+1]] = p_new_coords









    return t_x, t_y




#print(Main_loop(t0, t1, dt, x_range))
print(update_state_PP(Pure_Pursuit_Guidance, p1, t1, dt))

# print(f"x values: {Main_loop(t0, t1, dt, x_range)[0]}")
# print(f"y values: {Main_loop(t0, t1, dt, x_range)[1]}")    

#print(f"from APN: {Aug_Prop_Nav_Guidance(t0,t1,p0,p1, N, dt)}")
#print(f"from PN: {Prop_Nav_Guidance(t0,t1,p0,p1, N, dt)[0]}")







    





    


















    



