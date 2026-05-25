import numpy as np, os, matplotlib.pyplot as plt, math as m

os.system("cls")

#T = target, P = Pursuer

dt = 0.01 # time step
speed_multipier = 1.44 # How much faster P is than T
target_velocity_mag = 1000

#target inital state vector 
t0 = np.array([1000, 0, target_velocity_mag, 0]) # m
t1 = t0 + np.array([(dt * t0[2]), 0, 0, 0]) # @t = 0.01

#Pursuer inital state vector
p0 = np.array([0, 500, (speed_multipier * t0[2]), 0])
p1 = p0 + np.array([(dt * 1.1 * t0[2]), 0, 0, 0]) # @t = 0.01

# np.random.randint(-1000,1000)
x_range = 20000
x = np.arange(0,x_range,0.1)   #range of sim window
#global constants
g = 9.81
a = [0, -9.81] # m/s^2
N = 3

# Guidance Laws
def Pure_Pursuit_Guidance(ti2, pi2):

    p_ri = pi2[0:2]          #P inital position
    t_ri = ti2[0:2]          #T inital position

    los = t_ri - p_ri       #vector between P & T

    mag_los = np.linalg.norm(los)

    # if mag_los < 3:         #size of missile
    #     unit_los = np.zeros(2)
    
    # else:
    unit_los = los / mag_los

    return unit_los

def update_state_PP(Pure_Pursuit_Guidance, pi2, ti2, dt):

    p_current_r = pi2[0:2]    #current position of P
    t_current_r = ti2[0:2]    #current position of T

    p_current_v = pi2[2:4]    #current velocity of P

    p_v_mag = np.linalg.norm(p_current_v)

    new_direction = Pure_Pursuit_Guidance(t_current_r, p_current_r)

    p_velocity_new = p_v_mag * new_direction  # amount in x & y that P needs to move

    p_r_new = p_current_r + p_velocity_new * dt          # new coordinates of P

    return [p_r_new, p_velocity_new]


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

    # los_angle1 = np.arctan2(los1[1],los1[0])
    # los_angle2 = np.arctan2(los2[1],los2[0])

    # angles = np.unwrap([los_angle1, los_angle2])

    # los_rate = (angles[1] - angles[0]) / dt

    #closing rate 
    v_rel = p_vi2 - t_vi2

    los_cross_vrel = np.cross(np.append(los2, 0), np.append(v_rel, 0))[2]

    los_rate = -los_cross_vrel / np.linalg.norm(los2)**2

    Vc = np.dot(v_rel, unit_los2) # closing speed 

    perp_los2 = np.array(([0, -1], [1, 0])) @ los2

    unit_perp_los2 = perp_los2 / np.linalg.norm(perp_los2)

    a_c = N * Vc * los_rate * unit_perp_los2

    print(f"Vc = {Vc}, los_rate = {los_rate}")

    #print(f"los_rate={los_rate:.4f}, Vc={Vc:.4f}, unit_perp={unit_perp_los2}")

    return a_c, unit_perp_los2

def update_state_PN(Prop_Nav_Guidance,ti1, ti2, pi1, pi2, N, dt): 

    p_current_r = pi2[0:2]

    p_current_v = pi2[2:4]
    
    a_c = Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt)[0]

    p_velocity_new_raw = p_current_v + a_c * dt

    unit_p_velocity_new = p_velocity_new_raw / np.linalg.norm(p_velocity_new_raw)

    p_velocity_new = np.linalg.norm(p_current_v) * unit_p_velocity_new

    p_r_new = p_current_r + p_velocity_new * dt

    #print(f"k, a_c = {a_c}")

    #print(np.linalg.norm(p_velocity_new))
    
    return [p_r_new, p_velocity_new] 


def Aug_Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt):

    a_c_PN, unit_perp_los2 = Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt)

    t_vi1 = ti1[2:4]
    t_vi2 = ti2[2:4]

    t_a = (t_vi2 - t_vi1)/dt

    t_a_perp = np.dot(unit_perp_los2, t_a)  #component perp to LOS

    a_c_Aug = a_c_PN + N/2 * t_a_perp

    return a_c_Aug

def update_state_APN(Aug_Prop_Nav_Guidance,ti1, ti2, pi1, pi2, N, dt): 
    p_r_new = 0
    p_velocity_new = 0 
    return [p_r_new, p_velocity_new]

# Target Trajectories
def get_velocity(t_x, t_y, dt, target_velocity_mag): 

    t_v_raw = np.array([np.gradient(t_x, dt), np.gradient(t_y, dt)])

    unit_t_v = t_v_raw / (np.sqrt(t_v_raw[0]**2 + t_v_raw[1]**2))

    (t_vx, t_vy) = target_velocity_mag * unit_t_v

    return t_vx, t_vy

def straight_T_trajectory(x, x_start):
    t_y = np.zeros(len(x))
    t_vx, t_vy = get_velocity(x, t_y, dt, target_velocity_mag)

    return t_y, t_vx, t_vy


def sin_T_trajectory(x, x_start):

    c = x_start
    A = 1000
    w = 0.001

    t_y = A * np.sin(w * (x - c))
    t_vx, t_vy = get_velocity(x, t_y, dt, target_velocity_mag)

    return t_y, t_vx, t_vy

# plot coordinates
def target_states_values(ti1, ti2, dt, x_range, trajectory):

    x_start = ti2[0]

    x_pre = np.arange(x_start, x_range + dt * ti2[2], dt * ti2[2])
    x = np.insert(x_pre, 0, ti1[0]) #prepending 1st state

    y_pre, vx_pre, vy_pre = trajectory(x_pre, x_start)
    
    #y_pre = trajectory(x_pre, x_start)[0]
    y = np.insert(y_pre, 0, ti1[1])
    #vx_pre = trajectory(x_pre, x_start)[1]
    vx = np.insert(vx_pre, 0, ti1[2])
    #vy_pre = trajectory(x_pre, x_start)[2]
    vy = np.insert(vy_pre, 0, ti1[3])

    

    return x, y, vx, vy


def Main_loop(ti1, ti2, pi1, pi2, dt, x_range):

    Gl = "PN"   # Define what guidance law I wanna use

    # T trajectory

    (t_x, t_y, t_vx, t_vy) = target_states_values(ti1, ti2, dt, x_range, sin_T_trajectory)

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

    # # Loop to find P coords based on T place

    for k in range(1, len(t_x) - 1): #iterate through all values and assign a time value

        t_current_coords = np.array([t_x[k], t_y[k], t_vx[k], t_vy[k]])
        p_current_state = np.array([p_x[k], p_y[k], p_vx[k], p_vy[k]])

        # p_previous_coords = np.array([p_x[k-1], p_y[k-1]])
        # p_current_coords = np.array([p_x[k], p_y[k]])

        # p_previous_velocity = np.array([p_vx[k-1], p_vy[k-1]])
        # p_current_velocity = np.array([p_vx[k], p_vy[k]])

        if Gl == "PP":
            ((p_x[k+1], p_y[k+1]), (p_vx[k+1], p_vy[k+1])) = np.array(update_state_PP(Pure_Pursuit_Guidance, p_current_state, t_current_coords, dt))
                        
        else:
            t_previous_coords = np.array([t_x[k-1], t_y[k-1], t_vx[k-1], t_vy[k-1]])
            p_previous_state = np.array([p_x [k-1], p_y[k-1], p_vx[k-1], p_vy[k-1]])

            if Gl == 'PN':

                ((p_x[k+1], p_y[k+1]), (p_vx[k+1], p_vy[k+1])) = np.array(update_state_PN(Prop_Nav_Guidance,t_previous_coords, t_current_coords, p_previous_state, p_current_state, N, dt))

            
            if Gl == 'APN': 
                ((p_x[k+1], p_y[k+1]), (p_vx[k+1], p_vy[k+1])) = update_state_APN(Prop_Nav_Guidance,ti1, ti2, pi1, pi2, N, dt)

        #[p_x[k+1], p_y[k+1]] = p_new_coords        
        distance_between = np.sqrt((p_x[k] - t_x[k])**2 + (p_y[k] - t_y[k])**2)

        if distance_between < 25:
            k_intercept = k
            break
    
    myLabels = ["Target", "Pursuer"]
    plt.plot(t_x[0:k-1], t_y[0:k-1], color = "red", label = myLabels[0])
    plt.plot(p_x[0:k], p_y[0:k], color = "blue", label = myLabels[1])
    plt.xlabel("x displacment")
    plt.ylabel("y displacment")
    
    # plt.plot(p_vx, p_vy, color = "green")
    # plt.pause(0.001)

    plt.legend()
    plt.show()

    max_k = len(t_x) - 1

    # print(max_k)
    # print(k + 1)

Main_loop(t0, t1, p0, p1, dt, x_range)



#print (f"P x values{Main_loop(t0, t1, p0, p1, dt, x_range)[0]}")
#print (f"P y values{Main_loop(t0, t1, p0, p1, dt, x_range)[1]}")

# print(f"x values: {Main_loop(t0, t1, dt, x_range)[0]}")
# print(f"y values: {Main_loop(t0, t1, dt, x_range)[1]}")    

#print(f"from APN: {Aug_Prop_Nav_Guidance(t0,t1,p0,p1, N, dt)}")
#print(f"from PN: {Prop_Nav_Guidance(t0,t1,p0,p1, N, dt)[0]}")
