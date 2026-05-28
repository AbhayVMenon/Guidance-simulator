import numpy as np, os, matplotlib.pyplot as plt, math as m, mpl_toolkits.mplot3d as Axes3D

os.system("cls")

#T = target, P = Pursuer

dt = 0.01 # time step
speed_multipier = 1.44 # How much faster P is than T
target_velocity_mag = 1000

#Pursuer inital state vector
p0 = np.array([0, 500, 0, (speed_multipier * target_velocity_mag), 0, 0])
p1 = p0 + np.array([(dt * speed_multipier * target_velocity_mag), 0, 0, 0, 0, 0]) # @t = 0.01

# np.random.randint(-1000,1000)
x_start = 1000
x_range = 20000
x = np.arange(0,x_range,0.1)   #range of sim window

#global constants
g = 9.81
a = [0, -g] # m/s^2
N = 3

# Guidance Laws
def Pure_Pursuit_Guidance(ti2, pi2):

    p_ri = pi2[0:3]          #P inital position
    t_ri = ti2[0:3]          #T inital position

    los = t_ri - p_ri       #vector between P & T

    mag_los = np.linalg.norm(los)

    # if mag_los < 3:         #size of missile
    #     unit_los = np.zeros(2)
    
    # else:
    unit_los = los / mag_los

    return unit_los

def Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt):

    #LOS Rate
    t_ri1 = ti1[0:3]
    t_ri2 = ti2[0:3]  #T inital positions 

    p_ri1 = pi1[0:3]
    p_ri2 = pi2[0:3]  #P inital positions

    t_vi2 = ti2[3:6]
    p_vi2 = pi2[3:6]  #P inital velocities 

    los1 = t_ri1 - p_ri1
    los2 = t_ri2 - p_ri2
    unit_los2 = los2 / np.linalg.norm(los2)

    # los_angle1 = np.arctan2(los1[1],los1[0])
    # los_angle2 = np.arctan2(los2[1],los2[0])

    # angles = np.unwrap([los_angle1, los_angle2])

    # los_rate = (angles[1] - angles[0]) / dt

    #closing rate 
    v_rel = p_vi2 - t_vi2

    los_cross_vrel = np.cross(los2, v_rel)

    los_rate = -los_cross_vrel / np.linalg.norm(los2)**2

    Vc = np.dot(v_rel, unit_los2) # closing speed 

    perp_los2 = np.cross(los_rate, unit_los2)

    a_c = N * Vc * perp_los2

    #print(f"Vc = {Vc}, los_rate = {los_rate}")

    #print(f"los_rate={los_rate:.4f}, Vc={Vc:.4f}, unit_perp={unit_perp_los2}")

    return a_c, unit_los2
    
def Aug_Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt):

    a_c_PN, unit_los2 = Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt)

    t_vi1 = ti1[3:6]
    t_vi2 = ti2[3:6]

    t_a = (t_vi2 - t_vi1)/dt

    t_a_perp = t_a - (np.dot(t_a, unit_los2) * unit_los2)  #component perp to LOS w/o component parallel to LOS

    a_c_Aug = a_c_PN + N/2 * t_a_perp

    return a_c_Aug

def ac_into_velocity_and_position(pi2, dt, a_c):

    p_current_r = pi2[0:3]

    p_current_v = pi2[3:6]
    
    p_velocity_new_raw = p_current_v + a_c * dt

    unit_p_velocity_new = p_velocity_new_raw / np.linalg.norm(p_velocity_new_raw)

    p_velocity_new = np.linalg.norm(p_current_v) * unit_p_velocity_new

    p_r_new = p_current_r + p_velocity_new * dt

    return [p_r_new, p_velocity_new]

# Target Trajectories
def get_velocity(t_x, t_y, t_z, dt, target_velocity_mag):

    t_v_raw = np.array([np.gradient(t_x, dt), np.gradient(t_y, dt), np.gradient(t_z, dt)])

    unit_t_v = t_v_raw / np.linalg.norm(t_v_raw, axis = 0)

    (t_vx, t_vy, t_vz) = target_velocity_mag * unit_t_v

    return t_vx, t_vy, t_vz


#2D trajectories
def Straight(x, x_start):
    t_y = np.zeros(len(x))
    t_z = np.zeros(len(x))
    t_vx, t_vy, t_vz = get_velocity(x, t_y, t_z, dt, target_velocity_mag)

    return t_y, t_z, t_vx, t_vy, t_vz

def Sinusodial(x, x_start):

    A = 1000
    w = 0.001

    t_y = A * np.sin(w * (x - x_start))
    t_z = np.zeros(len(x - x_start))
    t_vx, t_vy, t_vz = get_velocity(x, t_y, t_z, dt, target_velocity_mag)

    return t_y, t_z, t_vx, t_vy, t_vz


#3D trajectories
def Helix(x, x_start):

    A = 500
    L = 3300

    t_y = A * np.sin(2 * np.pi / L * (x - x_start))
    t_z = A * np.cos(2 * np.pi / L * (x - x_start))

    t_vx, t_vy, t_vz = get_velocity(x, t_y, t_z, dt, target_velocity_mag)

    return t_y, t_z, t_vx, t_vy, t_vz

# plot coordinates
def target_states_values(dt, x_range, x_start, target_velocity_mag, trajectory):

    x = np.arange(x_start, x_range + dt * target_velocity_mag, dt * target_velocity_mag)

    y, z, vx, vy, vz = trajectory(x, x_start)

    return x, y, z, vx, vy, vz

def update_state_PP(Pure_Pursuit_Guidance, pi2, ti2, dt):

    p_current_r = pi2[0:3]    #current position of P
    t_current_r = ti2[0:3]    #current position of T

    p_current_v = pi2[3:6]    #current velocity of P

    p_v_mag = np.linalg.norm(p_current_v)

    new_direction = Pure_Pursuit_Guidance(t_current_r, p_current_r)

    p_velocity_new = p_v_mag * new_direction  # amount in x & y that P needs to move

    p_r_new = p_current_r + p_velocity_new * dt          # new coordinates of P

    return [p_r_new, p_velocity_new]

def update_state_PN(Prop_Nav_Guidance,ti1, ti2, pi1, pi2, N, dt): 

    a_c = Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt)[0]
    
    return ac_into_velocity_and_position(pi2, dt, a_c)

def update_state_APN(Aug_Prop_Nav_Guidance,ti1, ti2, pi1, pi2, N, dt): 

    a_c_Aug = Aug_Prop_Nav_Guidance(ti1, ti2, pi1, pi2, N, dt)

    return ac_into_velocity_and_position(pi2, dt, a_c_Aug)

def Main_loop(pi1, pi2, dt, x_range):

    Gl = "APN"   # Define what guidance law I wanna use

    # T trajectory

    (t_x, t_y, t_z, t_vx, t_vy, t_vz) = target_states_values(dt, x_range, x_start, target_velocity_mag, Helix)

    # P trajectory

    # initalize P x, y & z values
    p_x_pre = np.array([pi1[0], pi2[0]])
    p_y_pre = np.array([pi1[1], pi2[1]])
    p_z_pre = np.array([pi1[2], pi2[2]])

    p_vx_pre = np.array(([pi1[3], pi2[3]]))
    p_vy_pre = np.array(([pi1[4], pi2[4]]))
    p_vz_pre = np.array(([pi1[5], pi2[5]]))

    p_post = np.zeros(len(t_x)-2) 

    p_x = np.insert(p_post, 0, p_x_pre)   # position values
    p_y = np.insert(p_post, 0, p_y_pre)
    p_z = np.insert(p_post, 0, p_z_pre)

    p_vx = np.insert(p_post, 0, p_vx_pre) # velocity values
    p_vy = np.insert(p_post, 0, p_vy_pre)
    p_vz = np.insert(p_post, 0, p_vz_pre)

    # # Loop to find P coords based on T place

    for k in range(1, len(t_x) - 1): #iterate through all values and assign a time value

        t_current_coords = np.array([t_x[k], t_y[k], t_z[k], t_vx[k], t_vy[k], t_vz[k]])
        p_current_state = np.array([p_x[k], p_y[k], p_z[k], p_vx[k], p_vy[k], p_vz[k]])

        if Gl == "PP":
            ((p_x[k+1], p_y[k+1], p_z[k+1]), (p_vx[k+1], p_vy[k+1], p_vz[k+1])) = np.array(update_state_PP(Pure_Pursuit_Guidance, p_current_state, t_current_coords, dt))
                        
        else:
            t_previous_coords = np.array([t_x[k-1], t_y[k-1], t_z[k-1], t_vx[k-1], t_vy[k-1], t_vz[k-1]])
            p_previous_state = np.array([p_x[k-1], p_y[k-1], p_z[k-1], p_vx[k-1], p_vy[k-1], p_vz[k-1]])

            if Gl == 'PN':

                ((p_x[k+1], p_y[k+1], p_z[k+1]), (p_vx[k+1], p_vy[k+1], p_vz[k+1])) = np.array(update_state_PN(Prop_Nav_Guidance, t_previous_coords, t_current_coords, p_previous_state, p_current_state, N, dt))

            
            if Gl == 'APN': 
                ((p_x[k+1], p_y[k+1], p_z[k+1]), (p_vx[k+1], p_vy[k+1], p_vz[k+1])) = np.array(update_state_APN(Aug_Prop_Nav_Guidance, t_previous_coords, t_current_coords, p_previous_state, p_current_state, N, dt))

        #[p_x[k+1], p_y[k+1]] = p_new_coords        
        distance_between = np.sqrt((p_x[k] - t_x[k])**2 + (p_y[k] - t_y[k])**2 + (p_x[k] - t_x[k])**2)

        if distance_between < 25:
            k_intercept = k
            break

    #Plotting 

    # fig = plt.figure()
    # ax = fig.add_subplot(projection = "3d")



    # myLabels = ["Target", "Pursuer"]
    # ax.plot(t_x[0:(k)], t_y[0:(k)], t_z[0:(k)], color = "red", label = myLabels[0])
    # ax.plot(p_x[0:k], p_y[0:k], p_z[0:k], color = "blue", label = myLabels[1])
    # ax.set_xlabel("x displacment")
    # ax.set_ylabel("y displacment")
    # ax.set_zlabel("z displacment")
    # ax.grid(False)
    
    # max_k = len(t_x) - 1
    # max_k_x = len(t_x[0:(k-1)]) 
    # mid = int(max_k / 2)
    # mid_x = int(max_k_x / 2)
    
    # print(t_x[mid])

    # #ax.quiver(t_x[mid_x], t_y[mid], t_z[mid], t_vx[mid], t_vy[mid], t_vz[mid], normalize = True, length = 100, color = "red")
    
    # print(t_x[k+1])    
    


    # plt.legend()
    # plt.show()

    from matplotlib.animation import FuncAnimation

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    # plot full trajectories as faint trails
    ax.plot(t_x[0:k], t_y[0:k], t_z[0:k], color="red", alpha=0.2)
    ax.plot(p_x[0:k], p_y[0:k], p_z[0:k], color="blue", alpha=0.2)

    # animated points
    target_point, = ax.plot([], [], [], 'o', color="red", markersize=4, label="Target")
    pursuer_point, = ax.plot([], [], [], 'o', color="blue", markersize=4, label="Pursuer")

    # animated trails that grow over time
    target_trail, = ax.plot([], [], [], color="red", linewidth=1.5)
    pursuer_trail, = ax.plot([], [], [], color="blue", linewidth=1.5)

    ax.set_xlabel("x displacement")
    ax.set_ylabel("y displacement")
    ax.set_zlabel("z displacement")

    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_zticklabels([])

    ax.grid(False)

    plt.legend()

    if t_x[k+1] == x_range:
        print("Unsuccessful interception")
    else:
        print(f"x displacment at intercept: {t_x[k+1]}")

    def update(frame):
        target_point.set_data([t_x[frame]], [t_y[frame]])
        target_point.set_3d_properties([t_z[frame]])

        pursuer_point.set_data([p_x[frame]], [p_y[frame]])
        pursuer_point.set_3d_properties([p_z[frame]])

        target_trail.set_data(t_x[0:frame], t_y[0:frame])
        target_trail.set_3d_properties(t_z[0:frame])

        pursuer_trail.set_data(p_x[0:frame], p_y[0:frame])
        pursuer_trail.set_3d_properties(p_z[0:frame])

        if frame == k - 1:
            if t_x[k+1] != x_range:
                ax.text2D(0.5, 0.5, "INTERCEPT", transform=ax.transAxes,
                  fontsize=20, color="white", fontweight="bold",
                  ha="center", va="center",
                  bbox=dict(boxstyle="round", facecolor="green", alpha=0.8))
                
            else: 
                ax.text2D(0.5, 0.5, "UNSUCCESSFUL", transform=ax.transAxes,
                  fontsize=20, color="white", fontweight="bold",
                  ha="center", va="center",
                  bbox=dict(boxstyle="round", facecolor="red", alpha=0.8))

        return target_point, pursuer_point, target_trail, pursuer_trail

    ani = FuncAnimation(fig, update, frames=k, interval=0.01, blit=False)



    
    
    plt.show()

    

    # print(max_k)
    # print(k + 1)

    

Main_loop(p0, p1, dt, x_range)
