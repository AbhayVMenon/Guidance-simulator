import numpy as np, os, matplotlib.pyplot as plt, math as m

os.system("cls")

#T = target, P = Pursuer

dt = 0.1 # time step

#target inital state vector 
t0 = np.array([100, 0, 1000, 0]) # m
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
def Pure_Pursuit_Guidance(ti, pi):

    p_ri = pi[0:2]          #P inital position
    t_ri = ti[0:2]          #T inital position

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
def target_states_values(ti1, dt, x_range, trajectory):

    x_start = ti1[0]

    x = np.arange(x_start, x_range + dt, dt)

    y = trajectory(x, x_start)

    return x, y

def Main_loop(ti1, dt, x_range):

    # T trajectory

    x, y = target_states_values(ti1, dt, x_range, sin_T_trajectory)

    plt.plot(x, y, color = "red")

    plt.show()


Main_loop(t0, dt, x_range)

# print(f"x values: {Main_loop(t0, dt, x_range)[0]}")
# print(f"y values: {Main_loop(t0, dt, x_range)[1]}")    









#print(f"from APN: {Aug_Prop_Nav_Guidance(t0,t1,p0,p1, N, dt)}")
#print(f"from PN: {Prop_Nav_Guidance(t0,t1,p0,p1, N, dt)[0]}")







    





    


















    



