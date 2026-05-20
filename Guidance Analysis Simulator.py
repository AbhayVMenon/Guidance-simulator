import numpy as np, os, matplotlib as plt, math as m

os.system("cls")

#T = target, P = Pursuer

dt = 0.01 # time step

#target inital state vector 
t0 = np.array([100, 0, 1000, 0]) # m
t1 = t0 + np.array([(dt * t0[2]), 0, 0, 0]) # @t = 0.01

#Pursuer inital state vector
p0 = np.array([0, 500, (1.1 * t0[2]), 0])
p1 = p0 + np.array([(dt * 1.1 * t0[2]), 0, 0, 0]) # @t = 0.01

# np.random.randint(-1000,1000)

#global constants
g = 9.81
a = [0, -9.81] # m/s^2
N = 3
dt = 0.01

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

    return a_c



print(Prop_Nav_Guidance(t0,t1,p0,p1, N, dt))



    





    


















    



