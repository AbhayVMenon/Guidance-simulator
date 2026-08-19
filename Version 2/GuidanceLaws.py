from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np

@dataclass
class State: 
    r: np.ndarray #container for current pos (rx, ry, rz)
    v: np.ndarray #container for current vel (vx, vy, vz)

class GuidanceLaw(ABC): 
    def __init__(self):
        pass

    @abstractmethod
    def compute_command(self, p_state, t_state): #will not run unless that child class has one of these
        pass 

    def reset(self): #only use when a Gl defines something that others dont need
        pass

class PurePursuit(GuidanceLaw): 
    def __init__(self, max_gs): 
        self.max_gs = max_gs
        self.max_accel = max_gs * 9.81
        

    def compute_command(self, p_state, t_state):
        p_ri = p_state.r       #P inital position
        t_ri = t_state.r       #T inital position
        los = t_ri - p_ri       #vector between P & T

        mag_los = np.linalg.norm(los)


        unit_los = los / mag_los

        a_c = unit_los * self.max_accel

        return a_c

class PropNav(GuidanceLaw): 
    def __init__(self, N, max_gs): 
        self.N = N
        self.max_gs = max_gs
        self.max_accel = max_gs * 9.81
        

    def compute_command(self, p_state, t_state):
            
            #LOS Rate
            t_ri2 = t_state.r  #T inital positions 
            p_ri2 = p_state.r  #P inital positions
        
            t_vi2 = t_state.v #inital velocities 
            p_vi2 = p_state.v  
        
            los2 = t_ri2 - p_ri2
            self.unit_los2 = los2 / np.linalg.norm(los2)
        
            v_rel = p_vi2 - t_vi2
        
            los_cross_vrel = np.cross(los2, v_rel)
        
            los_rate = -los_cross_vrel / np.linalg.norm(los2)**2
        
            Vc = np.dot(v_rel, self.unit_los2) # closing speed 
        
            perp_los2 = np.cross(los_rate, self.unit_los2)
        
            a_c_PN = self.N * Vc * perp_los2

            if np.linalg.norm(a_c_PN) > self.max_accel:
                direction_PN = a_c_PN / np.linalg.norm(a_c_PN)
                a_c = direction_PN * self.max_accel
            else:
                a_c = a_c_PN

            return a_c

class AugPropNav(GuidanceLaw):
    def __init__(self, N, dt, max_gs):
        self.pn = PropNav(N, max_gs)
        self.N = N
        self.dt = dt
        self.prev_t_vel = None
        self.max_gs = max_gs
        self.max_accel = max_gs * 9.81

    def compute_command(self, p_state, t_state): 
        a_c_PN = self.pn.compute_command(p_state, t_state)
        unit_los2 = self.pn.unit_los2

        if self.prev_t_vel is None: #cold start 
            a_c_APN = a_c_PN
            
        else:

            t_vi1 = self.prev_t_vel
            t_vi2 = t_state.v

            t_a = (t_vi2 - t_vi1)/self.dt

            t_a_perp = t_a - (np.dot(t_a, unit_los2) * unit_los2)  #component perp to LOS w/o component parallel to LOS

            a_c_APN = a_c_PN + self.N/2 * t_a_perp

        self.prev_t_vel = t_state.v

        if np.linalg.norm(a_c_APN) > self.max_accel:
            direction_APN = a_c_APN / np.linalg.norm(a_c_APN)
            a_c = direction_APN * self.max_accel
        else: 
            a_c = a_c_APN

        return a_c

    def reset(self):
        self.prev_t_vel = None


        