from InitialConditions import *
from Simulator import run_trial
import numpy as np

# inital configs
law_names = ["PP", "PN", "APN"]
e_values = np.linspace(0,500,11)
n_trials = 10

#pre-allocated arrays
n_laws, n_e = len(law_names), len(e_values)
miss_distance = np.zeros((n_laws, n_e, n_trials))
hit = np.zeros((n_laws, n_e, n_trials), dtype=bool)
intercept_time = np.full((n_laws, n_e, n_trials), np.nan)

# looping trials
for law_index, this_law_name in enumerate(law_names): 
    for e_index, this_e_value in enumerate(e_values): 
        for trial in range(n_trials): 

            p_traj, t_traj = run_trial(e = this_e_value, law_name=this_law_name, law_params = law_params) #retrive trajectories of both P & T

            distance_between = p_traj[:, :3] - t_traj[:, :3] #first 3 columns of both

            mag_of_dist = np.linalg.norm(distance_between, axis = 1)

            miss_distance[law_index, e_index, trial] = mag_of_dist.min()

            if miss_distance[law_index, e_index, trial] < kill_zone_size: 
                hit[law_index, e_index, trial] = True
            else: 
                hit[law_index, e_index, trial] = False

            if hit[law_index, e_index, trial] == True:
                intercept_time[law_index, e_index, trial] = (len(p_traj)-1) * dt

np.set_printoptions(precision=2, suppress=True)

#take mean values for each trial

mean_miss = miss_distance.mean(axis=2)
hit_rate = hit.mean(axis=2)
mean_time = np.nanmean(intercept_time, axis=2)

print(f"mean miss distance [law, e]\n{mean_miss}\nhit rate [law, e]\n{hit_rate}\nmean intercept time [law, e]\n{mean_time}")
