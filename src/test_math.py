import numpy as np


planning_time = 15
delta_time = 0.01
num_points =  int(np.round(planning_time/delta_time,0))

print(num_points)