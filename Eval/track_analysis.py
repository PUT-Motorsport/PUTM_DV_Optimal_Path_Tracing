import numpy as np
from matplotlib import pyplot as plt
import pandas as pd

"""
Extract some basic statistics about the track to generate heuristics.
"""

df = pd.read_csv('tracks/berlin_2018.csv')

x = df['x_m'].to_numpy()
y = df['y_m'].to_numpy()
w_l = df['w_tr_left_m'].to_numpy()
w_r = df['w_tr_right_m'].to_numpy()

total_track_width = w_l + w_r

print(f'Max track width: {max(total_track_width)}')
print(f'Min track width: {min(total_track_width)}')