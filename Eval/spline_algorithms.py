import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import math

"""
Testing the track bounds recovery algorithms
"""

from os import getcwd
df = pd.read_csv(getcwd() + '/Eval/tracks/berlin_2018.csv')

x_middle = df['x_m'].to_numpy()
y_middle = df['y_m'].to_numpy()

assert(len(x_middle) == len(y_middle))

figure, axis = plt.subplots(2, 2)

axis[0, 0].plot(x_middle, y_middle, label='Centerline (provided)', color='gray')
axis[0, 0].grid()

# generate random cones positions

# load track widths
right_offset = df['w_tr_right_m'].to_numpy()
left_offset = df['w_tr_left_m'].to_numpy()

x_left = np.empty(*x_middle.shape)
y_left = np.empty(*x_left.shape)
x_right = np.empty(*x_left.shape)
y_right = np.empty(*x_left.shape)

#calculate tangent vector for each point

for index in range(len(x_middle)):
    #approximate_track_length_difference = math.sqrt((x_middle[index] - x_middle[index - 1])**2 + \
    #    (y_middle[index] - y_middle[index - 1])**2) + math.sqrt((x_middle[(index + 1) % len(x_middle)] - \
    #    x_middle[index])**2 + (y_middle[(index + 1) % len(x_middle)] - y_middle[index])**2)

    derivative = (y_middle[(index + 1) % len(y_middle)] - y_middle[index]) / (x_middle[(index + 1) % len(x_middle)] - x_middle[index])
    
    if derivative != 0:
        parallel_slope = (-1) / derivative
    else:
        parallel_slope = 1
    # find a point that is right_offset away, whoose slope is parallel_slope

    # from delta y / delta x = parallel slope

    delta_X = math.sqrt((left_offset[index])**2 / (1 + parallel_slope**2))
    x_left[index] = x_middle[index] - delta_X
    y_left[index] = y_middle[index] - delta_X * parallel_slope

    x_right[index] = x_middle[index] + delta_X
    y_right[index] = y_middle[index] + delta_X * parallel_slope

    # fix switched sides

    tangent_vector = np.array([x_middle[(index + 1) % len(x_middle)] - x_middle[index], (y_middle[(index + 1) % len(x_middle)] - y_middle[index])])
    right_vector = np.array([x_right[index] - x_middle[index], y_right[index] - y_middle[index]])

    cross = np.cross(right_vector, tangent_vector)

    if (cross < 0):     # cross product negative, points must be switched
        x_left[index], x_right[index] = x_right[index], x_left[index]
        y_left[index], y_right[index] = y_right[index], y_left[index] 

axis[0, 0].plot(x_left, y_left, label='left')
axis[0, 0].plot(x_right, y_right, label='right')
axis[0, 0].legend()
axis[0, 0].set_title('Original track')

# Collect random cones

from random import sample

indexes = [i for i in range(len(x_left))]
random_indexes_left = sample(indexes, 300)   # pick random 300 elements, retaining the order
random_indexes_right = sample(indexes, 300)

random_indexes_left.sort()
random_indexes_right.sort()

cones_left_x = x_left[random_indexes_left]
cones_left_y = y_left[random_indexes_left]
cones_right_x = x_right[random_indexes_right]
cones_right_y = y_right[random_indexes_right]

axis[1, 0].scatter(cones_left_x, cones_left_y, c='red')
axis[1, 0].scatter(cones_right_x, cones_right_y, c='blue')
axis[1, 0].set_title('Randomly selected cones')
axis[1, 0].grid()

# attempt to restore the track 

import splines

import time #bemchmark
t1 = time.time()

# parametrize the spline using integers
assert(len(cones_right_x) == len(cones_right_y))
assert(len(cones_left_x) == len(cones_left_y))


t = np.arange(0, len(cones_left_x))
x_spline_left = splines.CubicSpline(t, cones_left_x)
y_spline_left = splines.CubicSpline(t, cones_left_y)
x_spline_right = splines.CubicSpline(t, cones_right_x)
y_spline_right = splines.CubicSpline(t, cones_right_y)

t_plot = np.arange(0, max(t), 0.1)
restored_x_left = np.empty(*t_plot.shape)
restored_y_left = np.empty(*t_plot.shape)
restored_x_right = np.empty(*t_plot.shape)
restored_y_right = np.empty(*t_plot.shape)
for index, value in enumerate(t_plot):
    restored_x_left[index] = x_spline_left.get(value)
    restored_y_left[index] = y_spline_left.get(value)
    restored_x_right[index] = x_spline_right.get(value)
    restored_y_right[index] = y_spline_right.get(value)

t2 = time.time()

axis[1, 1].set_title(f'Restored track sides via [0-1) spline interpolation. {t2 - t1} s')
axis[1, 1].plot(restored_x_left, restored_y_left, label='Left')
axis[1, 1].plot(restored_x_right, restored_y_right, label='Right')
axis[1, 1].legend()

axis[1, 1].grid()

# paramterize the spline using L2 distance between points as the parameter

t1 = time.time()

left_param = np.zeros(*cones_left_x.shape)
right_param = np.zeros(*cones_right_x.shape)

for index in range(1, len(left_param)):
    left_param[index] = left_param[index - 1]
    left_param[index] += math.sqrt((cones_left_x[index] - cones_left_x[index - 1])**2 + \
        (cones_left_y[index] - cones_left_y[index - 1])**2)

for index in range(1, len(right_param)):
    right_param[index] = right_param[index - 1]
    right_param[index] += math.sqrt((cones_right_x[index] - cones_right_x[index - 1])**2 + \
        (cones_right_y[index] - cones_right_y[index - 1])**2)

right_x_spline = splines.CubicSpline(right_param, cones_right_x)
right_y_spline = splines.CubicSpline(right_param, cones_right_y)
left_x_spline = splines.CubicSpline(left_param, cones_left_x)
left_y_spline = splines.CubicSpline(left_param, cones_left_y)

t_plot_left = np.arange(0, max(left_param), 0.01)
t_plot_right = np.arange(0, max(right_param), 0.01)

restored_x_left = np.empty(*t_plot_left.shape)
restored_y_left = np.empty(*t_plot_left.shape)
restored_x_right = np.empty(*t_plot_right.shape)
restored_y_right = np.empty(*t_plot_right.shape)

for index, value in enumerate(t_plot_left):
    restored_x_left[index] = left_x_spline.get(value)
    restored_y_left[index] = left_y_spline.get(value)

for index, value in enumerate(t_plot_right):
    restored_x_right[index] = right_x_spline.get(value)
    restored_y_right[index] = right_y_spline.get(value)

t2 = time.time()

axis[0, 1].plot(restored_x_left, restored_y_left)
axis[0, 1].plot(restored_x_right, restored_y_right)
axis[0, 1].grid()
axis[0, 1].set_title(f'Restored track sides using L2 distance parameter. {t2 - t1} s')

plt.show()