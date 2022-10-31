import numpy as np
from matplotlib import pyplot as plt
import math
import generate_track
import splines

cones_left_x, cones_left_y, cones_right_x, cones_right_y = generate_track.generate()

plt.scatter(cones_left_x, cones_left_y, color='blue')
plt.scatter(cones_right_x, cones_right_y, color='red')

left_param = np.empty(*cones_left_x.shape)
right_param = np.empty(*cones_right_x.shape)

left_param[0] = 0
right_param[0] = 0

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

t_plot_left = np.arange(0, max(left_param), 0.1)
t_plot_right = np.arange(0, max(right_param), 0.1)

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

plt.plot(restored_x_left, restored_y_left, color='gray')
plt.plot(restored_x_right, restored_y_right, color='gray')

interval = 100
for index in range(0, min(len(restored_x_left), len(restored_x_right)), interval):
    plt.plot([restored_x_left[index], restored_x_right[index]], [restored_y_left[index], restored_y_right[index]])

plt.show()
