import numpy as np
from matplotlib import pyplot as plt
from time import time
import math

import generate_track
import splines

"""
The code for a discarded idea of using same-indices points (despite the offsets) to find the centerline. Averages two points.
"""

fig, axis = plt.subplots(2, 2)

cones_left_x, cones_left_y, cones_right_x, cones_right_y = generate_track.generate()

axis[0, 0].scatter(cones_left_x, cones_left_y, color='red')
axis[0, 0].scatter(cones_right_x, cones_right_y, color='blue')
axis[0, 0].set_title('cones')
axis[0, 0].grid()

t1 = time()

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

t5 = time()

for index, value in enumerate(t_plot_left):
    restored_x_left[index] = left_x_spline.get(value)
    restored_y_left[index] = left_y_spline.get(value)

for index, value in enumerate(t_plot_right):
    restored_x_right[index] = right_x_spline.get(value)
    restored_y_right[index] = right_y_spline.get(value)

print(f'Restoring the values cost {time() - t5} s')

t2 = time()

axis[0, 1].plot(restored_x_left, restored_y_left)
axis[0, 1].plot(restored_x_right, restored_y_right)
axis[0, 1].grid()
axis[0, 1].set_title(f'Restored track sides using L2 distance parameter. {t2 - t1} s')

axis[1, 0].plot(restored_x_left, restored_y_left, color='gray')
axis[1, 0].plot(restored_x_right, restored_y_right, color='gray')
axis[1, 1].plot(restored_x_left, restored_y_left, color='gray')
axis[1, 1].plot(restored_x_right, restored_y_right, color='gray')
axis[1, 1].grid()
# Raceline code...

assert(len(cones_left_x) == len(cones_right_x))
assert(len(cones_left_y) == len(cones_right_y))

# Assume the centerline lies between two spline points. Interpolate centerline points

# The algorithm will use the track length as a parameter, so it must be recalculated to provide accurate benchmarks

for index in range(1, len(left_param)):
    left_param[index] = left_param[index - 1]
    left_param[index] += math.sqrt((cones_left_x[index] - cones_left_x[index - 1])**2 + \
        (cones_left_y[index] - cones_left_y[index - 1])**2)

for index in range(1, len(right_param)):
    right_param[index] = right_param[index - 1]
    right_param[index] += math.sqrt((cones_right_x[index] - cones_right_x[index - 1])**2 + \
        (cones_right_y[index] - cones_right_y[index - 1])**2)

track_length_increment = int(max(max(left_param), max(right_param))) * 4
print(track_length_increment)

left_points = np.linspace(0, max(left_param), track_length_increment)
right_points = np.linspace(0, max(right_param), track_length_increment)

centerline_x = np.empty(*left_points.shape)
centerline_y = np.empty(*right_points.shape)

for index in range(len(centerline_x)):
    centerline_x[index] = (right_x_spline.get(right_points[index]) + left_x_spline.get(left_points[index])) / 2
    centerline_y[index] = (right_y_spline.get(right_points[index]) + left_y_spline.get(left_points[index])) / 2
    if not index % 50:
        x_arr = [right_x_spline.get(right_points[index]), left_x_spline.get(left_points[index])]
        y_arr = [right_y_spline.get(right_points[index]), left_y_spline.get(left_points[index])]
        axis[1, 0].plot(x_arr, y_arr)


axis[1, 0].plot(centerline_x, centerline_y, color='red')
axis[1, 0].set_title('Track progression extrapolation algorithm')
axis[1, 0].grid()


centerline_x_triangles = np.empty(*left_points.shape)
centerline_y_triangles = np.empty(*right_points.shape)

assert(len(centerline_x_triangles) == len(centerline_y_triangles))

interval = 50

for index in range(len(centerline_x)):
    Pa = (right_x_spline.get(right_points[index]), right_y_spline.get(right_points[index]))
    Pb = (right_x_spline.get(right_points[index - interval]), right_y_spline.get(right_points[index - interval]))
    Pc = (left_x_spline.get(left_points[index]), left_y_spline.get(left_points[index]))
    assert(Pa != Pb)
    assert(Pa != Pc)
    assert(Pb != Pc)
    tangent_x_Pc = left_x_spline.get(left_points[index]) - left_x_spline.get(left_points[index - 1]) / (track_length_increment)
    tangent_y_Pc = left_y_spline.get(left_points[index]) - left_x_spline.get(left_points[index - 1]) / track_length_increment

    a_b = math.sqrt((Pa[0] - Pb[0])**2 + (Pa[1] - Pb[1])**2)
    a_c = math.sqrt((Pa[0] - Pc[0])**2 + (Pa[1] - Pc[1])**2)
    b_c = math.sqrt((Pc[0] - Pb[0])**2 + (Pc[1] - Pb[1])**2)

    s = 0.5 * (a_b + b_c + a_c)
    triangle_area = math.sqrt(s * (s - a_b)  * (s - a_c) * (s - b_c))   #Heron's formula

    track_width_at_Pc = 2 * triangle_area / a_b
    parallel_vector_at_Pc = NotImplemented


    if not index % 50:
        plt.plot([Pa[0], Pb[0]], [Pa[1], Pb[1]], color='black')
        plt.plot([Pb[0], Pc[0]], [Pb[1], Pc[1]], color='black')
        plt.plot([Pa[0], Pc[0]], [Pa[1], Pc[1]], color='black')

plt.show()