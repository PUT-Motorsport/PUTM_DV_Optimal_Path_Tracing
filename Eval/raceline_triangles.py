import numpy as np
from matplotlib import pyplot as plt
import math
import generate_track
import splines
import utility_fncs

"""
The code for a discarded idea of using same-indices points (despite the offsets) to find the centerline. Uses triangles' 
geometric properties to find the centerline
"""

cones_left_x, cones_left_y, cones_right_x, cones_right_y = generate_track.generate()

plt.scatter(cones_left_x, cones_left_y, color='blue')
plt.scatter(cones_right_x, cones_right_y, color='red')

plt.show()

from time import time
t1 = time()
left_param = np.empty(*cones_left_x.shape)
right_param = np.empty(*cones_right_x.shape)

left_param[0] = 0   # first cone distance is 0
right_param[0] = 0

# calculate L2 distance as a parameter
for index in range(1, len(left_param)):
    left_param[index] = left_param[index - 1]
    left_param[index] += math.sqrt((cones_left_x[index] - cones_left_x[index - 1])**2 + \
        (cones_left_y[index] - cones_left_y[index - 1])**2)

for index in range(1, len(right_param)):
    right_param[index] = right_param[index - 1]
    right_param[index] += math.sqrt((cones_right_x[index] - cones_right_x[index - 1])**2 + \
        (cones_right_y[index] - cones_right_y[index - 1])**2)

# calculate parametric splines
right_x_spline = splines.CubicSpline(right_param, cones_right_x)
right_y_spline = splines.CubicSpline(right_param, cones_right_y)
left_x_spline = splines.CubicSpline(left_param, cones_left_x)
left_y_spline = splines.CubicSpline(left_param, cones_left_y)

t_plot_left = np.arange(0, max(left_param), 0.1)
t_plot_right = np.arange(0, max(right_param), 0.1)

restored_x_left = left_x_spline.get_range(t_plot_left)
restored_y_left = left_y_spline.get_range(t_plot_left)
restored_x_right = right_x_spline.get_range(t_plot_right)
restored_y_right = right_y_spline.get_range(t_plot_right)

plt.plot(restored_x_left, restored_y_left, color='gray')
plt.plot(restored_x_right, restored_y_right, color='lightgray')

plt.title(f'{time() - t1} s')
plt.show()

# find discretization interval for left and right side
points = 2500
left_points = np.linspace(0, max(left_param), points)
dt_left = left_points[1]
right_points = np.linspace(0, max(right_param), points)
dt_right = right_points[1]
assert(len(right_points) == len(left_points))

centerline_x = np.empty(*left_points.shape)
centerline_y = np.empty(*centerline_x.shape)

indices_difference = 20

for index in range(len(left_points)):

    # find 3 points to form a triangle
    a = (right_x_spline.get(right_points[index]), right_y_spline.get(right_points[index]))
    b = (left_x_spline.get(left_points[index - indices_difference]), left_y_spline.get(left_points[index - indices_difference]))
    c = (left_x_spline.get(left_points[index]), left_y_spline.get(left_points[index]))
    assert(a != c)
    assert(b != c)
    assert(b != c)
    a_b = utility_fncs.calculate_distance_point(a, b)
    a_c = utility_fncs.calculate_distance_point(a, c)
    b_c = utility_fncs.calculate_distance_point(b, c)

    # Heron's formula
    s = (a_b + b_c + a_c) / 2
    triangle_area = math.sqrt(s * (s - a_b) * (s - a_c) * (s - b_c))
    track_width_at_a = 2 * triangle_area / b_c

    x_tangent_vector_at_a = (right_x_spline.get(right_points[(index + 1) % len(left_points)] - right_x_spline.get(right_points[index - 1]))) / 2 / dt_right
    y_tangent_vector_at_a = (right_y_spline.get(right_points[(index + 1) % len(right_points)]) - right_y_spline.get(right_points[index - 1])) / 2 / dt_right

    perpendicular_vector_at_a = utility_fncs.left_side_perpendicular_vector(x_tangent_vector_at_a, y_tangent_vector_at_a)
    unit_perpendicular_vector_at_a = utility_fncs.unit_vector(perpendicular_vector_at_a[0], perpendicular_vector_at_a[1])

    centerline_x[index] = a[0] + unit_perpendicular_vector_at_a[0] * track_width_at_a / 2
    centerline_y[index] = a[1] + unit_perpendicular_vector_at_a[1] * track_width_at_a / 2

    if not index % 50:
        # print(f'a_b: {a_b}, a_c: {a_c}, b_c: {b_c}')
        print(f'Track width: {track_width_at_a}, a_c:{a_c}, triangle_area: {triangle_area}')
        plt.plot([a[0], b[0]], [a[1], b[1]], color='pink')
        plt.plot([a[0], c[0]], [a[1], c[1]], color='green')
        plt.scatter(centerline_x[index], centerline_y[index], color='brown')
        #plt.plot([b[0], c[0]], [b[1], c[1]], color='blue')
        
plt.grid()
plt.plot(centerline_x[::50], centerline_y[::50], color='red')
x_m, y_m = generate_track.get_track_centerline()
plt.plot(x_m, y_m, color='gray')
plt.plot(restored_x_left, restored_y_left, color='black')
plt.plot(restored_x_right, restored_y_right, color='black')
plt.title(f'Total cost: {time() - t1}')
plt.show()
