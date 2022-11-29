import numpy as np
from matplotlib import pyplot as plt
import math
import splines
import utility_fcn
import generate_track

"""
Finding the centerline by searching the other side of the track for the closest point. Takes ~3.5 s. Further optimisations possible.
"""


def get_centerline(interpolated_points_count: int):
    cones_left_x, cones_left_y, cones_right_x, cones_right_y = generate_track.generate()

    from time import time
    t1 = time()
    left_param = np.empty(*cones_left_x.shape)
    right_param = np.empty(*cones_right_x.shape)

    left_param[0] = 0  # first cone distance is 0
    right_param[0] = 0

    # calculate L2 distance as a parameter
    for index in range(1, len(left_param)):
        left_param[index] = left_param[index - 1]
        left_param[index] += math.sqrt((cones_left_x[index] - cones_left_x[index - 1]) ** 2 + \
                                       (cones_left_y[index] - cones_left_y[index - 1]) ** 2)

    for index in range(1, len(right_param)):
        right_param[index] = right_param[index - 1]
        right_param[index] += math.sqrt((cones_right_x[index] - cones_right_x[index - 1]) ** 2 + \
                                        (cones_right_y[index] - cones_right_y[index - 1]) ** 2)

    # calculate parametric splines
    right_x_spline = splines.CubicSpline(right_param, cones_right_x)
    right_y_spline = splines.CubicSpline(right_param, cones_right_y)
    left_x_spline = splines.CubicSpline(left_param, cones_left_x)
    left_y_spline = splines.CubicSpline(left_param, cones_left_y)

    t_plot_left = np.linspace(0, max(left_param), interpolated_points_count)
    t_plot_right = np.linspace(0, max(right_param), interpolated_points_count)

    restored_x_left = left_x_spline.get_range(t_plot_left)
    restored_y_left = left_y_spline.get_range(t_plot_left)
    restored_x_right = right_x_spline.get_range(t_plot_right)
    restored_y_right = right_y_spline.get_range(t_plot_right)

    # # calculate track curvature for both sides as a check
    # total_left_curvature = 0
    # total_right_curvature = 0
    # for index in range(len(t_plot_left)):
    #     right_x_prime = right_x_spline.get_derivative_at(t_plot_right[index])
    #     right_x_bis = right_x_spline.get_second_derivative_at(t_plot_right[index])
    #     right_y_prime = right_y_spline.get_derivative_at(t_plot_right[index])
    #     right_y_bis = right_y_spline.get_second_derivative_at(t_plot_right[index])
    #
    #     from utility_fcn import curvature_squared
    #     total_right_curvature += curvature_squared(right_x_prime, right_y_prime, right_x_bis, right_y_bis)
    #
    #     left_x_prime = left_x_spline.get_derivative_at(t_plot_left[index])
    #     left_x_bis = left_x_spline.get_second_derivative_at(t_plot_left[index])
    #     left_y_prime = left_y_spline.get_derivative_at(t_plot_left[index])
    #     left_y_bis = left_y_spline.get_second_derivative_at(t_plot_left[index])
    #     total_left_curvature += curvature_squared(left_x_prime, left_y_prime, left_x_bis, left_y_bis)
    #
    # print(f'Left: {total_left_curvature}, right: {total_right_curvature}')

    # print(f'Spline interpolation took {time() - t1} s')

    midpoint_search_time = time()

    discarding_ratio = 1
    searched_left_x = restored_x_left[::discarding_ratio]
    searched_left_y = restored_y_left[::discarding_ratio]
    searched_right_x = restored_x_right[::discarding_ratio]
    searched_right_y = restored_y_right[::discarding_ratio]

    centerline_x = np.empty(*searched_right_x.shape)
    centerline_y = np.empty(*searched_right_y.shape)
    centerline_track_width_left = np.empty_like(centerline_x)
    centerline_track_width_right = np.empty_like(centerline_track_width_left)

    assert (len(searched_left_x) == len(searched_right_y))

    for index in range(len(searched_right_x)):
        minimal_distance = 10000000
        minimal_index = -1

        for left_point in range(len(searched_left_x)):
            distance = utility_fcn.calculate_distance(searched_right_x[index], searched_right_x[index], searched_left_x[index], searched_left_y[index])
            if distance < minimal_distance:
                minimal_index = index
                minimal_distance = distance

        if minimal_index == -1:
            raise ValueError

        centerline_track_width_right[index] = minimal_distance / 2
        centerline_track_width_left[index] = minimal_distance / 2

        centerline_x[index] = (searched_right_x[index] + searched_left_x[minimal_index]) / 2
        centerline_y[index] = (searched_right_y[index] + searched_left_y[minimal_index]) / 2

    # print(f'Interpolated {len(centerline_x)} centerline points in {time() - midpoint_search_time} s')
    return centerline_x, centerline_y, centerline_track_width_left, centerline_track_width_right


if __name__ == '__main__':
    from time import time
    t1 = time()
    x, y, left_width, right_width = get_centerline(1000)
    plt.grid()
    plt.plot(x, y)
    plt.title(f"{time() - t1}")
    plt.show()
