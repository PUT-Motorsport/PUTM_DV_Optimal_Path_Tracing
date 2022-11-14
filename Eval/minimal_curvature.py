import math
import numpy as np
from matplotlib import pyplot as plt
from time import time
'''
Designing and testing the minimal curvature algorithms
'''


def scipys_curvature_calc(x, y):
    parameter = np.empty_like(x)
    parameter[0] = 0

    from time import time
    t1 = time()

    for index in range(1, len(parameter)):
        from utility_fcn import calculate_distance
        parameter[index] = parameter[index - 1]
        # parameter[index] += calculate_distance(x[index], y[index], x[index - 1], y[index])
        parameter[index] += math.sqrt((x[index] - x[index - 1]) ** 2 + (y[index] - y[index - 1]) ** 2)

    from scipy.interpolate import CubicSpline
    x_spline = CubicSpline(parameter, x, bc_type='natural')
    y_spline = CubicSpline(parameter, y, bc_type='natural')
    x_prime = x_spline.derivative()
    y_prime = y_spline.derivative()
    x_bis = x_prime.derivative()
    y_bis = y_prime.derivative()

    t_plot = np.arange(0, max(parameter), 1)

    total_curvature_squared = 0
    for _index in range(len(x)):
        from utility_fcn import curvature_squared
        val = parameter[_index]
        curvature = abs(curvature_squared(x_prime(val), y_prime(val), x_bis(val), y_bis(val)))
        assert curvature >= 0
        total_curvature_squared += curvature

    return total_curvature_squared

    plt.plot(x_spline(t_plot), y_spline(t_plot))
    plt.grid()
    plt.title(f'Curvature: {total_curvature_squared} in {time() - t1} s')
    plt.show()


'''
Approach 2 based on: DOI10.1080/00423114.2019.1631455
'''
def TUMFTMs_minimal_curvature(x, y, left_width, right_width):

    parameter = np.empty_like(x)
    parameter[0] = 0

    for index in range(1, len(parameter)):
        from utility_fcn import calculate_distance
        parameter[index] = parameter[index - 1]
        # parameter[index] += calculate_distance(x[index], y[index], x[index - 1], y[index])
        parameter[index] += math.sqrt((x[index] - x[index - 1])**2 + (y[index] - y[index - 1])**2)
    import splines
    x_spline = splines.CubicSpline(parameter, x)
    y_spline = splines.CubicSpline(parameter, y)

    t_plot = np.arange(0, max(parameter), 1)
    x_interpolated = x_spline.get_range(t_plot)
    y_interpolated = y_spline.get_range(t_plot)

    plt.plot(x_interpolated, y_interpolated, label='Spline')
    plt.plot(x, y, label='Original')
    plt.legend()
    plt.grid()
    plt.show()



    _start = time()
    total_curvature_squared = 0
    for i in range(len(t_plot) - 1):
        x_prime = x_spline.get_derivative_at(t_plot[i])
        x_bis = x_spline.get_second_derivative_at(t_plot[i])
        y_prime = y_spline.get_derivative_at(t_plot[i])
        y_bis = y_spline.get_second_derivative_at(t_plot[i])

        from utility_fcn import curvature_squared
        total_curvature_squared += curvature_squared(x_prime, y_prime, x_bis, y_bis)

    return total_curvature_squared


if __name__ == '__main__':
    import centerline_search

    interpolation_points = [1000, 2000, 3000]
    tries = 20
    results = np.empty((len(interpolation_points), tries))

    for i in range(len(interpolation_points)):
        for j in range(tries):
            centerline_x, centerline_y, centerline_track_width_left, centerline_track_width_right = centerline_search.get_centerline(
                interpolation_points[i])
            results[i, j] = scipys_curvature_calc(centerline_x, centerline_y)
        plt.plot(results[i])

    plt.grid()
    plt.show()
    print(results)