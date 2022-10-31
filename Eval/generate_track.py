import numpy as np
import pandas as pd
"""
Returns a tuple of x_left, y_left, x_right, y_right cone coordinates
"""

def generate():
    from os import getcwd
    df = pd.read_csv(getcwd() + '/Eval/tracks/berlin_2018.csv')

    x_middle = df['x_m'].to_numpy()
    y_middle = df['y_m'].to_numpy()

    assert(len(x_middle) == len(y_middle))

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
        
        if x_middle[(index + 1) % len(x_middle)] != x_middle[index]:
            derivative = (y_middle[(index + 1) % len(y_middle)] - y_middle[index]) / (x_middle[(index + 1) % len(x_middle)] - x_middle[index])
        else:
            from sys import float_info
            derivative = float_info.max
        if derivative != 0:
            parallel_slope = (-1) / derivative
        else:
            parallel_slope = 1
        # find a point that is right_offset away, whoose slope is parallel_slope

        # from delta y / delta x = parallel slope
        from math import sqrt
        delta_X = sqrt((left_offset[index])**2 / (1 + parallel_slope**2))
        x_left[index] = x_middle[index] - delta_X
        y_left[index] = y_middle[index] - delta_X * parallel_slope

        delta_X = sqrt((right_offset[index])**2 / (1 + parallel_slope**2))
        x_right[index] = x_middle[index] + delta_X
        y_right[index] = y_middle[index] + delta_X * parallel_slope

        # fix switched sides

        tangent_vector = np.array([x_middle[(index + 1) % len(x_middle)] - x_middle[index], (y_middle[(index + 1) % len(x_middle)] - y_middle[index])])
        right_vector = np.array([x_right[index] - x_middle[index], y_right[index] - y_middle[index]])

        cross = np.cross(right_vector, tangent_vector)

        if (cross < 0):     # cross product negative, points must be switched
            x_left[index], x_right[index] = x_right[index], x_left[index]
            y_left[index], y_right[index] = y_right[index], y_left[index] 

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

    return cones_left_x, cones_left_y, cones_right_x, cones_right_y