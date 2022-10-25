import numpy as np
from matplotlib import pyplot as plt

import splines

"""
Testing the potential solution of using parametric curves
"""

if __name__ == "__main__":
    
    #generate random samples
    increments = np.random.rand(30)
    x = np.empty(*increments.shape)

    x[0] = 0

    for i in range(1, len(increments)):
        x[i] = x[i - 1] + increments[i]

    y = np.random.randn(*x.shape) * 4

    #assume parametrisation where t = sample No.

    t = np.arange(0, len(x), 1)

    x_spline = splines.CubicSpline(t, x)
    y_spline = splines.CubicSpline(t, y)

    t_plot = np.arange(0, max(t), 0.01)

    x_plot = np.empty(*t_plot.shape)
    y_plot = np.empty(*t_plot.shape)

    for index, item in enumerate(t_plot):
        x_plot[index] = x_spline.get(item)
        y_plot[index] = y_spline.get(item)

    plt.scatter(t, x)
    plt.scatter(t, y)
    plt.plot(t_plot, x_plot, label="x_plot")
    plt.plot(t_plot, y_plot, label="y_plot")
    plt.legend()
    plt.show()

    plt.scatter(x, y)
    plt.plot(x_plot, y_plot)

    plt.show()